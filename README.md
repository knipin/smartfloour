app_main.cpp
```
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <sys/time.h> // Для получения времени для dt
#include <math.h>     // Для fabsf

// Matter Includes
#include <app/server/Server.h>
#include <app/util/attribute-storage.h>
#include <esp_matter.h>
#include <esp_matter_attribute_utils.h>
#include <esp_matter_cluster.h>
#include <esp_matter_core.h>
#include <esp_matter_endpoint.h>
#include <esp_matter_identify.h>

// Driver Includes
#include <driver/gpio.h>
#include <driver/ledc.h>             // Для PWM (LEDC)
#include <esp_adc/adc_oneshot.h>     // Новый API АЦП
#include <esp_adc/adc_cali.h>         // API калибровки
#include <esp_adc/adc_cali_scheme.h>  // Схемы калибровки

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::cluster;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static const char *TAG = "APP_MAIN_PID_AD22100";

// --- Configuration ---
#define HEATER_PWM_GPIO_PIN GPIO_NUM_10 // <--- ЗАМЕНИ НА СВОЙ GPIO ПИН для ШИМ
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // 10 бит = 0-1023
#define LEDC_FREQUENCY (5000)         // 5 кГц

#define PID_LOOP_INTERVAL_MS (2000)   // Интервал ПИД (2 секунды)

// --- ADC Configuration ---
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_4       // <--- ЗАМЕНИ, если GPIO другой (GPIO4 -> Channel 4)
#define ADC_ATTEN       ADC_ATTEN_DB_11     // <--- Аттенюация (11dB -> ~0-3.1V/3.3V) - ПРОВЕРЬ ДИАПАЗОН Vout!
// --- ВАЖНО: Укажи реальное напряжение питания (Vs) датчика AD22100KTZ в Вольтах! ---
// --- Если используешь 5V, НУЖЕН ДЕЛИТЕЛЬ НАПРЯЖЕНИЯ на Vout перед АЦП ESP32! ---
#define SENSOR_VCC      (3.3f)

// --- PID Configuration (ТРЕБУЕТ ТЩАТЕЛЬНОЙ НАСТРОЙКИ!) ---
#define PID_KP 10.0f // Пропорциональный
#define PID_KI 0.1f  // Интегральный
#define PID_KD 0.5f  // Дифференциальный

#define PID_OUTPUT_MAX 100.0f // Макс. выход ПИД (%)
#define PID_OUTPUT_MIN 0.0f   // Мин. выход ПИД (%)
// Ограничение интегральной суммы (для anti-windup, можно подстроить)
const float PID_INTEGRAL_MAX = (PID_KI > 1e-6) ? (PID_OUTPUT_MAX / PID_KI * 0.8f) : 1000.0f;
const float PID_INTEGRAL_MIN = (PID_KI > 1e-6) ? (PID_OUTPUT_MIN / PID_KI * 0.8f) : -1000.0f;


// --- Global Variables ---
static node_t *matter_node;
static endpoint_t *thermostat_endpoint;

// ADC & Calibration Handles
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc_calibration_ok = false;

// PID State
static float pid_integral = 0.0f;
static float pid_previous_error = NAN; // Используем NAN как флаг первого запуска
static int64_t pid_last_time_us = 0;

// Matter Thermostat State
static int16_t current_local_temperature_matter = 2000; // Начальное 20.00 C
static int16_t heating_setpoint_matter = 2200;          // 22.00 C
static uint8_t system_mode = Thermostat::SystemModeEnum::kOff; // Default off

// --- Forward Declarations ---
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data);
static void update_thermostat_running_state();
float read_temperature_celsius(); // Объявим здесь

// --- Hardware Control (PWM) ---
esp_err_t pwm_heater_init() {
    ESP_LOGI(TAG, "Initializing LEDC PWM for heater control on GPIO %d", HEATER_PWM_GPIO_PIN);
    ledc_timer_config_t ledc_timer = { .speed_mode = LEDC_MODE, .duty_resolution = LEDC_DUTY_RES, .timer_num = LEDC_TIMER, .freq_hz = LEDC_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) { ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err)); return err; }
    ledc_channel_config_t ledc_channel = { .gpio_num = HEATER_PWM_GPIO_PIN, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = 0, .hpoint = 0, .flags = { .output_invert = 0 } };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) { ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(err)); }
    else { ESP_LOGI(TAG, "LEDC PWM initialized"); }
    return err;
}

void set_heater_power(float power_percentage) {
    if (power_percentage > 100.0f) power_percentage = 100.0f;
    if (power_percentage < 0.0f) power_percentage = 0.0f;
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;
    uint32_t duty_cycle = (uint32_t)((power_percentage / 100.0f) * max_duty);
    esp_err_t err = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle);
    if (err == ESP_OK) {
        err = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        if (err != ESP_OK) { ESP_LOGE(TAG, "LEDC update duty failed: %s", esp_err_to_name(err)); }
    } else { ESP_LOGE(TAG, "LEDC set duty failed: %s", esp_err_to_name(err)); }
    // ESP_LOGD(TAG, "Set heater power: %.1f%% -> Duty: %lu", power_percentage, duty_cycle);
}

// --- ADC Initialization and Calibration ---
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "ADC calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = { .unit_id = unit, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) { calibrated = true; }
    else { ESP_LOGE(TAG, "Failed Line Fitting cali: %s", esp_err_to_name(ret)); }
#else
    ESP_LOGW(TAG, "ADC Calibration scheme Line Fitting not supported");
#endif
    *out_handle = handle;
    if (ret == ESP_OK) { ESP_LOGI(TAG, "ADC Calibration Success"); }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) { ESP_LOGW(TAG, "ADC Calibration scheme not supported or eFuse not burnt"); }
    else { ESP_LOGE(TAG, "ADC Invalid arg or no memory"); }
    return calibrated;
}

esp_err_t temperature_sensor_init() {
    ESP_LOGI(TAG, "Initializing ADC for temperature sensor");
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT, .ulp_mode = ADC_ULP_MODE_DISABLE };
    esp_err_t err = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err)); return err; }
    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN };
    err = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config);
    if (err != ESP_OK) { ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err)); return err; }
    adc_calibration_ok = adc_calibration_init(ADC_UNIT, ADC_ATTEN, &adc1_cali_handle);
    if (!adc_calibration_ok) {
         ESP_LOGE(TAG, "!!!!!!!!!! ADC CALIBRATION FAILED! TEMPERATURE READINGS WILL BE INACCURATE !!!!!!!!!!");
    }
    return ESP_OK;
}

// --- Temperature Sensor Reading ---
float read_temperature_celsius() {
    int adc_raw;
    int voltage_mv = -1; // Initialize to invalid value

    esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return NAN; // Return Not-a-Number on ADC read failure
    }

    if (adc_calibration_ok && adc1_cali_handle != NULL) {
        ret = adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_mv);
        if (ret != ESP_OK) {
             ESP_LOGE(TAG, "ADC calibration to voltage failed (raw %d): %s", adc_raw, esp_err_to_name(ret));
             return NAN; // Return Not-a-Number if calibration conversion fails
        }
    } else {
         // No calibration - calculation will be inaccurate
         // Return NaN or implement a rough estimation if absolutely needed, but calibration is key.
         ESP_LOGE(TAG, "ADC not calibrated! Cannot calculate accurate temperature.");
         return NAN;
    }

    // If voltage_mv is still -1, something went wrong
    if (voltage_mv < 0) {
         ESP_LOGE(TAG, "Failed to get valid voltage from ADC.");
         return NAN;
    }

    float voltage_v = (float)voltage_mv / 1000.0f;

    // Calculate temperature using the AD22100KTZ formula:
    // T_A = ((Vout * 5.0 / Vs) - 1.375) / 0.0225
    float temperature_c = ((voltage_v * 5.0f / SENSOR_VCC) - 1.375f) / 0.0225f;

    ESP_LOGD(TAG, "ADC Raw: %d, Voltage: %.3f V (Vs=%.1fV), Temp: %.2f C", adc_raw, voltage_v, SENSOR_VCC, temperature_c);

    // Check against the datasheet range (-50 to +150 C)
    if (temperature_c < -50.0f || temperature_c > 150.0f) {
        ESP_LOGW(TAG, "Temperature reading out of datasheet range: %.2f C. Check wiring/VCC/divider.", temperature_c);
        return NAN; // Return Not-a-Number if reading is out of sensor's specified range
    }

    return temperature_c;
}


// --- PID Controller Logic ---
float compute_pid(float setpoint_c, float current_temp_c) {
    int64_t current_time_us = esp_timer_get_time();
    float dt = 0.0f;
    if (pid_last_time_us != 0) { dt = (float)(current_time_us - pid_last_time_us) / 1000000.0f; }
    pid_last_time_us = current_time_us;

    // If dt is invalid or it's the first run, skip PID calculation this cycle
    if (dt <= 0.0f || isnan(pid_previous_error)) {
        pid_previous_error = setpoint_c - current_temp_c; // Initialize error for next D term calculation
        return 0.0f; // Output 0 if dt is invalid or first run
    }

    float error = setpoint_c - current_temp_c;
    float p_term = PID_KP * error;

    pid_integral += error * dt;
    if (pid_integral > PID_INTEGRAL_MAX) pid_integral = PID_INTEGRAL_MAX;
    else if (pid_integral < PID_INTEGRAL_MIN) pid_integral = PID_INTEGRAL_MIN;
    float i_term = PID_KI * pid_integral;

    float error_diff = error - pid_previous_error;
    float d_term = PID_KD * (error_diff / dt);

    float output = p_term + i_term + d_term;
    if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
    else if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;
    pid_previous_error = error;

    ESP_LOGD(TAG, "PID: SP=%.2f, PV=%.2f, Err=%.2f, P=%.2f, I=%.2f(Sum=%.2f), D=%.2f -> Out=%.2f",
             setpoint_c, current_temp_c, error, p_term, i_term, pid_integral, d_term, output);

    return output;
}

void reset_pid_state() {
    pid_integral = 0.0f;
    pid_previous_error = NAN; // Reset to NAN for first run condition in compute_pid
    pid_last_time_us = 0;
    ESP_LOGI(TAG, "PID state reset.");
}

// --- Matter Logic ---
void update_thermostat_running_state() {
    if (!thermostat_endpoint) return;
    cluster_t *thermostat_cluster = cluster::get(thermostat_endpoint, Thermostat::Id);
    if (!thermostat_cluster) return;
    uint16_t running_state = (system_mode == Thermostat::SystemModeEnum::kHeat) ? (1 << 0) : 0; // kHeat bit
    esp_matter_attr_val_t running_state_val = esp_matter_bitmap16(running_state);
    // Update only if changed to reduce traffic/logs
    uint16_t current_running_state = 0;
    attribute::get_val_raw(attribute::get(thermostat_cluster, Thermostat::Attributes::ThermostatRunningState::Id),
                           (uint8_t*)¤t_running_state, sizeof(current_running_state));
    if (current_running_state != running_state) {
        attribute::update(endpoint::get_id(thermostat_endpoint), Thermostat::Id, Thermostat::Attributes::ThermostatRunningState::Id, &running_state_val);
        ESP_LOGI(TAG, "Updated ThermostatRunningState to: %d (Mode: %s)", running_state, (system_mode == Thermostat::SystemModeEnum::kHeat) ? "Heat" : "Off");
    }
}

void pid_control_task(void *pvParameters) {
    pid_last_time_us = esp_timer_get_time(); // Initialize time
    reset_pid_state(); // Initialize PID state variables

    while (true) {
        float current_temp_c = read_temperature_celsius();

        // Handle invalid temperature readings
        if (isnan(current_temp_c)) {
             ESP_LOGE(TAG, "Invalid temperature reading (NaN), turning heater off.");
             set_heater_power(0.0f);
             // Optionally update Matter attribute to indicate invalid temp (e.g., a specific value or status)
        } else {
            current_local_temperature_matter = (int16_t)(current_temp_c * 100.0f);

            // Update Matter attribute
            if (thermostat_endpoint) {
                cluster_t *thermostat_cluster = cluster::get(thermostat_endpoint, Thermostat::Id);
                if (thermostat_cluster) {
                    esp_matter_attr_val_t temp_val = esp_matter_int16(current_local_temperature_matter);
                    // Update only if changed significantly to reduce logs/traffic
                    int16_t old_temp_matter = 0; // Default to 0 or read current value if needed
                     attribute::get_val_raw(attribute::get(thermostat_cluster, Thermostat::Attributes::LocalTemperature::Id), (uint8_t*)&old_temp_matter, sizeof(old_temp_matter));
                    // Update if difference is more than ~0.1 degree (10 in Matter units)
                    if (fabsf((float)current_local_temperature_matter - (float)old_temp_matter) >= 10.0f) {
                        attribute::update(endpoint::get_id(thermostat_endpoint), Thermostat::Id, Thermostat::Attributes::LocalTemperature::Id, &temp_val);
                        ESP_LOGI(TAG, "Updated LocalTemperature to %.2f C (%d)", current_temp_c, current_local_temperature_matter);
                    }
                }
            }

            // Run PID and control heater only if system mode is Heat
            float pid_output = 0.0f;
            if (system_mode == Thermostat::SystemModeEnum::kHeat) {
                float setpoint_c = (float)heating_setpoint_matter / 100.0f;
                pid_output = compute_pid(setpoint_c, current_temp_c);
                ESP_LOGI(TAG, "Current Temp: %.2f C, Setpoint: %.2f C, PID Output: %.1f %%", current_temp_c, setpoint_c, pid_output);
            } else {
                // If mode is Off, ensure PID state is reset (only once)
                if (!isnan(pid_previous_error)) { // Check if PID was running before
                     reset_pid_state();
                }
            }
            set_heater_power(pid_output);
        }

        // Update running state based on system mode (doesn't depend on temp validity)
        update_thermostat_running_state();

        vTaskDelay(pdMS_TO_TICKS(PID_LOOP_INTERVAL_MS));
    }
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data) {
    if (type == POST_UPDATE) {
        uint16_t thermostat_ep_id = endpoint::get_id(thermostat_endpoint);
        ESP_LOGI(TAG, "POST_UPDATE: EP:%d(Expected:%d), Cl:0x%lx, At:0x%lx", endpoint_id, thermostat_ep_id, cluster_id, attribute_id);

        if (endpoint_id == thermostat_ep_id && cluster_id == Thermostat::Id) {
            if (attribute_id == Thermostat::Attributes::SystemMode::Id && val->type == ESP_MATTER_VAL_TYPE_ENUM8) {
                uint8_t new_mode = val->val.u8;
                if (system_mode != new_mode) {
                    system_mode = new_mode;
                    ESP_LOGI(TAG, "SystemMode changed to: %d", system_mode);
                    if (system_mode != Thermostat::SystemModeEnum::kHeat) {
                        reset_pid_state(); // Reset PID when turning off
                        set_heater_power(0.0f); // Ensure heater is off
                    }
                    update_thermostat_running_state(); // Update state immediately
                }
            } else if (attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id && val->type == ESP_MATTER_VAL_TYPE_INT16) {
                 if (heating_setpoint_matter != val->val.i16) { // Reset PID only if setpoint really changed
                    reset_pid_state(); // Reset PID on setpoint change to avoid windup/jumps
                    heating_setpoint_matter = val->val.i16;
                    ESP_LOGI(TAG, "OccupiedHeatingSetpoint changed to: %d (%.2f C)", heating_setpoint_matter, (float)heating_setpoint_matter / 100.0f);
                 }
            }
        }
    }
    return ESP_OK;
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data) {
    ESP_LOGI(TAG, "Identification callback: type: %d, ep: %d, effect: %d", type, endpoint_id, effect_id);
    if (type == identification::callback_type_t::START && endpoint_id == endpoint::get_id(thermostat_endpoint)) {
         // Blink heater at 50% power? Careful not to interfere too much with PID task.
         // A simpler approach might be blinking an actual LED if available.
         // For now, just log it.
         ESP_LOGW(TAG, "Identify requested for thermostat endpoint!");
    }
    return ESP_OK;
}

extern "C" void app_main() {
    esp_err_t err = ESP_OK;
    /* Initialize NVS */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) { ESP_ERROR_CHECK(nvs_flash_erase()); err = nvs_flash_init(); }
    ESP_ERROR_CHECK(err);

    /* Initialize Hardware */
    err = pwm_heater_init(); ESP_ERROR_CHECK(err);
    err = temperature_sensor_init(); ESP_ERROR_CHECK(err); // Init ADC and Calibration

    /* Create Matter Node */
    node::config_t node_config;
    matter_node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ESP_ABORT_IF_FALSE(matter_node, "Failed to create Matter node");

    /* Create Thermostat Endpoint */
    thermostat::config_t thermostat_config;
    thermostat_config.feature_map = Thermostat::FeatureMap::kHeating; // Only Heating feature
    thermostat_config.control_sequence_of_operation = Thermostat::ControlSequenceOfOperationEnum::kHeatingOnly;
    thermostat_config.system_mode = system_mode;
    thermostat_config.occupied_heating_setpoint = heating_setpoint_matter;
    thermostat_config.local_temperature = current_local_temperature_matter;
    thermostat_config.min_heat_setpoint_limit = 500;  // 5.00 C
    thermostat_config.max_heat_setpoint_limit = 5000; // 50.00 C (Adjust if needed)
    thermostat_endpoint = thermostat::create(matter_node, &thermostat_config, ENDPOINT_FLAG_NONE, NULL);
    ESP_ABORT_IF_FALSE(thermostat_endpoint, "Failed to create Thermostat endpoint");
    uint16_t thermostat_ep_id = endpoint::get_id(thermostat_endpoint); // Get assigned ID
    ESP_LOGI(TAG, "Thermostat endpoint created with id: %d", thermostat_ep_id);


    // Set initial running state based on initial system_mode
    update_thermostat_running_state();

    /* Start Matter */
    err = esp_matter::start(NULL); ESP_ERROR_CHECK(err);

    /* Start PID Control Task */
    // Increased stack size slightly for safety with float operations
    xTaskCreate(pid_control_task, "pid_task", 5120, NULL, tskIDLE_PRIORITY + 3, NULL);
    ESP_LOGI(TAG, "PID control task started");

    ESP_LOGI(TAG, "app_main finished. Device is running.");
}
```
