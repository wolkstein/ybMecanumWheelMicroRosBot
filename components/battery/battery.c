/**
 * @file battery.c
 * @brief Implementation of battery pack monitoring, see battery.h.
 */
#include "battery.h"
#include <inttypes.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BATTERY";
#define NVS_NAMESPACE "battery"

/** Onboard battery-sense circuit (Yahboom microROS control board): GPIO3 ->
 *  ADC1 channel 2 on ESP32-S3, fixed 10k/3.3k divider (factor ~4.03). */
#define ADC_CHANNEL_BATTERY ADC_CHANNEL_2
#define ADC_ATTEN_BATTERY   ADC_ATTEN_DB_12

/** @name In-memory pack configuration.
 *  int32_t (not plain int) to match nvs_get_i32()'s out-param type exactly --
 *  on this toolchain int32_t is `long`, distinct from `int` for strict
 *  pointer-type checking even though both are 32-bit. Initialized from the
 *  Kconfig defaults, overwritten by battery_nvs_load() if NVS has saved
 *  values, and updated live by Battery_Save().
 *  @{
 */
static int32_t g_cell_count             = BATTERY_DEFAULT_CELL_COUNT;
static int32_t g_capacity_mah           = BATTERY_DEFAULT_CAPACITY_MAH;
static int32_t g_cell_voltage_max_mv    = BATTERY_DEFAULT_CELL_VOLTAGE_MAX_MV;
static int32_t g_cell_voltage_warn_mv   = BATTERY_DEFAULT_CELL_VOLTAGE_WARN_MV;
static int32_t g_cell_voltage_cutoff_mv = BATTERY_DEFAULT_CELL_VOLTAGE_CUTOFF_MV;
static int32_t g_technology             = BATTERY_DEFAULT_TECHNOLOGY;
static int32_t g_adc_divider_factor_x1000 = BATTERY_DEFAULT_ADC_DIVIDER_FACTOR_X1000;
/** @} */

static adc_oneshot_unit_handle_t battery_adc_handle;
static adc_cali_handle_t battery_cali_handle;

/** Latest measured pack voltage in Volts, written by battery_task() every
 *  ~100ms and read by Battery_Get_Voltage(). */
static volatile float g_battery_voltage = 0.0f;

/**
 * @brief Load the pack configuration from NVS, if any was previously saved.
 *
 * Initializes the NVS flash partition (erasing and retrying once if it's
 * missing or from an incompatible version). If the "battery" NVS namespace
 * doesn't exist yet (first boot / never saved), the Kconfig-derived defaults
 * already in the g_* globals are left untouched.
 */
static void battery_nvs_load(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGI(TAG, "No saved battery config, using defaults");
        return;
    }

    nvs_get_i32(handle, "cells",        &g_cell_count);
    nvs_get_i32(handle, "capacity_mah", &g_capacity_mah);
    nvs_get_i32(handle, "v_max_mv",     &g_cell_voltage_max_mv);
    nvs_get_i32(handle, "v_warn_mv",    &g_cell_voltage_warn_mv);
    nvs_get_i32(handle, "v_cut_mv",     &g_cell_voltage_cutoff_mv);
    nvs_get_i32(handle, "tech",         &g_technology);
    nvs_get_i32(handle, "div_x1000",    &g_adc_divider_factor_x1000);
    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded: %" PRId32 "S %" PRId32 "mAh max=%" PRId32 "mV/cell warn=%" PRId32 "mV/cell cutoff=%" PRId32 "mV/cell tech=%" PRId32 " div=%" PRId32,
             g_cell_count, g_capacity_mah, g_cell_voltage_max_mv, g_cell_voltage_warn_mv,
             g_cell_voltage_cutoff_mv, g_technology, g_adc_divider_factor_x1000);
}

/**
 * @brief Create the ADC hardware calibration scheme for the battery channel.
 *
 * Uses ESP-IDF's curve-fitting calibration so adc_cali_raw_to_voltage() in
 * battery_task() returns an accurate millivolt reading, not just a raw
 * ADC count.
 *
 * @param unit    ADC unit to calibrate (ADC_UNIT_1 for the battery channel).
 * @param channel ADC channel to calibrate (ADC_CHANNEL_BATTERY).
 * @param atten   Attenuation setting to calibrate for (ADC_ATTEN_BATTERY).
 * @return true if the calibration scheme was created successfully, false otherwise.
 */
static bool battery_adc_cali_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten)
{
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id  = unit,
        .chan     = channel,
        .atten    = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &battery_cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration ok");
        return true;
    }
    ESP_LOGE(TAG, "ADC calibration failed (0x%x)", ret);
    return false;
}

/**
 * @brief Configure the ADC oneshot unit/channel used for battery sensing.
 *
 * Sets up ADC1 channel 2 (GPIO3) with ADC_ATTEN_DB_12 attenuation, then
 * calls battery_adc_cali_init() to enable calibrated voltage readings.
 */
static void battery_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &battery_adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_BATTERY,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(battery_adc_handle, ADC_CHANNEL_BATTERY, &config));

    battery_adc_cali_init(ADC_UNIT_1, ADC_CHANNEL_BATTERY, ADC_ATTEN_BATTERY);
}

/**
 * @brief Background FreeRTOS task that samples the pack voltage.
 *
 * Every 100ms: reads the calibrated ADC voltage on the battery-sense
 * channel, applies the (possibly just-updated-via-Battery_Save())
 * voltage-divider factor, and stores the result in g_battery_voltage. The
 * divider factor is re-read from the global on every cycle (not cached at
 * task start) so that a `/battery_config` update takes effect immediately,
 * without needing a reboot.
 *
 * @param arg Unused (required by the FreeRTOS task function signature).
 */
static void battery_task(void *arg)
{
    int adc_raw, cali_voltage_mv;

    while (1) {
        if (adc_oneshot_read(battery_adc_handle, ADC_CHANNEL_BATTERY, &adc_raw) == ESP_OK &&
            adc_cali_raw_to_voltage(battery_cali_handle, adc_raw, &cali_voltage_mv) == ESP_OK) {
            float divider_factor = g_adc_divider_factor_x1000 / 1000.0f;
            g_battery_voltage = (cali_voltage_mv / 1000.0f) * divider_factor;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Battery_Init(void)
{
    battery_nvs_load();
    battery_adc_init();
    xTaskCreate(battery_task, "battery_task", 2560, NULL, 3, NULL);
}

float Battery_Get_Voltage(void)             { return g_battery_voltage; }
int   Battery_Get_CellCount(void)           { return g_cell_count; }
int   Battery_Get_CapacityMah(void)         { return g_capacity_mah; }
int   Battery_Get_CellVoltageMaxMV(void)    { return g_cell_voltage_max_mv; }
int   Battery_Get_CellVoltageWarnMV(void)   { return g_cell_voltage_warn_mv; }
int   Battery_Get_CellVoltageCutoffMV(void) { return g_cell_voltage_cutoff_mv; }
int   Battery_Get_Technology(void)          { return g_technology; }
int   Battery_Get_DividerFactorX1000(void)  { return g_adc_divider_factor_x1000; }

void Battery_Save(int cell_count, int capacity_mah, int cell_voltage_max_mv,
                   int cell_voltage_warn_mv, int cell_voltage_cutoff_mv,
                   int technology, int adc_divider_factor_x1000)
{
    g_cell_count               = cell_count;
    g_capacity_mah             = capacity_mah;
    g_cell_voltage_max_mv      = cell_voltage_max_mv;
    g_cell_voltage_warn_mv     = cell_voltage_warn_mv;
    g_cell_voltage_cutoff_mv   = cell_voltage_cutoff_mv;
    g_technology               = technology;
    g_adc_divider_factor_x1000 = adc_divider_factor_x1000;

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing");
        return;
    }

    nvs_set_i32(handle, "cells",        cell_count);
    nvs_set_i32(handle, "capacity_mah", capacity_mah);
    nvs_set_i32(handle, "v_max_mv",     cell_voltage_max_mv);
    nvs_set_i32(handle, "v_warn_mv",    cell_voltage_warn_mv);
    nvs_set_i32(handle, "v_cut_mv",     cell_voltage_cutoff_mv);
    nvs_set_i32(handle, "tech",         technology);
    nvs_set_i32(handle, "div_x1000",    adc_divider_factor_x1000);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Saved: %dS %dmAh max=%dmV/cell warn=%dmV/cell cutoff=%dmV/cell tech=%d div=%d",
             cell_count, capacity_mah, cell_voltage_max_mv, cell_voltage_warn_mv,
             cell_voltage_cutoff_mv, technology, adc_divider_factor_x1000);
}
