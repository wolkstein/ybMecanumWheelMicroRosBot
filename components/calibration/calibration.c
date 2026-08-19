/**
 * @file calibration.c
 * @brief Implementation of robot-dimension calibration, see calibration.h.
 */
#include "calibration.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "CALIB";
#define NVS_NAMESPACE "calib"

/** In-memory calibration, initialized from Kconfig defaults, overwritten by
 *  Calibration_Init() if NVS has saved values, updated live by Calibration_Save(). */
static float g_wheel_diameter_mm = CALIB_DEFAULT_WHEEL_DIAMETER_MM;
static float g_robot_width_m     = CALIB_DEFAULT_ROBOT_WIDTH_M;
static float g_robot_length_m    = CALIB_DEFAULT_ROBOT_LENGTH_M;

/**
 * @brief Read a float value from NVS.
 *
 * NVS has no native float type, so values are stored as their raw 4-byte
 * bit pattern in a u32 slot and reinterpreted here. Leaves *out untouched
 * if the key doesn't exist (e.g. first boot).
 *
 * @param h   Open NVS handle.
 * @param key NVS key to read.
 * @param out Destination for the decoded float.
 */
static void nvs_get_float(nvs_handle_t h, const char *key, float *out)
{
    uint32_t raw;
    if (nvs_get_u32(h, key, &raw) == ESP_OK)
        memcpy(out, &raw, sizeof(float));
}

/**
 * @brief Write a float value to NVS as its raw bit pattern.
 * @param h   Open NVS handle (must have been opened with write access).
 * @param key NVS key to write.
 * @param val Float value to store.
 */
static void nvs_set_float(nvs_handle_t h, const char *key, float val)
{
    uint32_t raw;
    memcpy(&raw, &val, sizeof(float));
    nvs_set_u32(h, key, raw);
}

void Calibration_Init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Partition layout changed or is uninitialized -- wipe and retry once.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        // Namespace doesn't exist yet -- nothing has been saved, keep Kconfig defaults.
        ESP_LOGI(TAG, "No saved calibration, using defaults");
        return;
    }

    nvs_get_float(handle, "wheel_diam",  &g_wheel_diameter_mm);
    nvs_get_float(handle, "robot_width", &g_robot_width_m);
    nvs_get_float(handle, "robot_len",   &g_robot_length_m);
    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded: wheel=%.1fmm width=%.3fm len=%.3fm",
             g_wheel_diameter_mm, g_robot_width_m, g_robot_length_m);
}

float Calib_Get_WheelDiameterMM(void) { return g_wheel_diameter_mm; }
float Calib_Get_RobotWidth(void)       { return g_robot_width_m; }
float Calib_Get_RobotLength(void)      { return g_robot_length_m; }

void Calibration_Save(float wheel_diameter_mm, float robot_width_m, float robot_length_m)
{
    g_wheel_diameter_mm = wheel_diameter_mm;
    g_robot_width_m     = robot_width_m;
    g_robot_length_m    = robot_length_m;

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing");
        return;
    }

    nvs_set_float(handle, "wheel_diam",  wheel_diameter_mm);
    nvs_set_float(handle, "robot_width", robot_width_m);
    nvs_set_float(handle, "robot_len",   robot_length_m);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Saved: wheel=%.1fmm width=%.3fm len=%.3fm",
             wheel_diameter_mm, robot_width_m, robot_length_m);
}
