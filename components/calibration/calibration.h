#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Compile-time defaults from menuconfig (overridden by NVS at runtime)
#define CALIB_DEFAULT_WHEEL_DIAMETER_MM  ((float)CONFIG_ROBOT_WHEEL_DIAMETER_MM)
#define CALIB_DEFAULT_ROBOT_WIDTH_M      (CONFIG_ROBOT_WIDTH_MM  / 1000.0f)
#define CALIB_DEFAULT_ROBOT_LENGTH_M     (CONFIG_ROBOT_LENGTH_MM / 1000.0f)

// Load from NVS on boot, falls back to defaults if nothing stored
void Calibration_Init(void);

float Calib_Get_WheelDiameterMM(void);
float Calib_Get_RobotWidth(void);
float Calib_Get_RobotLength(void);

// Persist new values to NVS (call Motor_Set_WheelCirc / Motion_Set_Calibration afterwards)
void Calibration_Save(float wheel_diameter_mm, float robot_width_m, float robot_length_m);

#ifdef __cplusplus
}
#endif
