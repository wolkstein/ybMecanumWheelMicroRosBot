/**
 * @file calibration.h
 * @brief Robot-dimension calibration (wheel diameter, track width, wheelbase).
 *
 * Keeps the physical dimensions used for odometry and motion control, with
 * Kconfig-derived defaults that can be overridden at runtime via the
 * `/calibrate` micro-ROS topic (see main.c) and persisted in NVS.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** @name Compile-time defaults from menuconfig ("Robot physical dimensions").
 *  Used until overridden by NVS or by a `/calibrate` message.
 *  @{
 */
#define CALIB_DEFAULT_WHEEL_DIAMETER_MM  ((float)CONFIG_ROBOT_WHEEL_DIAMETER_MM)
#define CALIB_DEFAULT_ROBOT_WIDTH_M      (CONFIG_ROBOT_WIDTH_MM  / 1000.0f)
#define CALIB_DEFAULT_ROBOT_LENGTH_M     (CONFIG_ROBOT_LENGTH_MM / 1000.0f)
/** @} */

/**
 * @brief Load calibration values from NVS, if any were previously saved.
 *
 * Falls back to the Kconfig defaults above if the "calib" NVS namespace
 * doesn't exist yet (first boot / never saved). Must be called once during
 * startup, before Calib_Get_*() is relied upon.
 */
void Calibration_Init(void);

/** @brief Current wheel diameter in millimeters. */
float Calib_Get_WheelDiameterMM(void);

/** @brief Current track width (distance between left/right wheel contact points) in meters. */
float Calib_Get_RobotWidth(void);

/** @brief Current wheelbase (distance between front/rear wheel contact points) in meters. */
float Calib_Get_RobotLength(void);

/**
 * @brief Persist new calibration values to NVS and apply them immediately.
 *
 * Note: this only updates the values returned by Calib_Get_*() and NVS —
 * the caller is still responsible for pushing the new wheel circumference
 * into the motor/odometry code afterwards (Motor_Set_WheelCirc() /
 * Motion_Set_Calibration(), see calibrate_callback() in main.c).
 *
 * @param wheel_diameter_mm Wheel diameter in millimeters.
 * @param robot_width_m     Track width (left/right) in meters.
 * @param robot_length_m    Wheelbase (front/rear) in meters.
 */
void Calibration_Save(float wheel_diameter_mm, float robot_width_m, float robot_length_m);

#ifdef __cplusplus
}
#endif
