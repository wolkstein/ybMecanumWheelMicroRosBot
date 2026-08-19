/**
 * @file battery.h
 * @brief Battery pack monitoring for the ybMecanumWheelMicroRosBot robot board.
 *
 * Reads the pack voltage via the onboard ADC (GPIO3 / ADC1 channel 2, per
 * Yahboom's battery-sense circuit) and keeps a small set of pack parameters
 * (cell count, capacity, voltage thresholds, chemistry, ADC divider factor)
 * that are persisted in NVS and can be updated at runtime via the
 * `/battery_config` micro-ROS topic (see main.c).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** @name Compile-time defaults from menuconfig (Kconfig "Battery configuration").
 *  Used as the initial value of each parameter until overridden by NVS or
 *  by a `/battery_config` message (see Battery_Save()).
 *  @{
 */
#define BATTERY_DEFAULT_CELL_COUNT             CONFIG_BATTERY_CELL_COUNT
#define BATTERY_DEFAULT_CAPACITY_MAH           CONFIG_BATTERY_CAPACITY_MAH
#define BATTERY_DEFAULT_CELL_VOLTAGE_MAX_MV    CONFIG_BATTERY_CELL_VOLTAGE_MAX_MV
#define BATTERY_DEFAULT_CELL_VOLTAGE_WARN_MV   CONFIG_BATTERY_CELL_VOLTAGE_WARN_MV
#define BATTERY_DEFAULT_CELL_VOLTAGE_CUTOFF_MV CONFIG_BATTERY_CELL_VOLTAGE_CUTOFF_MV
#define BATTERY_DEFAULT_TECHNOLOGY             CONFIG_BATTERY_TECHNOLOGY
#define BATTERY_DEFAULT_ADC_DIVIDER_FACTOR_X1000 CONFIG_BATTERY_ADC_DIVIDER_FACTOR_X1000
/** @} */

/**
 * @brief Initialize battery monitoring.
 *
 * Loads the pack configuration from NVS (falling back to the Kconfig
 * defaults above if nothing has been saved yet), initializes the battery-
 * sense ADC channel with hardware calibration, and starts a background
 * FreeRTOS task that samples the pack voltage every 100ms.
 *
 * Must be called once during startup, before Battery_Get_Voltage() or any
 * of the config getters are relied upon.
 */
void Battery_Init(void);

/**
 * @brief Get the most recently measured pack voltage.
 * @return Pack voltage in Volts, updated every ~100ms by the background
 *         sampling task started in Battery_Init().
 */
float Battery_Get_Voltage(void);

/** @name NVS-backed pack configuration getters.
 *  Reflect the values currently in effect (Kconfig default, or whatever
 *  was last loaded from NVS / set via Battery_Save()). See Kconfig
 *  "Battery configuration" for the meaning and units of each value.
 *  @{
 */

/** @brief Number of cells in series (e.g. 2 for a 2S pack). */
int Battery_Get_CellCount(void);

/** @brief Rated pack capacity in mAh. */
int Battery_Get_CapacityMah(void);

/** @brief Fully-charged voltage per cell, in millivolts. */
int Battery_Get_CellVoltageMaxMV(void);

/** @brief Per-cell threshold (millivolts) below which the low-battery
 *         warning beep is triggered (see main.c timer_battery_callback). */
int Battery_Get_CellVoltageWarnMV(void);

/** @brief Per-cell threshold (millivolts) below which the pack is reported
 *         as sensor_msgs/BatteryState::POWER_SUPPLY_HEALTH_DEAD. */
int Battery_Get_CellVoltageCutoffMV(void);

/** @brief Cell chemistry as a sensor_msgs/BatteryState POWER_SUPPLY_TECHNOLOGY_*
 *         constant (e.g. 2 = Li-ion, 3 = LiPo). */
int Battery_Get_Technology(void);

/**
 * @brief Get the ADC voltage-divider calibration factor.
 * @return Divider factor multiplied by 1000, e.g. 4030 means factor 4.03.
 *         Pack voltage = ADC-measured GPIO voltage * this / 1000. Measure
 *         with a multimeter (pack voltage vs. GPIO3 voltage) and adjust if
 *         a given board's onboard divider deviates from Yahboom's stock
 *         10k/3.3k value.
 */
int Battery_Get_DividerFactorX1000(void);
/** @} */

/**
 * @brief Persist a new pack configuration to NVS and apply it immediately.
 *
 * Updates the in-memory configuration (picked up by the next voltage sample
 * and the next `/battery_state` publish) and writes all values to the NVS
 * namespace "battery", so they survive a reboot/reset. Called from
 * battery_config_callback() in main.c on every `/battery_config` message.
 *
 * @param cell_count               Number of cells in series.
 * @param capacity_mah             Rated pack capacity in mAh.
 * @param cell_voltage_max_mv      Fully-charged voltage per cell, in mV.
 * @param cell_voltage_warn_mv     Per-cell low-battery warning threshold, in mV.
 * @param cell_voltage_cutoff_mv   Per-cell shutdown/DEAD threshold, in mV.
 * @param technology               sensor_msgs/BatteryState POWER_SUPPLY_TECHNOLOGY_* constant.
 * @param adc_divider_factor_x1000 ADC voltage-divider factor, multiplied by 1000.
 */
void Battery_Save(int cell_count, int capacity_mah, int cell_voltage_max_mv,
                   int cell_voltage_warn_mv, int cell_voltage_cutoff_mv,
                   int technology, int adc_divider_factor_x1000);

#ifdef __cplusplus
}
#endif
