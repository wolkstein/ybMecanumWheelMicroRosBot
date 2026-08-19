#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Compile-time defaults from menuconfig (overridden by NVS at runtime)
#define BATTERY_DEFAULT_CELL_COUNT             CONFIG_BATTERY_CELL_COUNT
#define BATTERY_DEFAULT_CAPACITY_MAH           CONFIG_BATTERY_CAPACITY_MAH
#define BATTERY_DEFAULT_CELL_VOLTAGE_MAX_MV    CONFIG_BATTERY_CELL_VOLTAGE_MAX_MV
#define BATTERY_DEFAULT_CELL_VOLTAGE_WARN_MV   CONFIG_BATTERY_CELL_VOLTAGE_WARN_MV
#define BATTERY_DEFAULT_CELL_VOLTAGE_CUTOFF_MV CONFIG_BATTERY_CELL_VOLTAGE_CUTOFF_MV
#define BATTERY_DEFAULT_TECHNOLOGY             CONFIG_BATTERY_TECHNOLOGY
#define BATTERY_DEFAULT_ADC_DIVIDER_FACTOR_X1000 CONFIG_BATTERY_ADC_DIVIDER_FACTOR_X1000

// Load NVS-backed pack config (falls back to Kconfig defaults), init the
// onboard battery-sense ADC (GPIO3 / ADC1 channel 2) and start a background
// task that samples the pack voltage every 100ms.
void Battery_Init(void);

// Latest measured pack voltage in Volts.
float Battery_Get_Voltage(void);

// NVS-backed pack configuration, see Kconfig "Battery configuration" for
// the meaning of each value.
int Battery_Get_CellCount(void);
int Battery_Get_CapacityMah(void);
int Battery_Get_CellVoltageMaxMV(void);
int Battery_Get_CellVoltageWarnMV(void);
int Battery_Get_CellVoltageCutoffMV(void);
int Battery_Get_Technology(void);
// Divider factor x1000, e.g. 4030 = factor 4.03. Measure with a multimeter
// (pack voltage / GPIO3 voltage) and adjust if your board's onboard divider
// deviates from Yahboom's stock 10k/3.3k value.
int Battery_Get_DividerFactorX1000(void);

// Persist new pack configuration to NVS.
void Battery_Save(int cell_count, int capacity_mah, int cell_voltage_max_mv,
                   int cell_voltage_warn_mv, int cell_voltage_cutoff_mv,
                   int technology, int adc_divider_factor_x1000);

#ifdef __cplusplus
}
#endif
