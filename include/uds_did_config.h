/**
 * @file uds_did_config.h
 * @brief Data Identifier (DID) and Routine Control Registry for UDS Server.
 *
 * Defines standardized automotive identifiers according to ISO 14229-1:
 * - Standard Vehicle Identification Number (VIN)
 * - ECU Identification (Part Number, Software Version)
 * - Live Battery Pack Telemetry DIDs
 * - Hardware Diagnostic & Diagnostic Trouble Codes (DTC)
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#ifndef UDS_DID_CONFIG_H
#define UDS_DID_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Standard Automotive DIDs (ISO 14229-1 Annex C & SAE J1979)
 * ========================================================================= */
#define UDS_DID_VIN                     0xF190U /**< Vehicle Identification Number (17 bytes) */
#define UDS_DID_ECU_PART_NUMBER         0xF187U /**< ECU Spare Part Number (10 bytes) */
#define UDS_DID_ECU_SW_VERSION          0xF189U /**< ECU Software Version (4 bytes: M.m.p.b) */
#define UDS_DID_BOOT_SW_IDENTIFIER      0xF180U /**< Bootloader Version Identifier */

/* Custom Application DIDs */
#define UDS_DID_BMS_TELEMETRY           0x0100U /**< Pack Voltage, Current, SOC, Temp */
#define UDS_DID_MCU_HARDWARE_HEALTH     0x0200U /**< Core Temp, VREF, SysTick, Bus Errors */
#define UDS_DID_SPEED_GOVERNOR_LIMIT    0x0105U /**< Maximum Vehicle Speed Limit (Writable) */

/* Routine Identifiers (RID) */
#define UDS_RID_FLASH_INTEGRITY_CHECK   0x0201U /**< Flash CRC Check Routine */
#define UDS_RID_LED_ACTUATOR_TEST       0x0202U /**< User LED Flash Actuator Test */

/* Diagnostic Trouble Codes (DTC - 3 Bytes) */
#define UDS_DTC_BATT_OVERTEMP           0xE00100U /**< Battery Cell High Temp Alarm */
#define UDS_DTC_CAN_COMM_FAULT          0xC00100U /**< CAN Bus Lost Communication */
#define UDS_DTC_STATUS_ACTIVE           0x09U     /**< TestFailed | ConfirmedDTC */

/* =========================================================================
 * Live Telemetry Data Structures
 * ========================================================================= */

/**
 * @brief Live Battery Pack Diagnostic Telemetry
 */
typedef struct {
    uint16_t pack_voltage_deci_v;  /**< Pack Voltage in 0.1V units (e.g., 512 = 51.2V) */
    int16_t  pack_current_deci_a;  /**< Pack Current in 0.1A units (signed: charge/discharge) */
    uint8_t  state_of_charge_pct;  /**< SOC percentage (0 - 100%) */
    int8_t   max_cell_temp_c;      /**< Maximum Cell Temperature in °C */
} uds_bms_telemetry_t;

/**
 * @brief MCU Hardware Diagnostics
 */
typedef struct {
    int16_t  mcu_temperature_c;    /**< Internal Silicon Temp Sensor */
    uint16_t vrefint_mv;           /**< Internal Reference Voltage (mV) */
    uint32_t uptime_seconds;       /**< SysTick Uptime */
    uint16_t can_tec;              /**< Transmit Error Counter */
    uint16_t can_rec;              /**< Receive Error Counter */
} uds_mcu_health_t;

/**
 * @brief Global Application Calibration & State
 */
typedef struct {
    uint8_t speed_governor_kmh;    /**< Configured speed governor limit (km/h) */
    bool    routine_flash_running;
    bool    routine_led_running;
    uint8_t routine_led_blink_count;
} uds_app_state_t;

#ifdef __cplusplus
}
#endif

#endif /* UDS_DID_CONFIG_H */
