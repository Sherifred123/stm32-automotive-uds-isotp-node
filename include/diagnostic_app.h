/**
 * @file diagnostic_app.h
 * @brief Application Orchestrator for STM32 Automotive Diagnostic ECU Node.
 *
 * Coordinates bxCAN peripheral, ISO-TP network layer, and UDS diagnostic
 * server. Bridges hardware I/O (User LED LD2 session indicators, User Button B1,
 * USART2 diagnostic console trace).
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#ifndef DIAGNOSTIC_APP_H
#define DIAGNOSTIC_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "can_driver.h"
#include "isotp.h"
#include "uds_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Standard ISO 15765-4 Diagnostic CAN Identifiers */
#define DIAG_CAN_ID_PHYS_REQ        0x7E0U /**< Physical Request to ECU */
#define DIAG_CAN_ID_FUNC_REQ        0x7DFU /**< Functional Broadcast Request */
#define DIAG_CAN_ID_PHYS_RESP       0x7E8U /**< Physical Response from ECU */

/**
 * @brief Diagnostic Application Controller Context
 */
typedef struct {
    can_config_t     can_config;
    isotp_channel_t  isotp_channel;
    uds_server_t     uds_server;

    /* Application status metrics */
    uint32_t uptime_ms;
    uint32_t led_blink_timer_ms;
    bool     led_state;
    bool     silicon_loopback_enabled;
} diagnostic_app_t;

/**
 * @brief Initialize all diagnostic subsystems (bxCAN, ISO-TP, UDS).
 * @param app Pointer to application context.
 * @param loopback_mode Set true for standalone self-test without external transceivers.
 */
void diagnostic_app_init(diagnostic_app_t *app, bool loopback_mode);

/**
 * @brief Execute one non-blocking tick of the diagnostic stack.
 *
 * Should be called inside the main while(1) loop or from a periodic 1ms timer.
 * Handles CAN frame ingestion, ISO-TP segmentation/reassembly, UDS request processing,
 * and LED animation.
 *
 * @param app Pointer to application context.
 * @param delta_ms Milliseconds elapsed since last tick.
 */
void diagnostic_app_process(diagnostic_app_t *app, uint32_t delta_ms);

/**
 * @brief Formatted UART trace logger for diagnostic frames and events.
 * @param fmt Printf-style format string.
 */
void diag_log(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* DIAGNOSTIC_APP_H */
