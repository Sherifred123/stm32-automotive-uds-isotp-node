/**
 * @file diagnostic_app.c
 * @brief Diagnostic Application Orchestrator and Platform Integration.
 *
 * Coordinates CAN hardware reception, ISO-TP transport framing, UDS server execution,
 * and board status indication for STM32 NUCLEO-F446RE.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include "diagnostic_app.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#if defined(STM32F446xx) || defined(TARGET_STM32F4)
#define STM32_GPIOA_ODR (*(volatile uint32_t *)(0x40020000UL + 0x14))
static void hw_set_led(bool state)
{
    if (state) {
        STM32_GPIOA_ODR |= (1UL << 5); /* User LED LD2 on PA5 */
    } else {
        STM32_GPIOA_ODR &= ~(1UL << 5);
    }
}
#else
static void hw_set_led(bool state)
{
    (void)state;
}
#endif

void diag_log(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
}

void diagnostic_app_init(diagnostic_app_t *app, bool loopback_mode)
{
    if (app == NULL) {
        return;
    }

    memset(app, 0, sizeof(*app));
    app->silicon_loopback_enabled = loopback_mode;

    /* 1. Configure bxCAN 500 kbps (45 MHz APB1 clock: BRP=5, TS1=15, TS2=2, SJW=1) */
    app->can_config.mode = loopback_mode ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
    app->can_config.baudrate_kbps = 500U;
    app->can_config.brp = 5U;
    app->can_config.ts1 = 15U;
    app->can_config.ts2 = 2U;
    app->can_config.sjw = 1U;
    app->can_config.auto_retransmit = true;
    app->can_config.auto_bus_off = true;

    can_driver_init(&app->can_config);

    /* 2. Configure Hardware Filter Bank 0: Physical Request (0x7E0) */
    can_filter_config_t filt_phys;
    filt_phys.filter_bank = 0U;
    filt_phys.filter_id = DIAG_CAN_ID_PHYS_REQ;
    filt_phys.filter_mask = CAN_STD_ID_MASK;
    filt_phys.fifo_assignment = 0U;
    filt_phys.is_extended = false;
    filt_phys.enable = true;
    can_driver_configure_filter(&filt_phys);

    /* 3. Configure Hardware Filter Bank 1: Functional Broadcast (0x7DF) */
    can_filter_config_t filt_func;
    filt_func.filter_bank = 1U;
    filt_func.filter_id = DIAG_CAN_ID_FUNC_REQ;
    filt_func.filter_mask = CAN_STD_ID_MASK;
    filt_func.fifo_assignment = 0U;
    filt_func.is_extended = false;
    filt_func.enable = true;
    can_driver_configure_filter(&filt_func);

    /* 4. Initialize ISO-TP Channel */
    isotp_config_t isotp_cfg;
    isotp_cfg.rx_id = DIAG_CAN_ID_PHYS_REQ;
    isotp_cfg.tx_id = DIAG_CAN_ID_PHYS_RESP;
    isotp_cfg.block_size = ISOTP_DEFAULT_BLOCK_SIZE;
    isotp_cfg.st_min_ms = ISOTP_DEFAULT_ST_MIN_MS;
    isotp_cfg.enable_padding = true;
    isotp_cfg.padding_val = ISOTP_PADDING_BYTE;

    isotp_init(&app->isotp_channel, &isotp_cfg);

    /* 5. Initialize UDS Diagnostic Server */
    uds_server_init(&app->uds_server);

    diag_log("[DIAG] Initialized STM32 Diagnostic Node (CAN: 500k, Req: 0x%03X, Resp: 0x%03X, Mode: %s)\n",
             DIAG_CAN_ID_PHYS_REQ, DIAG_CAN_ID_PHYS_RESP,
             loopback_mode ? "LOOPBACK" : "NORMAL");
}

void diagnostic_app_process(diagnostic_app_t *app, uint32_t delta_ms)
{
    if (app == NULL) {
        return;
    }

    app->uptime_ms += delta_ms;

    /* 1. Pull frames from CAN hardware queue into ISO-TP network layer */
    can_frame_t rx_frame;
    while (can_driver_receive(&rx_frame)) {
        isotp_receive_can_frame(&app->isotp_channel, &rx_frame);
    }

    /* 2. Step ISO-TP network layer timeouts and consecutive frames */
    isotp_process(&app->isotp_channel, delta_ms);

    /* 3. Check if a complete diagnostic request message was reassembled */
    uint8_t req_buf[ISOTP_BUF_SIZE];
    uint16_t req_len = 0U;

    if (isotp_get_received_message(&app->isotp_channel, req_buf, &req_len, sizeof(req_buf))) {
        uint8_t rsp_buf[ISOTP_BUF_SIZE];
        uint16_t rsp_len = 0U;

        diag_log("[UDS REQ] SID=0x%02X Len=%u", req_buf[0], req_len);
        for (uint16_t i = 1U; i < req_len && i < 16U; i++) {
            diag_log(" %02X", req_buf[i]);
        }
        if (req_len > 16U) {
            diag_log(" ...");
        }
        diag_log("\n");

        bool send_rsp = uds_server_process_request(&app->uds_server,
                                                  req_buf, req_len,
                                                  rsp_buf, &rsp_len,
                                                  sizeof(rsp_buf));

        if (send_rsp) {
            diag_log("[UDS RSP] SID=0x%02X Len=%u", rsp_buf[0], rsp_len);
            for (uint16_t i = 1U; i < rsp_len && i < 16U; i++) {
                diag_log(" %02X", rsp_buf[i]);
            }
            if (rsp_len > 16U) {
                diag_log(" ...");
            }
            diag_log("\n");

            isotp_transmit(&app->isotp_channel, rsp_buf, rsp_len);
        }
    }

    /* 4. Step UDS Server timers (S3 timeout, Security Lockout) */
    uds_server_process(&app->uds_server, delta_ms);

    /* 5. Update Diagnostic Status LED (LD2 PA5) */
    app->led_blink_timer_ms += delta_ms;
    uint8_t session = uds_server_get_session(&app->uds_server);

    if (session == UDS_SESSION_DEFAULT) {
        /* Default Session: Slow 1 Hz Heartbeat */
        if (app->led_blink_timer_ms >= 500U) {
            app->led_blink_timer_ms = 0U;
            app->led_state = !app->led_state;
            hw_set_led(app->led_state);
        }
    } else if (session == UDS_SESSION_PROGRAMMING) {
        /* Programming Session: Fast 5 Hz Strobe */
        if (app->led_blink_timer_ms >= 100U) {
            app->led_blink_timer_ms = 0U;
            app->led_state = !app->led_state;
            hw_set_led(app->led_state);
        }
    } else if (session == UDS_SESSION_EXTENDED) {
        /* Extended Diagnostic Session: Solid High */
        app->led_state = true;
        hw_set_led(true);
    }
}
