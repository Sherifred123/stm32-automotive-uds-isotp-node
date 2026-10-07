/**
 * @file can_driver.h
 * @brief Hardware Abstraction Layer & Driver Interface for STM32 bxCAN.
 *
 * Provides register-level abstraction, bit timing configuration (ISO 11898-1),
 * hardware acceptance filter bank configuration, and non-blocking frame TX/RX
 * for STM32F446RE (ARM Cortex-M4) and host-native simulation harnesses.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Standard CAN 2.0B Frame Maximum Payload Length */
#define CAN_MAX_DLC                 8U

/** @brief Standard 11-bit Identifier Mask */
#define CAN_STD_ID_MASK             0x000007FFU

/** @brief Extended 29-bit Identifier Mask */
#define CAN_EXT_ID_MASK             0x1FFFFFFFU

/**
 * @brief CAN Operating Modes
 */
typedef enum {
    CAN_MODE_NORMAL = 0,    /**< Normal on-bus transceiver communication */
    CAN_MODE_LOOPBACK,      /**< Internal silicon loopback (self-test mode) */
    CAN_MODE_SILENT,        /**< Listen-only mode without ACK generation */
    CAN_MODE_SILENT_LOOPBACK /**< Internal loopback with RX detached from bus */
} can_mode_t;

/**
 * @brief CAN Frame Structure
 */
typedef struct {
    uint32_t id;            /**< Standard (11-bit) or Extended (29-bit) CAN ID */
    bool is_extended;       /**< True if 29-bit identifier, false for 11-bit standard */
    bool is_rtr;            /**< True if Remote Transmission Request */
    uint8_t dlc;            /**< Data Length Code (0 to 8 bytes) */
    uint8_t data[CAN_MAX_DLC]; /**< Payload data buffer */
    uint32_t timestamp_ms;  /**< Millisecond arrival/dispatch timestamp */
} can_frame_t;

/**
 * @brief CAN Bus Error & Health Statistics
 */
typedef struct {
    uint32_t tx_success_count;
    uint32_t rx_success_count;
    uint32_t tx_overflow_count;
    uint32_t rx_overflow_count;
    uint32_t bus_off_count;
    uint32_t error_warning_count;
    uint8_t  tec;            /**< Transmit Error Counter */
    uint8_t  rec;            /**< Receive Error Counter */
} can_stats_t;

/**
 * @brief CAN Driver Configuration Parameters
 */
typedef struct {
    can_mode_t mode;         /**< Operational mode (Normal, Loopback) */
    uint32_t baudrate_kbps;  /**< Nominal baudrate (typically 500 kbps) */
    uint16_t brp;            /**< Baud Rate Prescaler */
    uint8_t  ts1;            /**< Time Segment 1 (Time quanta) */
    uint8_t  ts2;            /**< Time Segment 2 (Time quanta) */
    uint8_t  sjw;            /**< Resynchronization Jump Width */
    bool auto_retransmit;    /**< True: Enable automatic retransmission */
    bool auto_bus_off;       /**< True: Automatic bus-off recovery */
} can_config_t;

/**
 * @brief Hardware Filter Bank Configuration (32-bit Identifier Mask Mode)
 */
typedef struct {
    uint8_t  filter_bank;    /**< Filter bank index (0 to 13 on STM32 bxCAN1) */
    uint32_t filter_id;      /**< Acceptance Identifier */
    uint32_t filter_mask;    /**< Acceptance Mask (1 = must match, 0 = don't care) */
    uint8_t  fifo_assignment;/**< 0: FIFO 0, 1: FIFO 1 */
    bool     is_extended;    /**< True for 29-bit filter, false for 11-bit standard */
    bool     enable;         /**< True to activate filter bank */
} can_filter_config_t;

/**
 * @brief Initialize the CAN peripheral and configure bit timing.
 * @param config Pointer to configuration structure.
 * @return true if initialized successfully, false on error.
 */
bool can_driver_init(const can_config_t *config);

/**
 * @brief Configure a hardware acceptance filter bank.
 * @param filter_cfg Pointer to filter configuration parameters.
 * @return true on success, false if parameter out of range.
 */
bool can_driver_configure_filter(const can_filter_config_t *filter_cfg);

/**
 * @brief Transmit a CAN frame (Non-blocking).
 * @param frame Pointer to frame to transmit.
 * @return true if enqueued into TX mailbox, false if mailboxes full.
 */
bool can_driver_transmit(const can_frame_t *frame);

/**
 * @brief Check if a received frame is available in the RX queue.
 * @param frame Output buffer for received frame.
 * @return true if a frame was read, false if queue is empty.
 */
bool can_driver_receive(can_frame_t *frame);

/**
 * @brief Retrieve current CAN bus statistics and error counters.
 * @param stats Output pointer to statistics structure.
 */
void can_driver_get_stats(can_stats_t *stats);

/**
 * @brief Reset internal CAN bus statistics.
 */
void can_driver_reset_stats(void);

/**
 * @brief Periodic supervisory tick for CAN driver (bus-off recovery, etc.).
 * @param delta_ms Milliseconds elapsed since last tick.
 */
void can_driver_process(uint32_t delta_ms);

#ifdef __cplusplus
}
#endif

#endif /* CAN_DRIVER_H */
