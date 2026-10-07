/**
 * @file isotp.h
 * @brief ISO 15765-2 (DoCAN) Transport / Network Layer Protocol Engine.
 *
 * Implements segmentation, reassembly, flow control negotiation, sequence
 * number validation, and ISO 15765-2 timing parameter supervision for
 * automotive Controller Area Network diagnostic stacks.
 *
 * Designed according to ISO 15765-2:2016 for MISRA-C:2012 compliance with
 * deterministic static memory allocation (zero dynamic heap).
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#ifndef ISOTP_H
#define ISOTP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "can_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum Diagnostic Message Buffer Size (bytes) */
#define ISOTP_BUF_SIZE              512U

/** @brief Standard ISO 15765-2 Flow Status Codes */
#define ISOTP_FS_CTS                0x00U /**< Continue To Send */
#define ISOTP_FS_WT                 0x01U /**< Wait */
#define ISOTP_FS_OVFLW              0x02U /**< Buffer Overflow */

/** @brief ISO 15765-2 Protocol Control Information (N_PCI) Frame Types */
#define ISOTP_FRAME_SF              0x00U /**< Single Frame */
#define ISOTP_FRAME_FF              0x01U /**< First Frame */
#define ISOTP_FRAME_CF              0x02U /**< Consecutive Frame */
#define ISOTP_FRAME_FC              0x03U /**< Flow Control Frame */

/** @brief Default ISO-TP Timing Parameters (milliseconds) */
#define ISOTP_DEFAULT_N_AS_MS       1000U /**< Max time to complete CAN frame transmission */
#define ISOTP_DEFAULT_N_AR_MS       1000U /**< Max time for receiver to transmit FC */
#define ISOTP_DEFAULT_N_BS_MS       1000U /**< Max time transmitter waits for FC after FF */
#define ISOTP_DEFAULT_N_CR_MS       1000U /**< Max time receiver waits for next CF */

/** @brief Default Flow Control Parameters */
#define ISOTP_DEFAULT_BLOCK_SIZE    8U    /**< Send 8 Consecutive Frames per Flow Control */
#define ISOTP_DEFAULT_ST_MIN_MS     10U   /**< Separation Time minimum (10 ms between CFs) */
#define ISOTP_PADDING_BYTE          0xAAU /**< Standard AUTOSAR/OEM padding byte */

/**
 * @brief ISO-TP Link Status / Result Codes
 */
typedef enum {
    ISOTP_RESULT_OK = 0,
    ISOTP_RESULT_BUFFER_OVERFLOW,
    ISOTP_RESULT_TIMEOUT_A,
    ISOTP_RESULT_TIMEOUT_BS,
    ISOTP_RESULT_TIMEOUT_CR,
    ISOTP_RESULT_WRONG_SEQUENCE,
    ISOTP_RESULT_INVALID_FRAME,
    ISOTP_RESULT_UNEXPECTED_PDU,
    ISOTP_RESULT_BUSY
} isotp_result_t;

/**
 * @brief ISO-TP Receiver State Machine States
 */
typedef enum {
    ISOTP_RX_STATE_IDLE = 0,
    ISOTP_RX_STATE_SEND_FC,
    ISOTP_RX_STATE_WAIT_CF,
    ISOTP_RX_STATE_COMPLETE
} isotp_rx_state_t;

/**
 * @brief ISO-TP Transmitter State Machine States
 */
typedef enum {
    ISOTP_TX_STATE_IDLE = 0,
    ISOTP_TX_STATE_SEND_FF,
    ISOTP_TX_STATE_WAIT_FC,
    ISOTP_TX_STATE_SEND_CF,
    ISOTP_TX_STATE_COMPLETE
} isotp_tx_state_t;

/**
 * @brief ISO-TP Link Configuration
 */
typedef struct {
    uint32_t rx_id;            /**< Functional or Physical CAN ID to accept */
    uint32_t tx_id;            /**< Diagnostic Response CAN ID to transmit */
    uint8_t  block_size;       /**< Maximum Consecutive Frames before requiring new FC */
    uint8_t  st_min_ms;        /**< Separation Time minimum between consecutive frames */
    bool     enable_padding;   /**< True to pad unused frame bytes with ISOTP_PADDING_BYTE */
    uint8_t  padding_val;      /**< Value for unused bytes (e.g., 0xAA or 0xCC) */
} isotp_config_t;

/**
 * @brief ISO-TP Channel Context
 */
typedef struct {
    isotp_config_t config;

    /* RX Engine Context */
    isotp_rx_state_t rx_state;
    uint8_t  rx_buffer[ISOTP_BUF_SIZE];
    uint16_t rx_total_len;
    uint16_t rx_index;
    uint8_t  rx_expected_sn;
    uint8_t  rx_block_counter;
    uint32_t rx_timer_ms;

    /* TX Engine Context */
    isotp_tx_state_t tx_state;
    uint8_t  tx_buffer[ISOTP_BUF_SIZE];
    uint16_t tx_total_len;
    uint16_t tx_index;
    uint8_t  tx_seq_num;
    uint8_t  tx_client_bs;     /**< Flow Control Block Size reported by tester */
    uint8_t  tx_client_stmin;  /**< Flow Control STmin reported by tester */
    uint8_t  tx_block_counter;
    uint32_t tx_timer_ms;
    uint32_t tx_stmin_timer_ms;

    /* Telemetry & Counters */
    uint32_t rx_frames_count;
    uint32_t tx_frames_count;
    uint32_t error_count;
} isotp_channel_t;

/**
 * @brief Initialize an ISO-TP communication channel.
 * @param channel Pointer to channel structure.
 * @param config Pointer to channel configuration.
 */
void isotp_init(isotp_channel_t *channel, const isotp_config_t *config);

/**
 * @brief Feed a received CAN frame into the ISO-TP network layer.
 * @param channel Pointer to channel structure.
 * @param frame Received CAN frame.
 * @return ISOTP_RESULT_OK or error code.
 */
isotp_result_t isotp_receive_can_frame(isotp_channel_t *channel, const can_frame_t *frame);

/**
 * @brief Query if a complete diagnostic message has been assembled by the receiver.
 * @param channel Pointer to channel structure.
 * @param data Output buffer pointer to store message.
 * @param len Output pointer to store received message length.
 * @param max_len Size of data output buffer.
 * @return true if complete message was copied, false if no message available.
 */
bool isotp_get_received_message(isotp_channel_t *channel, uint8_t *data, uint16_t *len, uint16_t max_len);

/**
 * @brief Enqueue a multi-byte diagnostic message for transmission over ISO-TP.
 * @param channel Pointer to channel structure.
 * @param data Payload data buffer to transmit.
 * @param len Length in bytes (1 to ISOTP_BUF_SIZE).
 * @return ISOTP_RESULT_OK if accepted, or error code if busy.
 */
isotp_result_t isotp_transmit(isotp_channel_t *channel, const uint8_t *data, uint16_t len);

/**
 * @brief Periodic time-slice process for ISO-TP state machines and timeouts.
 *
 * MUST be invoked at deterministic periodic intervals (e.g., 1 ms or 5 ms)
 * to service STmin inter-frame delays, N_Cr, N_Bs timeouts, and generate CF/FC frames.
 *
 * @param channel Pointer to channel structure.
 * @param delta_ms Milliseconds elapsed since last invocation.
 */
void isotp_process(isotp_channel_t *channel, uint32_t delta_ms);

/**
 * @brief Query if the transmitter is currently busy sending a message.
 * @param channel Pointer to channel structure.
 * @return true if transmission is in progress, false if idle.
 */
bool isotp_is_tx_busy(const isotp_channel_t *channel);

/**
 * @brief Reset ISO-TP channel states and abort ongoing transfers.
 * @param channel Pointer to channel structure.
 */
void isotp_reset(isotp_channel_t *channel);

#ifdef __cplusplus
}
#endif

#endif /* ISOTP_H */
