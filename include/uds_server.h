/**
 * @file uds_server.h
 * @brief ISO 14229-1 Unified Diagnostic Services (UDS) Server Engine.
 *
 * Implements an automotive diagnostic ECU server compliant with ISO 14229-1:
 * - Diagnostic Session Management (Default, Programming, Extended)
 * - Security Access Engine with pseudo-random seed generation and brute-force lockout
 * - Negative Response Code (NRC) generator
 * - Multi-frame Data Identifier (DID) read & write handlers
 * - Non-volatile Routine Control execution
 * - S3 Server Session Timeout supervision
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#ifndef UDS_SERVER_H
#define UDS_SERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "uds_did_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * ISO 14229-1 Service Identifiers (SID)
 * ========================================================================= */
#define UDS_SID_DIAGNOSTIC_SESSION_CONTROL  0x10U
#define UDS_SID_ECU_RESET                   0x11U
#define UDS_SID_CLEAR_DIAGNOSTIC_INFO       0x14U
#define UDS_SID_READ_DTC_INFO               0x19U
#define UDS_SID_READ_DATA_BY_ID             0x22U
#define UDS_SID_SECURITY_ACCESS             0x27U
#define UDS_SID_WRITE_DATA_BY_ID            0x2EU
#define UDS_SID_ROUTINE_CONTROL             0x31U
#define UDS_SID_TESTER_PRESENT              0x3EU

#define UDS_SID_NEGATIVE_RESPONSE           0x7FU
#define UDS_RSP_OFFSET                      0x40U

/* =========================================================================
 * Negative Response Codes (NRC) - ISO 14229-1
 * ========================================================================= */
#define UDS_NRC_GENERAL_REJECT                  0x10U
#define UDS_NRC_SERVICE_NOT_SUPPORTED           0x11U
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED       0x12U
#define UDS_NRC_INCORRECT_MESSAGE_LENGTH        0x13U
#define UDS_NRC_RESPONSE_TOO_LONG               0x14U
#define UDS_NRC_BUSY_REPEAT_REQUEST             0x21U
#define UDS_NRC_CONDITIONS_NOT_CORRECT          0x22U
#define UDS_NRC_REQUEST_SEQUENCE_ERROR          0x24U
#define UDS_NRC_REQUEST_OUT_OF_RANGE            0x31U
#define UDS_NRC_SECURITY_ACCESS_DENIED          0x33U
#define UDS_NRC_INVALID_KEY                     0x35U
#define UDS_NRC_EXCEEDED_NUMBER_OF_ATTEMPTS     0x36U
#define UDS_NRC_REQUIRED_TIME_DELAY_NOT_EXPIRED 0x37U
#define UDS_NRC_RESPONSE_PENDING                0x78U

/* =========================================================================
 * Diagnostic Session Types
 * ========================================================================= */
#define UDS_SESSION_DEFAULT                     0x01U
#define UDS_SESSION_PROGRAMMING                 0x02U
#define UDS_SESSION_EXTENDED                    0x03U

/* S3 Server Timeout (5000 ms per ISO 14229-2) */
#define UDS_S3_SERVER_TIMEOUT_MS                5000U
#define UDS_SECURITY_MAX_ATTEMPTS               3U
#define UDS_SECURITY_LOCKOUT_DELAY_MS           10000U /**< 10-second penalty delay */

/**
 * @brief UDS Security Access State
 */
typedef enum {
    UDS_SECURITY_LOCKED = 0,
    UDS_SECURITY_SEED_ISSUED,
    UDS_SECURITY_UNLOCKED
} uds_security_state_t;

/**
 * @brief UDS Server Statistics & Telemetry
 */
typedef struct {
    uint32_t requests_received;
    uint32_t positive_responses;
    uint32_t negative_responses;
    uint32_t session_timeouts;
    uint32_t security_unlock_count;
    uint32_t security_fail_count;
} uds_stats_t;

/**
 * @brief UDS Server State Machine Context
 */
typedef struct {
    uint8_t  active_session;           /**< Current diagnostic session */
    uint32_t s3_timer_ms;              /**< Non-default session timeout monitor */
    
    uds_security_state_t security_state;
    uint32_t active_seed;              /**< Currently issued cryptographic seed */
    uint8_t  failed_security_attempts; /**< Counter for failed key responses */
    uint32_t lockout_timer_ms;         /**< Countdown timer for security lockout */

    uds_app_state_t app_state;         /**< Application runtime parameters */
    uds_stats_t     stats;             /**< Server operational metrics */

    /* Flag indicating ECU reset requested */
    bool reset_pending;
    uint8_t reset_type;
} uds_server_t;

/**
 * @brief Initialize the UDS diagnostic server context.
 * @param server Pointer to server context structure.
 */
void uds_server_init(uds_server_t *server);

/**
 * @brief Process an incoming diagnostic request payload.
 *
 * Decodes the Service Identifier (SID), validates session authorization,
 * executes service logic, and produces either a positive response payload
 * or an ISO 14229 Negative Response Code (NRC).
 *
 * @param server Pointer to server context structure.
 * @param req_buf Pointer to received diagnostic request payload.
 * @param req_len Length of request buffer in bytes.
 * @param rsp_buf Pointer to output response buffer.
 * @param rsp_len Output pointer storing generated response length in bytes.
 * @param max_rsp_len Capacity of response buffer.
 * @return true if a response needs to be dispatched, false if response is suppressed.
 */
bool uds_server_process_request(uds_server_t *server,
                                const uint8_t *req_buf,
                                uint16_t req_len,
                                uint8_t *rsp_buf,
                                uint16_t *rsp_len,
                                uint16_t max_rsp_len);

/**
 * @brief Periodic supervisory tick for UDS server timers.
 *
 * Updates S3 session timeout monitor, security lockout countdowns,
 * and background routine executions.
 *
 * @param server Pointer to server context structure.
 * @param delta_ms Milliseconds elapsed since last invocation.
 */
void uds_server_process(uds_server_t *server, uint32_t delta_ms);

/**
 * @brief Compute expected cryptographic security key from seed.
 *
 * Uses automotive polynomial transform:
 * Key = ((Seed ^ 0xA5A55A5A) << 3) | ((Seed ^ 0xA5A55A5A) >> 29) + 0x12345678
 *
 * @param seed 32-bit challenge seed.
 * @return 32-bit calculated key.
 */
uint32_t uds_calculate_key(uint32_t seed);

/**
 * @brief Query current diagnostic server session.
 * @param server Pointer to server context structure.
 * @return Active session ID (0x01: Default, 0x02: Programming, 0x03: Extended).
 */
uint8_t uds_server_get_session(const uds_server_t *server);

/**
 * @brief Query if security access is currently unlocked.
 * @param server Pointer to server context structure.
 * @return true if unlocked, false if locked.
 */
bool uds_server_is_security_unlocked(const uds_server_t *server);

#ifdef __cplusplus
}
#endif

#endif /* UDS_SERVER_H */
