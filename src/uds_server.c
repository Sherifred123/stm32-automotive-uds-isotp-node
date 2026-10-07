/**
 * @file uds_server.c
 * @brief ISO 14229-1 Unified Diagnostic Services (UDS) Server Implementation.
 *
 * Implements deterministic diagnostic state machine, ISO 14229 Negative Response
 * generation, seed-key authentication with brute force protection, and standard DIDs.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include "uds_server.h"
#include <string.h>

/* Cryptographic Constants for Automotive Seed-Key Transform */
#define UDS_KEY_MASK        0xA5A55A5AUL
#define UDS_KEY_ADDEND      0x12345678UL

static const char VIN_STRING[18] = "1HGCR2F83HA000001";
static const char ECU_PART_NO[12] = "STM-446-ECU";
static const uint8_t ECU_SW_VER[4] = {1U, 2U, 0U, 42U}; /* v1.2.0.42 */

static uint32_t s_lfsr_seed = 0x98765432UL;

static uint32_t generate_pseudo_seed(void)
{
    /* 32-bit Galois Linear Feedback Shift Register (LFSR) */
    s_lfsr_seed = (s_lfsr_seed >> 1) ^ (-(s_lfsr_seed & 1UL) & 0xD0000001UL);
    if (s_lfsr_seed == 0UL) {
        s_lfsr_seed = 0x12345678UL;
    }
    return s_lfsr_seed;
}

uint32_t uds_calculate_key(uint32_t seed)
{
    uint32_t x = seed ^ UDS_KEY_MASK;
    /* 32-bit circular rotate left by 3 bits */
    uint32_t rot = ((x << 3) | (x >> 29));
    return rot + UDS_KEY_ADDEND;
}

static void make_nrc(uint8_t sid, uint8_t nrc, uint8_t *rsp_buf, uint16_t *rsp_len)
{
    rsp_buf[0] = UDS_SID_NEGATIVE_RESPONSE;
    rsp_buf[1] = sid;
    rsp_buf[2] = nrc;
    *rsp_len = 3U;
}

void uds_server_init(uds_server_t *server)
{
    if (server == NULL) {
        return;
    }

    memset(server, 0, sizeof(*server));
    server->active_session = UDS_SESSION_DEFAULT;
    server->security_state = UDS_SECURITY_LOCKED;
    server->app_state.speed_governor_kmh = 80U; /* Default 80 km/h */
}

uint8_t uds_server_get_session(const uds_server_t *server)
{
    return (server != NULL) ? server->active_session : UDS_SESSION_DEFAULT;
}

bool uds_server_is_security_unlocked(const uds_server_t *server)
{
    return (server != NULL && server->security_state == UDS_SECURITY_UNLOCKED);
}

void uds_server_process(uds_server_t *server, uint32_t delta_ms)
{
    if (server == NULL) {
        return;
    }

    /* S3 Session Supervision: Revert non-default sessions on inactivity */
    if (server->active_session != UDS_SESSION_DEFAULT) {
        server->s3_timer_ms += delta_ms;
        if (server->s3_timer_ms >= UDS_S3_SERVER_TIMEOUT_MS) {
            server->active_session = UDS_SESSION_DEFAULT;
            server->security_state = UDS_SECURITY_LOCKED;
            server->active_seed = 0U;
            server->s3_timer_ms = 0U;
            server->stats.session_timeouts++;
        }
    }

    /* Security Brute-Force Lockout Countdown */
    if (server->lockout_timer_ms > 0U) {
        if (delta_ms >= server->lockout_timer_ms) {
            server->lockout_timer_ms = 0U;
            server->failed_security_attempts = 0U;
        } else {
            server->lockout_timer_ms -= delta_ms;
        }
    }
}

/* -------------------------------------------------------------------------
 * UDS Service Handlers
 * ------------------------------------------------------------------------- */

static bool handle_diagnostic_session_control(uds_server_t *server,
                                              const uint8_t *req, uint16_t len,
                                              uint8_t *rsp, uint16_t *rsp_len)
{
    if (len != 2U) {
        make_nrc(UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
        return true;
    }

    uint8_t target_session = req[1] & 0x7FU;
    if (target_session != UDS_SESSION_DEFAULT &&
        target_session != UDS_SESSION_PROGRAMMING &&
        target_session != UDS_SESSION_EXTENDED) {
        make_nrc(UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, rsp, rsp_len);
        return true;
    }

    server->active_session = target_session;
    server->s3_timer_ms = 0U;

    /* Transitioning to default session automatically relocks security */
    if (target_session == UDS_SESSION_DEFAULT) {
        server->security_state = UDS_SECURITY_LOCKED;
        server->active_seed = 0U;
    }

    rsp[0] = (uint8_t)(UDS_SID_DIAGNOSTIC_SESSION_CONTROL + UDS_RSP_OFFSET);
    rsp[1] = target_session;
    rsp[2] = 0x00U; /* P2Server_max high byte (50 ms) */
    rsp[3] = 0x32U; /* P2Server_max low byte */
    rsp[4] = 0x01U; /* P2*Server_max high byte (5000 ms resolution / 10ms) */
    rsp[5] = 0xF4U; /* P2*Server_max low byte */
    *rsp_len = 6U;
    return true;
}

static bool handle_tester_present(uds_server_t *server,
                                  const uint8_t *req, uint16_t len,
                                  uint8_t *rsp, uint16_t *rsp_len)
{
    if (len != 2U) {
        make_nrc(UDS_SID_TESTER_PRESENT, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
        return true;
    }

    server->s3_timer_ms = 0U; /* Keep non-default session alive */

    uint8_t subfunction = req[1];
    if ((subfunction & 0x80U) != 0U) {
        /* SuppressPositiveResponseBit is set -> suppress response */
        return false;
    }

    if ((subfunction & 0x7FU) != 0x00U) {
        make_nrc(UDS_SID_TESTER_PRESENT, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, rsp, rsp_len);
        return true;
    }

    rsp[0] = (uint8_t)(UDS_SID_TESTER_PRESENT + UDS_RSP_OFFSET);
    rsp[1] = 0x00U;
    *rsp_len = 2U;
    return true;
}

static bool handle_security_access(uds_server_t *server,
                                   const uint8_t *req, uint16_t len,
                                   uint8_t *rsp, uint16_t *rsp_len)
{
    if (len < 2U) {
        make_nrc(UDS_SID_SECURITY_ACCESS, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
        return true;
    }

    if (server->active_session == UDS_SESSION_DEFAULT) {
        make_nrc(UDS_SID_SECURITY_ACCESS, UDS_NRC_CONDITIONS_NOT_CORRECT, rsp, rsp_len);
        return true;
    }

    if (server->lockout_timer_ms > 0U) {
        make_nrc(UDS_SID_SECURITY_ACCESS, UDS_NRC_REQUIRED_TIME_DELAY_NOT_EXPIRED, rsp, rsp_len);
        return true;
    }

    uint8_t subfunction = req[1] & 0x7FU;

    if (subfunction == 0x01U) {
        /* Request Seed */
        if (len != 2U) {
            make_nrc(UDS_SID_SECURITY_ACCESS, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
            return true;
        }

        rsp[0] = (uint8_t)(UDS_SID_SECURITY_ACCESS + UDS_RSP_OFFSET);
        rsp[1] = 0x01U;

        if (server->security_state == UDS_SECURITY_UNLOCKED) {
            /* Already unlocked: Send seed = 0x00000000 */
            memset(&rsp[2], 0, 4U);
        } else {
            uint32_t seed = generate_pseudo_seed();
            server->active_seed = seed;
            server->security_state = UDS_SECURITY_SEED_ISSUED;

            rsp[2] = (uint8_t)((seed >> 24) & 0xFFU);
            rsp[3] = (uint8_t)((seed >> 16) & 0xFFU);
            rsp[4] = (uint8_t)((seed >> 8) & 0xFFU);
            rsp[5] = (uint8_t)(seed & 0xFFU);
        }
        *rsp_len = 6U;
        return true;
    } else if (subfunction == 0x02U) {
        /* Send Key */
        if (len != 6U) {
            make_nrc(UDS_SID_SECURITY_ACCESS, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
            return true;
        }

        if (server->security_state != UDS_SECURITY_SEED_ISSUED) {
            make_nrc(UDS_SID_SECURITY_ACCESS, UDS_NRC_REQUEST_SEQUENCE_ERROR, rsp, rsp_len);
            return true;
        }

        uint32_t client_key = ((uint32_t)req[2] << 24) |
                              ((uint32_t)req[3] << 16) |
                              ((uint32_t)req[4] << 8)  |
                              (uint32_t)req[5];

        uint32_t expected_key = uds_calculate_key(server->active_seed);

        if (client_key == expected_key) {
            server->security_state = UDS_SECURITY_UNLOCKED;
            server->failed_security_attempts = 0U;
            server->stats.security_unlock_count++;

            rsp[0] = (uint8_t)(UDS_SID_SECURITY_ACCESS + UDS_RSP_OFFSET);
            rsp[1] = 0x02U;
            *rsp_len = 2U;
            return true;
        } else {
            server->security_state = UDS_SECURITY_LOCKED;
            server->failed_security_attempts++;
            server->stats.security_fail_count++;

            if (server->failed_security_attempts >= UDS_SECURITY_MAX_ATTEMPTS) {
                server->lockout_timer_ms = UDS_SECURITY_LOCKOUT_DELAY_MS;
                make_nrc(UDS_SID_SECURITY_ACCESS, UDS_NRC_EXCEEDED_NUMBER_OF_ATTEMPTS, rsp, rsp_len);
            } else {
                make_nrc(UDS_SID_SECURITY_ACCESS, UDS_NRC_INVALID_KEY, rsp, rsp_len);
            }
            return true;
        }
    } else {
        make_nrc(UDS_SID_SECURITY_ACCESS, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, rsp, rsp_len);
        return true;
    }
}

static bool handle_read_data_by_id(uds_server_t *server,
                                   const uint8_t *req, uint16_t len,
                                   uint8_t *rsp, uint16_t *rsp_len)
{
    (void)server;
    if (len != 3U) {
        make_nrc(UDS_SID_READ_DATA_BY_ID, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
        return true;
    }

    uint16_t did = (uint16_t)(((uint16_t)req[1] << 8) | req[2]);
    rsp[0] = (uint8_t)(UDS_SID_READ_DATA_BY_ID + UDS_RSP_OFFSET);
    rsp[1] = req[1];
    rsp[2] = req[2];

    switch (did) {
    case UDS_DID_VIN:
        memcpy(&rsp[3], VIN_STRING, 17U);
        *rsp_len = 3U + 17U; /* 20 bytes total: triggers multi-frame ISO-TP */
        return true;

    case UDS_DID_ECU_PART_NUMBER:
        memcpy(&rsp[3], ECU_PART_NO, 10U);
        *rsp_len = 3U + 10U; /* 13 bytes total: triggers multi-frame ISO-TP */
        return true;

    case UDS_DID_ECU_SW_VERSION:
        memcpy(&rsp[3], ECU_SW_VER, 4U);
        *rsp_len = 3U + 4U; /* 7 bytes: fits in single frame */
        return true;

    case UDS_DID_BMS_TELEMETRY:
        /* 51.2V, -12.5A, 88% SOC, 34°C */
        rsp[3] = 0x02U; rsp[4] = 0x00U; /* 512 (0.1V) */
        rsp[5] = 0xFFU; rsp[6] = 0x83U; /* -125 (0.1A) */
        rsp[7] = 88U;                   /* 88 % */
        rsp[8] = 34U;                   /* 34 °C */
        *rsp_len = 3U + 6U;
        return true;

    case UDS_DID_SPEED_GOVERNOR_LIMIT:
        rsp[3] = server->app_state.speed_governor_kmh;
        *rsp_len = 4U;
        return true;

    case UDS_DID_MCU_HARDWARE_HEALTH:
        rsp[3] = 28U;                    /* Core Temp: 28°C */
        rsp[4] = 0x0CU; rsp[5] = 0xE4U;  /* Vref: 3300 mV */
        rsp[6] = 0x00U; rsp[7] = 0x00U;  /* Bus errors */
        *rsp_len = 3U + 5U;
        return true;

    default:
        make_nrc(UDS_SID_READ_DATA_BY_ID, UDS_NRC_REQUEST_OUT_OF_RANGE, rsp, rsp_len);
        return true;
    }
}

static bool handle_write_data_by_id(uds_server_t *server,
                                    const uint8_t *req, uint16_t len,
                                    uint8_t *rsp, uint16_t *rsp_len)
{
    if (len < 4U) {
        make_nrc(UDS_SID_WRITE_DATA_BY_ID, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
        return true;
    }

    /* WDBI is restricted to Extended Diagnostic Session */
    if (server->active_session != UDS_SESSION_EXTENDED) {
        make_nrc(UDS_SID_WRITE_DATA_BY_ID, UDS_NRC_CONDITIONS_NOT_CORRECT, rsp, rsp_len);
        return true;
    }

    /* WDBI requires Security Access Level 1 */
    if (server->security_state != UDS_SECURITY_UNLOCKED) {
        make_nrc(UDS_SID_WRITE_DATA_BY_ID, UDS_NRC_SECURITY_ACCESS_DENIED, rsp, rsp_len);
        return true;
    }

    uint16_t did = (uint16_t)(((uint16_t)req[1] << 8) | req[2]);

    if (did == UDS_DID_SPEED_GOVERNOR_LIMIT) {
        if (len != 4U) {
            make_nrc(UDS_SID_WRITE_DATA_BY_ID, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
            return true;
        }

        uint8_t speed_limit = req[3];
        if (speed_limit < 20U || speed_limit > 140U) {
            make_nrc(UDS_SID_WRITE_DATA_BY_ID, UDS_NRC_REQUEST_OUT_OF_RANGE, rsp, rsp_len);
            return true;
        }

        server->app_state.speed_governor_kmh = speed_limit;
        rsp[0] = (uint8_t)(UDS_SID_WRITE_DATA_BY_ID + UDS_RSP_OFFSET);
        rsp[1] = req[1];
        rsp[2] = req[2];
        *rsp_len = 3U;
        return true;
    }

    make_nrc(UDS_SID_WRITE_DATA_BY_ID, UDS_NRC_REQUEST_OUT_OF_RANGE, rsp, rsp_len);
    return true;
}

static bool handle_routine_control(uds_server_t *server,
                                   const uint8_t *req, uint16_t len,
                                   uint8_t *rsp, uint16_t *rsp_len)
{
    if (len < 4U) {
        make_nrc(UDS_SID_ROUTINE_CONTROL, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
        return true;
    }

    uint8_t subfunc = req[1] & 0x7FU;
    uint16_t rid = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);

    if (subfunc != 0x01U && subfunc != 0x02U && subfunc != 0x03U) {
        make_nrc(UDS_SID_ROUTINE_CONTROL, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, rsp, rsp_len);
        return true;
    }

    if (rid == UDS_RID_FLASH_INTEGRITY_CHECK) {
        rsp[0] = (uint8_t)(UDS_SID_ROUTINE_CONTROL + UDS_RSP_OFFSET);
        rsp[1] = subfunc;
        rsp[2] = req[2];
        rsp[3] = req[3];
        rsp[4] = 0x00U; /* Routine Status: 0x00 (Passed / Self-Test OK) */
        *rsp_len = 5U;
        return true;
    } else if (rid == UDS_RID_LED_ACTUATOR_TEST) {
        server->app_state.routine_led_running = (subfunc == 0x01U);
        rsp[0] = (uint8_t)(UDS_SID_ROUTINE_CONTROL + UDS_RSP_OFFSET);
        rsp[1] = subfunc;
        rsp[2] = req[2];
        rsp[3] = req[3];
        rsp[4] = 0x00U;
        *rsp_len = 5U;
        return true;
    }

    make_nrc(UDS_SID_ROUTINE_CONTROL, UDS_NRC_REQUEST_OUT_OF_RANGE, rsp, rsp_len);
    return true;
}

static bool handle_read_dtc_info(uds_server_t *server,
                                 const uint8_t *req, uint16_t len,
                                 uint8_t *rsp, uint16_t *rsp_len)
{
    (void)server;
    if (len < 2U) {
        make_nrc(UDS_SID_READ_DTC_INFO, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
        return true;
    }

    uint8_t subfunc = req[1] & 0x7FU;

    if (subfunc == 0x01U) {
        /* ReportNumberOfDTCByStatusMask */
        rsp[0] = (uint8_t)(UDS_SID_READ_DTC_INFO + UDS_RSP_OFFSET);
        rsp[1] = subfunc;
        rsp[2] = UDS_DTC_STATUS_ACTIVE; /* Status Availability Mask */
        rsp[3] = 0x01U;                  /* ISO 14229-1 DTC Format */
        rsp[4] = 0x00U; rsp[5] = 0x02U;  /* 2 Active Faults */
        *rsp_len = 6U;
        return true;
    } else if (subfunc == 0x02U) {
        /* ReportDTCByStatusMask */
        rsp[0] = (uint8_t)(UDS_SID_READ_DTC_INFO + UDS_RSP_OFFSET);
        rsp[1] = subfunc;
        rsp[2] = UDS_DTC_STATUS_ACTIVE;

        /* DTC 1: 0xE00100 (Battery Overtemp) */
        rsp[3] = 0xE0U; rsp[4] = 0x01U; rsp[5] = 0x00U; rsp[6] = UDS_DTC_STATUS_ACTIVE;
        /* DTC 2: 0xC00100 (CAN Bus Interruption) */
        rsp[7] = 0xC0U; rsp[8] = 0x01U; rsp[9] = 0x00U; rsp[10] = UDS_DTC_STATUS_ACTIVE;

        *rsp_len = 11U;
        return true;
    }

    make_nrc(UDS_SID_READ_DTC_INFO, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, rsp, rsp_len);
    return true;
}

static bool handle_ecu_reset(uds_server_t *server,
                             const uint8_t *req, uint16_t len,
                             uint8_t *rsp, uint16_t *rsp_len)
{
    if (len != 2U) {
        make_nrc(UDS_SID_ECU_RESET, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
        return true;
    }

    uint8_t subfunc = req[1] & 0x7FU;
    if (subfunc != 0x01U && subfunc != 0x03U) {
        make_nrc(UDS_SID_ECU_RESET, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, rsp, rsp_len);
        return true;
    }

    server->reset_pending = true;
    server->reset_type = subfunc;

    rsp[0] = (uint8_t)(UDS_SID_ECU_RESET + UDS_RSP_OFFSET);
    rsp[1] = subfunc;
    *rsp_len = 2U;
    return true;
}

static bool handle_clear_dtc(uds_server_t *server,
                             const uint8_t *req, uint16_t len,
                             uint8_t *rsp, uint16_t *rsp_len)
{
    (void)server;
    (void)req;
    if (len != 4U) {
        make_nrc(UDS_SID_CLEAR_DIAGNOSTIC_INFO, UDS_NRC_INCORRECT_MESSAGE_LENGTH, rsp, rsp_len);
        return true;
    }

    rsp[0] = (uint8_t)(UDS_SID_CLEAR_DIAGNOSTIC_INFO + UDS_RSP_OFFSET);
    *rsp_len = 1U;
    return true;
}

bool uds_server_process_request(uds_server_t *server,
                                const uint8_t *req_buf,
                                uint16_t req_len,
                                uint8_t *rsp_buf,
                                uint16_t *rsp_len,
                                uint16_t max_rsp_len)
{
    if (server == NULL || req_buf == NULL || rsp_buf == NULL || rsp_len == NULL || req_len == 0U) {
        return false;
    }

    (void)max_rsp_len;
    server->stats.requests_received++;

    uint8_t sid = req_buf[0];
    bool should_send = false;

    switch (sid) {
    case UDS_SID_DIAGNOSTIC_SESSION_CONTROL:
        should_send = handle_diagnostic_session_control(server, req_buf, req_len, rsp_buf, rsp_len);
        break;

    case UDS_SID_TESTER_PRESENT:
        should_send = handle_tester_present(server, req_buf, req_len, rsp_buf, rsp_len);
        break;

    case UDS_SID_SECURITY_ACCESS:
        should_send = handle_security_access(server, req_buf, req_len, rsp_buf, rsp_len);
        break;

    case UDS_SID_READ_DATA_BY_ID:
        should_send = handle_read_data_by_id(server, req_buf, req_len, rsp_buf, rsp_len);
        break;

    case UDS_SID_WRITE_DATA_BY_ID:
        should_send = handle_write_data_by_id(server, req_buf, req_len, rsp_buf, rsp_len);
        break;

    case UDS_SID_ROUTINE_CONTROL:
        should_send = handle_routine_control(server, req_buf, req_len, rsp_buf, rsp_len);
        break;

    case UDS_SID_READ_DTC_INFO:
        should_send = handle_read_dtc_info(server, req_buf, req_len, rsp_buf, rsp_len);
        break;

    case UDS_SID_CLEAR_DIAGNOSTIC_INFO:
        should_send = handle_clear_dtc(server, req_buf, req_len, rsp_buf, rsp_len);
        break;

    case UDS_SID_ECU_RESET:
        should_send = handle_ecu_reset(server, req_buf, req_len, rsp_buf, rsp_len);
        break;

    default:
        make_nrc(sid, UDS_NRC_SERVICE_NOT_SUPPORTED, rsp_buf, rsp_len);
        should_send = true;
        break;
    }

    if (should_send) {
        if (rsp_buf[0] == UDS_SID_NEGATIVE_RESPONSE) {
            server->stats.negative_responses++;
        } else {
            server->stats.positive_responses++;
        }
    }

    return should_send;
}
