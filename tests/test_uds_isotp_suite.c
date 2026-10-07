/**
 * @file test_uds_isotp_suite.c
 * @brief Automated Unity Unit Test Suite for Automotive UDS & ISO-TP Stack.
 *
 * Validates ISO 15765-2 network layer framing, segmentation, sequence verification,
 * Flow Control, ISO 14229-1 UDS diagnostic services, and cryptographic seed-key security.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include "unity.h"
#include "can_driver.h"
#include "isotp.h"
#include "uds_server.h"
#include "diagnostic_app.h"

static isotp_channel_t s_channel;
static uds_server_t    s_server;

void setUp(void)
{
    can_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = CAN_MODE_LOOPBACK;
    cfg.baudrate_kbps = 500U;
    cfg.brp = 5U;
    cfg.ts1 = 15U;
    cfg.ts2 = 2U;
    cfg.sjw = 1U;
    can_driver_init(&cfg);

    can_filter_config_t filt;
    memset(&filt, 0, sizeof(filt));
    filt.filter_bank = 0U;
    filt.filter_id = DIAG_CAN_ID_PHYS_REQ; /* 0x7E0 */
    filt.filter_mask = CAN_STD_ID_MASK;
    filt.enable = true;
    can_driver_configure_filter(&filt);

    can_filter_config_t filt_resp;
    memset(&filt_resp, 0, sizeof(filt_resp));
    filt_resp.filter_bank = 1U;
    filt_resp.filter_id = DIAG_CAN_ID_PHYS_RESP; /* 0x7E8 */
    filt_resp.filter_mask = CAN_STD_ID_MASK;
    filt_resp.enable = true;
    can_driver_configure_filter(&filt_resp);

    isotp_config_t isotp_cfg;
    memset(&isotp_cfg, 0, sizeof(isotp_cfg));
    isotp_cfg.rx_id = DIAG_CAN_ID_PHYS_REQ;
    isotp_cfg.tx_id = DIAG_CAN_ID_PHYS_RESP;
    isotp_cfg.block_size = 8U;
    isotp_cfg.st_min_ms = 10U;
    isotp_cfg.enable_padding = true;
    isotp_cfg.padding_val = ISOTP_PADDING_BYTE;
    isotp_init(&s_channel, &isotp_cfg);

    uds_server_init(&s_server);
}

void tearDown(void)
{
    /* Clean up after each test */
}

/* -------------------------------------------------------------------------
 * CAN Driver & Filter Tests
 * ------------------------------------------------------------------------- */
static void test_can_driver_transmit_and_loopback(void)
{
    can_frame_t tx_frame;
    memset(&tx_frame, 0, sizeof(tx_frame));
    tx_frame.id = DIAG_CAN_ID_PHYS_REQ;
    tx_frame.dlc = 8U;
    tx_frame.data[0] = 0xAA;
    tx_frame.data[1] = 0x55;

    TEST_ASSERT_TRUE(can_driver_transmit(&tx_frame));

    can_frame_t rx_frame;
    TEST_ASSERT_TRUE(can_driver_receive(&rx_frame));
    TEST_ASSERT_EQUAL_HEX32(DIAG_CAN_ID_PHYS_REQ, rx_frame.id);
    TEST_ASSERT_EQUAL_HEX8(0xAA, rx_frame.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x55, rx_frame.data[1]);
}

static void test_can_driver_hardware_filter_rejection(void)
{
    can_frame_t rogue_frame;
    memset(&rogue_frame, 0, sizeof(rogue_frame));
    rogue_frame.id = 0x123U; /* Unauthorized CAN ID */
    rogue_frame.dlc = 8U;

    can_driver_transmit(&rogue_frame);

    can_frame_t rx_frame;
    /* Rogue frame must be rejected by hardware filter */
    TEST_ASSERT_FALSE(can_driver_receive(&rx_frame));
}

/* -------------------------------------------------------------------------
 * ISO 15765-2 (ISO-TP) Network Layer Tests
 * ------------------------------------------------------------------------- */
static void test_isotp_single_frame_reception(void)
{
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = DIAG_CAN_ID_PHYS_REQ;
    frame.dlc = 8U;
    frame.data[0] = 0x03U; /* SF, DL = 3 */
    frame.data[1] = 0x22U; /* SID = ReadDataByIdentifier */
    frame.data[2] = 0xF1U;
    frame.data[3] = 0x90U;

    TEST_ASSERT_EQUAL_INT(ISOTP_RESULT_OK, isotp_receive_can_frame(&s_channel, &frame));

    uint8_t msg[16];
    uint16_t len = 0;
    TEST_ASSERT_TRUE(isotp_get_received_message(&s_channel, msg, &len, sizeof(msg)));
    TEST_ASSERT_EQUAL_INT(3, len);
    TEST_ASSERT_EQUAL_HEX8(0x22, msg[0]);
    TEST_ASSERT_EQUAL_HEX8(0xF1, msg[1]);
    TEST_ASSERT_EQUAL_HEX8(0x90, msg[2]);
}

static void test_isotp_single_frame_transmission(void)
{
    uint8_t payload[4] = {0x62, 0xF1, 0x89, 0x01};
    TEST_ASSERT_EQUAL_INT(ISOTP_RESULT_OK, isotp_transmit(&s_channel, payload, 4U));

    can_frame_t out_frame;
    TEST_ASSERT_TRUE(can_driver_receive(&out_frame));
    TEST_ASSERT_EQUAL_HEX32(DIAG_CAN_ID_PHYS_RESP, out_frame.id);
    TEST_ASSERT_EQUAL_HEX8(0x04, out_frame.data[0]); /* SF DL = 4 */
    TEST_ASSERT_EQUAL_HEX8(0x62, out_frame.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, out_frame.data[5]); /* Verifies ISO padding */
}

static void test_isotp_multi_frame_reassembly(void)
{
    /* First Frame: Total DL = 12 bytes */
    can_frame_t ff;
    memset(&ff, 0, sizeof(ff));
    ff.id = DIAG_CAN_ID_PHYS_REQ;
    ff.dlc = 8U;
    ff.data[0] = 0x10U; /* FF high nibble */
    ff.data[1] = 12U;   /* FF DL = 12 bytes */
    for (int i = 0; i < 6; i++) {
        ff.data[2 + i] = (uint8_t)(0x10 + i);
    }
    TEST_ASSERT_EQUAL_INT(ISOTP_RESULT_OK, isotp_receive_can_frame(&s_channel, &ff));

    /* Verify Flow Control frame was dispatched */
    can_frame_t fc;
    TEST_ASSERT_TRUE(can_driver_receive(&fc));
    TEST_ASSERT_EQUAL_HEX8(0x30, fc.data[0]); /* FC: CTS */
    TEST_ASSERT_EQUAL_HEX8(8, fc.data[1]);    /* BS = 8 */
    TEST_ASSERT_EQUAL_HEX8(10, fc.data[2]);   /* STmin = 10 ms */

    /* Consecutive Frame 1 (SN = 1) */
    can_frame_t cf;
    memset(&cf, 0, sizeof(cf));
    cf.id = DIAG_CAN_ID_PHYS_REQ;
    cf.dlc = 8U;
    cf.data[0] = 0x21U; /* CF, SN = 1 */
    for (int i = 0; i < 6; i++) {
        cf.data[1 + i] = (uint8_t)(0x16 + i);
    }
    TEST_ASSERT_EQUAL_INT(ISOTP_RESULT_OK, isotp_receive_can_frame(&s_channel, &cf));

    /* Verify reassembled 12-byte message */
    uint8_t msg[16];
    uint16_t len = 0;
    TEST_ASSERT_TRUE(isotp_get_received_message(&s_channel, msg, &len, sizeof(msg)));
    TEST_ASSERT_EQUAL_INT(12, len);
    for (int i = 0; i < 12; i++) {
        TEST_ASSERT_EQUAL_HEX8((uint8_t)(0x10 + i), msg[i]);
    }
}

static void test_isotp_sequence_counter_violation(void)
{
    /* Send First Frame */
    can_frame_t ff;
    memset(&ff, 0, sizeof(ff));
    ff.id = DIAG_CAN_ID_PHYS_REQ;
    ff.dlc = 8U;
    ff.data[0] = 0x10U;
    ff.data[1] = 14U;
    TEST_ASSERT_EQUAL_INT(ISOTP_RESULT_OK, isotp_receive_can_frame(&s_channel, &ff));

    /* Flush Flow Control */
    can_frame_t fc;
    can_driver_receive(&fc);

    /* Send wrong Sequence Number (Expect SN=1, but send SN=5) */
    can_frame_t cf_bad;
    memset(&cf_bad, 0, sizeof(cf_bad));
    cf_bad.id = DIAG_CAN_ID_PHYS_REQ;
    cf_bad.dlc = 8U;
    cf_bad.data[0] = 0x25U; /* SN = 5 */

    TEST_ASSERT_EQUAL_INT(ISOTP_RESULT_WRONG_SEQUENCE, isotp_receive_can_frame(&s_channel, &cf_bad));
}

static void test_isotp_cr_timeout(void)
{
    can_frame_t ff;
    memset(&ff, 0, sizeof(ff));
    ff.id = DIAG_CAN_ID_PHYS_REQ;
    ff.dlc = 8U;
    ff.data[0] = 0x10U;
    ff.data[1] = 15U;
    isotp_receive_can_frame(&s_channel, &ff);

    /* Flush FC */
    can_frame_t fc;
    can_driver_receive(&fc);

    /* Simulate 1050 ms elapsing without receiving Consecutive Frame */
    isotp_process(&s_channel, 1050U);

    /* Channel state should have timed out and reset to IDLE */
    TEST_ASSERT_EQUAL_INT(ISOTP_RX_STATE_IDLE, s_channel.rx_state);
}

/* -------------------------------------------------------------------------
 * ISO 14229-1 (UDS) Server Diagnostic Tests
 * ------------------------------------------------------------------------- */
static void test_uds_session_transition(void)
{
    uint8_t req[2] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED};
    uint8_t rsp[16];
    uint16_t rsp_len = 0;

    TEST_ASSERT_TRUE(uds_server_process_request(&s_server, req, 2, rsp, &rsp_len, sizeof(rsp)));
    TEST_ASSERT_EQUAL_HEX8(0x50, rsp[0]);
    TEST_ASSERT_EQUAL_HEX8(UDS_SESSION_EXTENDED, rsp[1]);
    TEST_ASSERT_EQUAL_INT(6, rsp_len);
    TEST_ASSERT_EQUAL_INT(UDS_SESSION_EXTENDED, uds_server_get_session(&s_server));
}

static void test_uds_tester_present_suppression(void)
{
    /* TesterPresent with SuppressPositiveResponseBit (0x80) */
    uint8_t req[2] = {UDS_SID_TESTER_PRESENT, 0x80U};
    uint8_t rsp[16];
    uint16_t rsp_len = 0;

    /* Response should be suppressed (returns false) */
    TEST_ASSERT_FALSE(uds_server_process_request(&s_server, req, 2, rsp, &rsp_len, sizeof(rsp)));
}

static void test_uds_rdbi_vin_length(void)
{
    uint8_t req[3] = {UDS_SID_READ_DATA_BY_ID, 0xF1, 0x90};
    uint8_t rsp[32];
    uint16_t rsp_len = 0;

    TEST_ASSERT_TRUE(uds_server_process_request(&s_server, req, 3, rsp, &rsp_len, sizeof(rsp)));
    TEST_ASSERT_EQUAL_HEX8(0x62, rsp[0]);
    TEST_ASSERT_EQUAL_HEX8(0xF1, rsp[1]);
    TEST_ASSERT_EQUAL_HEX8(0x90, rsp[2]);
    TEST_ASSERT_EQUAL_INT(20, rsp_len); /* 3 header bytes + 17 VIN ASCII bytes */
}

static void test_uds_rdbi_unknown_did_nrc_31(void)
{
    uint8_t req[3] = {UDS_SID_READ_DATA_BY_ID, 0x99, 0x99};
    uint8_t rsp[16];
    uint16_t rsp_len = 0;

    TEST_ASSERT_TRUE(uds_server_process_request(&s_server, req, 3, rsp, &rsp_len, sizeof(rsp)));
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_NEGATIVE_RESPONSE, rsp[0]);
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_READ_DATA_BY_ID, rsp[1]);
    TEST_ASSERT_EQUAL_HEX8(UDS_NRC_REQUEST_OUT_OF_RANGE, rsp[2]);
}

static void test_uds_security_access_seed_and_key(void)
{
    /* Transition to Extended Session first */
    uint8_t sess_req[2] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED};
    uint8_t rsp[16];
    uint16_t rsp_len = 0;
    uds_server_process_request(&s_server, sess_req, 2, rsp, &rsp_len, sizeof(rsp));

    /* Request Seed */
    uint8_t seed_req[2] = {UDS_SID_SECURITY_ACCESS, 0x01U};
    uds_server_process_request(&s_server, seed_req, 2, rsp, &rsp_len, sizeof(rsp));
    TEST_ASSERT_EQUAL_HEX8(0x67, rsp[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, rsp[1]);

    uint32_t seed = s_server.active_seed;
    TEST_ASSERT_TRUE(seed != 0U);

    /* Compute key */
    uint32_t key = uds_calculate_key(seed);

    /* Send Key */
    uint8_t key_req[6] = {
        UDS_SID_SECURITY_ACCESS, 0x02U,
        (uint8_t)((key >> 24) & 0xFF),
        (uint8_t)((key >> 16) & 0xFF),
        (uint8_t)((key >> 8) & 0xFF),
        (uint8_t)(key & 0xFF)
    };

    uds_server_process_request(&s_server, key_req, 6, rsp, &rsp_len, sizeof(rsp));
    TEST_ASSERT_EQUAL_HEX8(0x67, rsp[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02, rsp[1]);
    TEST_ASSERT_TRUE(uds_server_is_security_unlocked(&s_server));
}

static void test_uds_security_brute_force_lockout(void)
{
    /* Transition to Extended Session */
    uint8_t sess_req[2] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED};
    uint8_t rsp[16];
    uint16_t rsp_len = 0;
    uds_server_process_request(&s_server, sess_req, 2, rsp, &rsp_len, sizeof(rsp));

    /* Attempt 1: Bad key */
    uint8_t seed_req[2] = {UDS_SID_SECURITY_ACCESS, 0x01U};
    uds_server_process_request(&s_server, seed_req, 2, rsp, &rsp_len, sizeof(rsp));
    uint8_t bad_key[6] = {UDS_SID_SECURITY_ACCESS, 0x02U, 0xDE, 0xAD, 0xBE, 0xEF};
    uds_server_process_request(&s_server, bad_key, 6, rsp, &rsp_len, sizeof(rsp));
    TEST_ASSERT_EQUAL_HEX8(UDS_NRC_INVALID_KEY, rsp[2]);

    /* Attempt 2: Bad key */
    uds_server_process_request(&s_server, seed_req, 2, rsp, &rsp_len, sizeof(rsp));
    uds_server_process_request(&s_server, bad_key, 6, rsp, &rsp_len, sizeof(rsp));
    TEST_ASSERT_EQUAL_HEX8(UDS_NRC_INVALID_KEY, rsp[2]);

    /* Attempt 3: Bad key -> Lockout! */
    uds_server_process_request(&s_server, seed_req, 2, rsp, &rsp_len, sizeof(rsp));
    uds_server_process_request(&s_server, bad_key, 6, rsp, &rsp_len, sizeof(rsp));
    TEST_ASSERT_EQUAL_HEX8(UDS_NRC_EXCEEDED_NUMBER_OF_ATTEMPTS, rsp[2]);

    /* Next attempt during lockout must return NRC 0x37 */
    uds_server_process_request(&s_server, seed_req, 2, rsp, &rsp_len, sizeof(rsp));
    TEST_ASSERT_EQUAL_HEX8(UDS_NRC_REQUIRED_TIME_DELAY_NOT_EXPIRED, rsp[2]);
}

static void test_uds_wdbi_security_enforcement(void)
{
    /* In Extended Session but locked: WDBI should return NRC 0x33 */
    uint8_t sess_req[2] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED};
    uint8_t rsp[16];
    uint16_t rsp_len = 0;
    uds_server_process_request(&s_server, sess_req, 2, rsp, &rsp_len, sizeof(rsp));

    uint8_t wdbi_req[4] = {UDS_SID_WRITE_DATA_BY_ID, 0x01, 0x05, 95};
    uds_server_process_request(&s_server, wdbi_req, 4, rsp, &rsp_len, sizeof(rsp));
    TEST_ASSERT_EQUAL_HEX8(UDS_SID_NEGATIVE_RESPONSE, rsp[0]);
    TEST_ASSERT_EQUAL_HEX8(UDS_NRC_SECURITY_ACCESS_DENIED, rsp[2]);
}

static void test_uds_s3_session_timeout(void)
{
    /* Enter Extended Session */
    uint8_t sess_req[2] = {UDS_SID_DIAGNOSTIC_SESSION_CONTROL, UDS_SESSION_EXTENDED};
    uint8_t rsp[16];
    uint16_t rsp_len = 0;
    uds_server_process_request(&s_server, sess_req, 2, rsp, &rsp_len, sizeof(rsp));
    TEST_ASSERT_EQUAL_INT(UDS_SESSION_EXTENDED, uds_server_get_session(&s_server));

    /* Simulate 5001 ms of inactivity */
    uds_server_process(&s_server, 5001U);

    /* Should automatically revert to Default Session */
    TEST_ASSERT_EQUAL_INT(UDS_SESSION_DEFAULT, uds_server_get_session(&s_server));
}

int main(void)
{
    UnityBegin("Automotive UDS & ISO-TP Stack Unit Test Suite");

    /* CAN Hardware Driver Tests */
    RUN_TEST(test_can_driver_transmit_and_loopback);
    RUN_TEST(test_can_driver_hardware_filter_rejection);

    /* ISO-TP Network Layer Tests */
    RUN_TEST(test_isotp_single_frame_reception);
    RUN_TEST(test_isotp_single_frame_transmission);
    RUN_TEST(test_isotp_multi_frame_reassembly);
    RUN_TEST(test_isotp_sequence_counter_violation);
    RUN_TEST(test_isotp_cr_timeout);

    /* UDS Diagnostic Server Tests */
    RUN_TEST(test_uds_session_transition);
    RUN_TEST(test_uds_tester_present_suppression);
    RUN_TEST(test_uds_rdbi_vin_length);
    RUN_TEST(test_uds_rdbi_unknown_did_nrc_31);
    RUN_TEST(test_uds_security_access_seed_and_key);
    RUN_TEST(test_uds_security_brute_force_lockout);
    RUN_TEST(test_uds_wdbi_security_enforcement);
    RUN_TEST(test_uds_s3_session_timeout);

    return UnityEnd();
}
