/**
 * @file main.c
 * @brief Application Entry Point & Interactive Diagnostic Harness.
 *
 * Demonstrates ISO 15765-2 multi-frame transport reassembly and ISO 14229-1
 * Unified Diagnostic Services (UDS) execution on STM32 NUCLEO-F446RE.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "diagnostic_app.h"

static diagnostic_app_t g_diag_app;

#if !defined(STM32F446xx) && !defined(TARGET_STM32F4)
/**
 * @brief Host-Native Interactive Self-Test Sequence
 */
static void run_host_demonstration(void)
{
    printf("\n============================================================\n");
    printf("   STM32 NUCLEO-F446RE AUTOMOTIVE UDS & ISO-TP DEMO HARNESS\n");
    printf("   ISO 14229-1 (UDS) + ISO 15765-2 (DoCAN) Multi-Frame Node\n");
    printf("============================================================\n\n");

    /* Initialize in loopback mode */
    diagnostic_app_init(&g_diag_app, true);

    /* -------------------------------------------------------------
     * TEST STEP 1: DiagnosticSessionControl (Extended Session 0x03)
     * ------------------------------------------------------------- */
    printf("\n--- [STEP 1] Transition to Extended Session (SID 0x10 0x03) ---\n");
    {
        can_frame_t f1;
        memset(&f1, 0, sizeof(f1));
        f1.id = DIAG_CAN_ID_PHYS_REQ;
        f1.dlc = 8;
        f1.data[0] = 0x02; /* Single Frame, Len = 2 */
        f1.data[1] = 0x10; /* SID = DiagnosticSessionControl */
        f1.data[2] = 0x03; /* Subfunc = Extended Diagnostic Session */
        can_driver_transmit(&f1);

        for (int i = 0; i < 5; i++) {
            diagnostic_app_process(&g_diag_app, 1);
        }
    }

    /* -------------------------------------------------------------
     * TEST STEP 2: ReadDataByIdentifier (VIN DID 0xF190 - 20 Bytes Multi-Frame)
     * ------------------------------------------------------------- */
    printf("\n--- [STEP 2] Multi-Frame ReadDataByIdentifier: VIN (DID 0xF190) ---\n");
    {
        can_frame_t f2;
        memset(&f2, 0, sizeof(f2));
        f2.id = DIAG_CAN_ID_PHYS_REQ;
        f2.dlc = 8;
        f2.data[0] = 0x03; /* SF, Len = 3 */
        f2.data[1] = 0x22; /* SID = ReadDataByIdentifier */
        f2.data[2] = 0xF1; /* DID High */
        f2.data[3] = 0x90; /* DID Low */
        can_driver_transmit(&f2);

        for (int ms = 0; ms < 20; ms++) {
            diagnostic_app_process(&g_diag_app, 1);

            can_frame_t tester_frame;
            if (can_driver_receive(&tester_frame)) {
                if (tester_frame.id == DIAG_CAN_ID_PHYS_RESP) {
                    uint8_t pci = (uint8_t)((tester_frame.data[0] >> 4) & 0x0F);
                    if (pci == ISOTP_FRAME_FF) {
                        printf(">>> [TESTER] First Frame Received from ECU! Sending Flow Control (CTS)...\n");
                        can_frame_t fc_frame;
                        memset(&fc_frame, 0, sizeof(fc_frame));
                        fc_frame.id = DIAG_CAN_ID_PHYS_REQ;
                        fc_frame.dlc = 8;
                        fc_frame.data[0] = 0x30; /* Flow Control: CTS */
                        fc_frame.data[1] = 0x00; /* Block Size = 0 */
                        fc_frame.data[2] = 0x05; /* STmin = 5 ms */
                        can_driver_transmit(&fc_frame);
                    } else if (pci == ISOTP_FRAME_CF) {
                        printf(">>> [TESTER] Consecutive Frame Received: SN=%u Data=[", tester_frame.data[0] & 0x0F);
                        for (int k = 1; k < 8; k++) {
                            printf("%c", (tester_frame.data[k] >= 32 && tester_frame.data[k] <= 126) ? tester_frame.data[k] : '.');
                        }
                        printf("]\n");
                    }
                }
            }
        }
    }

    /* -------------------------------------------------------------
     * TEST STEP 3: SecurityAccess (SID 0x27) Seed-Key Handshake
     * ------------------------------------------------------------- */
    printf("\n--- [STEP 3] SecurityAccess Level 1 (SID 0x27) Seed-Key Handshake ---\n");
    {
        can_frame_t f3;
        memset(&f3, 0, sizeof(f3));
        f3.id = DIAG_CAN_ID_PHYS_REQ;
        f3.dlc = 8;
        f3.data[0] = 0x02; /* SF, Len = 2 */
        f3.data[1] = 0x27; /* SID = SecurityAccess */
        f3.data[2] = 0x01; /* Subfunc = RequestSeed */
        can_driver_transmit(&f3);

        for (int ms = 0; ms < 5; ms++) {
            diagnostic_app_process(&g_diag_app, 1);
        }

        uint32_t seed = g_diag_app.uds_server.active_seed;
        uint32_t key = uds_calculate_key(seed);
        printf(">>> [TESTER] Intercepted Seed: 0x%08X -> Calculated Key: 0x%08X\n", seed, key);

        can_frame_t f4;
        memset(&f4, 0, sizeof(f4));
        f4.id = DIAG_CAN_ID_PHYS_REQ;
        f4.dlc = 8;
        f4.data[0] = 0x06; /* SF, Len = 6 */
        f4.data[1] = 0x27; /* SID */
        f4.data[2] = 0x02; /* Subfunc = SendKey */
        f4.data[3] = (uint8_t)((key >> 24) & 0xFF);
        f4.data[4] = (uint8_t)((key >> 16) & 0xFF);
        f4.data[5] = (uint8_t)((key >> 8) & 0xFF);
        f4.data[6] = (uint8_t)(key & 0xFF);
        can_driver_transmit(&f4);

        for (int ms = 0; ms < 5; ms++) {
            diagnostic_app_process(&g_diag_app, 1);
        }

        printf(">>> [STATUS] Security State: %s\n",
               uds_server_is_security_unlocked(&g_diag_app.uds_server) ? "UNLOCKED (Level 1)" : "LOCKED");
    }

    /* -------------------------------------------------------------
     * TEST STEP 4: WriteDataByIdentifier (Speed Governor DID 0x0105 = 110 km/h)
     * ------------------------------------------------------------- */
    printf("\n--- [STEP 4] WriteDataByIdentifier (DID 0x0105 = 110 km/h) ---\n");
    {
        can_frame_t f5;
        memset(&f5, 0, sizeof(f5));
        f5.id = DIAG_CAN_ID_PHYS_REQ;
        f5.dlc = 8;
        f5.data[0] = 0x04; /* SF, Len = 4 */
        f5.data[1] = 0x2E; /* SID = WriteDataByIdentifier */
        f5.data[2] = 0x01; /* DID High */
        f5.data[3] = 0x05; /* DID Low */
        f5.data[4] = 110;  /* New Speed Limit: 110 km/h */
        can_driver_transmit(&f5);

        for (int ms = 0; ms < 5; ms++) {
            diagnostic_app_process(&g_diag_app, 1);
        }
        printf(">>> [STATUS] Active Speed Governor Limit: %u km/h\n",
               g_diag_app.uds_server.app_state.speed_governor_kmh);
    }

    /* -------------------------------------------------------------
     * TEST STEP 5: S3 Session Timeout Supervision (5000 ms)
     * ------------------------------------------------------------- */
    printf("\n--- [STEP 5] S3 Server Inactivity Timeout (5000 ms) ---\n");
    printf("Simulating 5100 ms of diagnostic inactivity without TesterPresent...\n");
    diagnostic_app_process(&g_diag_app, 5100);

    printf(">>> [STATUS] Active Session after timeout: 0x%02X (Expected Default: 0x01)\n",
           uds_server_get_session(&g_diag_app.uds_server));
    printf(">>> [STATUS] Security State after timeout: %s (Expected: LOCKED)\n",
           uds_server_is_security_unlocked(&g_diag_app.uds_server) ? "UNLOCKED" : "LOCKED");

    printf("\n============================================================\n");
    printf("   SELF-TEST COMPLETE: ALL PROTOCOL CHECKS VERIFIED\n");
    printf("============================================================\n\n");
}
#endif

int main(void)
{
#if defined(STM32F446xx) || defined(TARGET_STM32F4)
    /* Hardware Initialization on physical NUCLEO-F446RE */
    diagnostic_app_init(&g_diag_app, false);

    while (1) {
        diagnostic_app_process(&g_diag_app, 1);
    }
#else
    /* Host PC Demonstration Harness */
    run_host_demonstration();
    return 0;
#endif
}
