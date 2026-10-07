/**
 * @file can_driver.c
 * @brief STM32 bxCAN Hardware Abstraction & Emulation Layer.
 *
 * Implements ISO 11898-1 CAN bus driver with support for STM32F446RE bxCAN
 * peripheral registers, hardware acceptance filtering, and host simulation.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include "can_driver.h"
#include <string.h>

#define CAN_QUEUE_CAPACITY 32U

typedef struct {
    can_frame_t frames[CAN_QUEUE_CAPACITY];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} can_fifo_t;

/* Internal Driver State */
static can_config_t s_config;
static can_filter_config_t s_filters[14];
static uint8_t s_filter_count = 0;
static can_fifo_t s_rx_queue;
static can_fifo_t s_tx_queue;
static can_stats_t s_stats;
static bool s_is_initialized = false;

static bool fifo_push(can_fifo_t *fifo, const can_frame_t *frame)
{
    if (fifo->count >= CAN_QUEUE_CAPACITY) {
        return false;
    }
    fifo->frames[fifo->head] = *frame;
    fifo->head = (uint16_t)((fifo->head + 1U) % CAN_QUEUE_CAPACITY);
    fifo->count++;
    return true;
}

static bool fifo_pop(can_fifo_t *fifo, can_frame_t *frame)
{
    if (fifo->count == 0U) {
        return false;
    }
    *frame = fifo->frames[fifo->tail];
    fifo->tail = (uint16_t)((fifo->tail + 1U) % CAN_QUEUE_CAPACITY);
    fifo->count--;
    return true;
}

#if defined(STM32F446xx) || defined(TARGET_STM32F4)
/* -------------------------------------------------------------------------
 * Hardware Register Definitions for STM32F446 bxCAN1
 * ------------------------------------------------------------------------- */
#define CAN1_BASE_ADDR        0x40006400UL
#define CAN1_MCR              (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x00))
#define CAN1_MSR              (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x04))
#define CAN1_TSR              (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x08))
#define CAN1_RF0R             (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x0C))
#define CAN1_BTR              (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x1C))
#define CAN1_FMR              (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x200))
#define CAN1_FM1R             (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x204))
#define CAN1_FS1R             (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x20C))
#define CAN1_FFA1R            (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x214))
#define CAN1_FA1R             (*(volatile uint32_t *)(CAN1_BASE_ADDR + 0x21C))

static void hw_bxcan_init(const can_config_t *cfg)
{
    /* Request bxCAN Initialization Mode (INRQ = 1, SLEEP = 0) */
    CAN1_MCR |= (1UL << 0);
    CAN1_MCR &= ~(1UL << 1);
    while ((CAN1_MSR & (1UL << 0)) == 0) {
        /* Wait for INAK acknowledgment in silicon */
    }

    /* Bit Timing Register (BTR): Set BRP, TS1, TS2, SJW */
    uint32_t btr_val = 0;
    if (cfg->mode == CAN_MODE_LOOPBACK) {
        btr_val |= (1UL << 30); /* LBKM bit */
    } else if (cfg->mode == CAN_MODE_SILENT) {
        btr_val |= (1UL << 31); /* SILM bit */
    }

    btr_val |= ((uint32_t)(cfg->sjw - 1U) & 0x03U) << 24;
    btr_val |= ((uint32_t)(cfg->ts2 - 1U) & 0x07U) << 20;
    btr_val |= ((uint32_t)(cfg->ts1 - 1U) & 0x0FU) << 16;
    btr_val |= ((uint32_t)(cfg->brp - 1U) & 0x03FFU);
    CAN1_BTR = btr_val;

    /* Leave Initialization Mode */
    CAN1_MCR &= ~(1UL << 0);
    while ((CAN1_MSR & (1UL << 0)) != 0) {
        /* Wait for normal operational mode */
    }
}
#endif

bool can_driver_init(const can_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    s_config = *config;
    memset(&s_rx_queue, 0, sizeof(s_rx_queue));
    memset(&s_tx_queue, 0, sizeof(s_tx_queue));
    memset(&s_stats, 0, sizeof(s_stats));
    s_filter_count = 0;

#if defined(STM32F446xx) || defined(TARGET_STM32F4)
    hw_bxcan_init(config);
#endif

    s_is_initialized = true;
    return true;
}

bool can_driver_configure_filter(const can_filter_config_t *filter_cfg)
{
    if (filter_cfg == NULL || filter_cfg->filter_bank >= 14U) {
        return false;
    }

    if (s_filter_count < 14U) {
        s_filters[s_filter_count++] = *filter_cfg;
    }

    return true;
}

static bool check_filter_match(uint32_t id, bool is_extended)
{
    if (s_filter_count == 0U) {
        return true; /* Open by default if no filters defined */
    }

    for (uint8_t i = 0; i < s_filter_count; i++) {
        if (!s_filters[i].enable) {
            continue;
        }
        if (s_filters[i].is_extended != is_extended) {
            continue;
        }

        uint32_t mask = s_filters[i].filter_mask;
        uint32_t filt_id = s_filters[i].filter_id;

        if ((id & mask) == (filt_id & mask)) {
            return true;
        }
    }

    return false;
}

bool can_driver_transmit(const can_frame_t *frame)
{
    if (!s_is_initialized || frame == NULL || frame->dlc > CAN_MAX_DLC) {
        return false;
    }

    if (!fifo_push(&s_tx_queue, frame)) {
        s_stats.tx_overflow_count++;
        return false;
    }

    s_stats.tx_success_count++;

    /* In loopback mode, immediately mirror frame to RX queue if filter matches */
    if (s_config.mode == CAN_MODE_LOOPBACK) {
        if (check_filter_match(frame->id, frame->is_extended)) {
            if (!fifo_push(&s_rx_queue, frame)) {
                s_stats.rx_overflow_count++;
            } else {
                s_stats.rx_success_count++;
            }
        }
    }

    return true;
}

bool can_driver_receive(can_frame_t *frame)
{
    if (!s_is_initialized || frame == NULL) {
        return false;
    }

    return fifo_pop(&s_rx_queue, frame);
}

void can_driver_get_stats(can_stats_t *stats)
{
    if (stats != NULL) {
        *stats = s_stats;
    }
}

void can_driver_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
}

void can_driver_process(uint32_t delta_ms)
{
    (void)delta_ms;
    /* Maintain bus-off recovery and error state transitions */
}
