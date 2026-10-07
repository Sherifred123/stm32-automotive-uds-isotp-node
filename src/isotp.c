/**
 * @file isotp.c
 * @brief ISO 15765-2 (DoCAN) Transport Layer Implementation.
 *
 * Implements deterministic segmentation, multi-frame reassembly, flow control,
 * sequence counter verification, and ISO-TP timeouts.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include "isotp.h"
#include <string.h>

static bool send_flow_control(isotp_channel_t *channel, uint8_t flow_status)
{
    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    frame.id = channel->config.tx_id;
    frame.is_extended = false;
    frame.is_rtr = false;
    frame.dlc = 8U;

    frame.data[0] = (uint8_t)((ISOTP_FRAME_FC << 4) | (flow_status & 0x0FU));
    frame.data[1] = channel->config.block_size;
    frame.data[2] = channel->config.st_min_ms;

    if (channel->config.enable_padding) {
        for (uint8_t i = 3U; i < 8U; i++) {
            frame.data[i] = channel->config.padding_val;
        }
    }

    bool ok = can_driver_transmit(&frame);
    if (ok) {
        channel->tx_frames_count++;
    }
    return ok;
}

void isotp_init(isotp_channel_t *channel, const isotp_config_t *config)
{
    if (channel == NULL || config == NULL) {
        return;
    }

    memset(channel, 0, sizeof(*channel));
    channel->config = *config;

    channel->rx_state = ISOTP_RX_STATE_IDLE;
    channel->tx_state = ISOTP_TX_STATE_IDLE;
}

isotp_result_t isotp_receive_can_frame(isotp_channel_t *channel, const can_frame_t *frame)
{
    if (channel == NULL || frame == NULL) {
        return ISOTP_RESULT_INVALID_FRAME;
    }

    /* Verify accepted Diagnostic Request Identifier (Physical 0x7E0 or Functional 0x7DF) */
    if (frame->id != channel->config.rx_id && frame->id != 0x7DFU) {
        return ISOTP_RESULT_UNEXPECTED_PDU;
    }

    if (frame->dlc == 0U) {
        return ISOTP_RESULT_INVALID_FRAME;
    }

    channel->rx_frames_count++;
    uint8_t pci_type = (uint8_t)((frame->data[0] >> 4) & 0x0FU);

    switch (pci_type) {
    case ISOTP_FRAME_SF: {
        uint8_t sf_dl = (uint8_t)(frame->data[0] & 0x0FU);
        if (sf_dl == 0U || sf_dl > 7U || sf_dl > (frame->dlc - 1U)) {
            channel->error_count++;
            return ISOTP_RESULT_INVALID_FRAME;
        }

        memcpy(channel->rx_buffer, &frame->data[1], sf_dl);
        channel->rx_total_len = sf_dl;
        channel->rx_index = sf_dl;
        channel->rx_state = ISOTP_RX_STATE_COMPLETE;
        return ISOTP_RESULT_OK;
    }

    case ISOTP_FRAME_FF: {
        if (frame->dlc < 8U) {
            channel->error_count++;
            return ISOTP_RESULT_INVALID_FRAME;
        }

        uint16_t ff_dl = (uint16_t)(((uint16_t)(frame->data[0] & 0x0FU) << 8) | frame->data[1]);
        if (ff_dl < 8U) {
            channel->error_count++;
            return ISOTP_RESULT_INVALID_FRAME;
        }

        if (ff_dl > ISOTP_BUF_SIZE) {
            /* Receiver buffer overflow per ISO 15765-2 */
            send_flow_control(channel, ISOTP_FS_OVFLW);
            channel->error_count++;
            channel->rx_state = ISOTP_RX_STATE_IDLE;
            return ISOTP_RESULT_BUFFER_OVERFLOW;
        }

        channel->rx_total_len = ff_dl;
        memcpy(channel->rx_buffer, &frame->data[2], 6U);
        channel->rx_index = 6U;
        channel->rx_expected_sn = 1U;
        channel->rx_block_counter = 0U;
        channel->rx_timer_ms = 0U;

        /* Immediately respond with Flow Control: CTS */
        send_flow_control(channel, ISOTP_FS_CTS);
        channel->rx_state = ISOTP_RX_STATE_WAIT_CF;
        return ISOTP_RESULT_OK;
    }

    case ISOTP_FRAME_CF: {
        if (channel->rx_state != ISOTP_RX_STATE_WAIT_CF) {
            return ISOTP_RESULT_UNEXPECTED_PDU;
        }

        uint8_t sn = (uint8_t)(frame->data[0] & 0x0FU);
        if (sn != channel->rx_expected_sn) {
            /* Sequence error! Abort reception */
            channel->rx_state = ISOTP_RX_STATE_IDLE;
            channel->error_count++;
            return ISOTP_RESULT_WRONG_SEQUENCE;
        }

        channel->rx_expected_sn = (uint8_t)((channel->rx_expected_sn + 1U) & 0x0FU);
        uint16_t remaining = (uint16_t)(channel->rx_total_len - channel->rx_index);
        uint8_t chunk = (remaining > 7U) ? 7U : (uint8_t)remaining;

        memcpy(&channel->rx_buffer[channel->rx_index], &frame->data[1], chunk);
        channel->rx_index = (uint16_t)(channel->rx_index + chunk);
        channel->rx_timer_ms = 0U; /* Reset N_Cr timeout */

        if (channel->rx_index >= channel->rx_total_len) {
            channel->rx_state = ISOTP_RX_STATE_COMPLETE;
            return ISOTP_RESULT_OK;
        }

        if (channel->config.block_size > 0U) {
            channel->rx_block_counter++;
            if (channel->rx_block_counter >= channel->config.block_size) {
                channel->rx_block_counter = 0U;
                send_flow_control(channel, ISOTP_FS_CTS);
            }
        }
        return ISOTP_RESULT_OK;
    }

    case ISOTP_FRAME_FC: {
        if (channel->tx_state != ISOTP_TX_STATE_WAIT_FC) {
            return ISOTP_RESULT_UNEXPECTED_PDU;
        }

        uint8_t fs = (uint8_t)(frame->data[0] & 0x0FU);
        if (fs == ISOTP_FS_OVFLW) {
            channel->tx_state = ISOTP_TX_STATE_IDLE;
            channel->error_count++;
            return ISOTP_RESULT_BUFFER_OVERFLOW;
        } else if (fs == ISOTP_FS_WT) {
            channel->tx_timer_ms = 0U; /* Reset N_Bs timeout */
            return ISOTP_RESULT_OK;
        } else if (fs == ISOTP_FS_CTS) {
            channel->tx_client_bs = frame->data[1];
            channel->tx_client_stmin = frame->data[2];
            channel->tx_block_counter = 0U;
            channel->tx_timer_ms = 0U;
            channel->tx_stmin_timer_ms = 0U;
            channel->tx_state = ISOTP_TX_STATE_SEND_CF;
            return ISOTP_RESULT_OK;
        } else {
            channel->tx_state = ISOTP_TX_STATE_IDLE;
            channel->error_count++;
            return ISOTP_RESULT_INVALID_FRAME;
        }
    }

    default:
        channel->error_count++;
        return ISOTP_RESULT_INVALID_FRAME;
    }
}

bool isotp_get_received_message(isotp_channel_t *channel, uint8_t *data, uint16_t *len, uint16_t max_len)
{
    if (channel == NULL || data == NULL || len == NULL || channel->rx_state != ISOTP_RX_STATE_COMPLETE) {
        return false;
    }

    if (channel->rx_total_len > max_len) {
        channel->rx_state = ISOTP_RX_STATE_IDLE;
        return false;
    }

    memcpy(data, channel->rx_buffer, channel->rx_total_len);
    *len = channel->rx_total_len;

    channel->rx_state = ISOTP_RX_STATE_IDLE;
    channel->rx_total_len = 0U;
    channel->rx_index = 0U;
    return true;
}

isotp_result_t isotp_transmit(isotp_channel_t *channel, const uint8_t *data, uint16_t len)
{
    if (channel == NULL || data == NULL) {
        return ISOTP_RESULT_INVALID_FRAME;
    }

    if (channel->tx_state != ISOTP_TX_STATE_IDLE) {
        return ISOTP_RESULT_BUSY;
    }

    if (len == 0U || len > ISOTP_BUF_SIZE) {
        return ISOTP_RESULT_INVALID_FRAME;
    }

    memcpy(channel->tx_buffer, data, len);
    channel->tx_total_len = len;
    channel->tx_index = 0U;

    can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = channel->config.tx_id;
    frame.is_extended = false;
    frame.is_rtr = false;

    if (len <= 7U) {
        /* Single Frame */
        frame.dlc = channel->config.enable_padding ? 8U : (uint8_t)(len + 1U);
        frame.data[0] = (uint8_t)((ISOTP_FRAME_SF << 4) | (len & 0x0FU));
        memcpy(&frame.data[1], data, len);

        if (channel->config.enable_padding) {
            for (uint8_t i = (uint8_t)(len + 1U); i < 8U; i++) {
                frame.data[i] = channel->config.padding_val;
            }
        }

        if (can_driver_transmit(&frame)) {
            channel->tx_frames_count++;
            channel->tx_state = ISOTP_TX_STATE_IDLE;
            return ISOTP_RESULT_OK;
        } else {
            return ISOTP_RESULT_BUSY;
        }
    } else {
        /* Multi-frame First Frame */
        frame.dlc = 8U;
        frame.data[0] = (uint8_t)((ISOTP_FRAME_FF << 4) | ((len >> 8) & 0x0FU));
        frame.data[1] = (uint8_t)(len & 0xFFU);
        memcpy(&frame.data[2], data, 6U);

        if (can_driver_transmit(&frame)) {
            channel->tx_frames_count++;
            channel->tx_index = 6U;
            channel->tx_seq_num = 1U;
            channel->tx_timer_ms = 0U;
            channel->tx_state = ISOTP_TX_STATE_WAIT_FC;
            return ISOTP_RESULT_OK;
        } else {
            return ISOTP_RESULT_BUSY;
        }
    }
}

void isotp_process(isotp_channel_t *channel, uint32_t delta_ms)
{
    if (channel == NULL) {
        return;
    }

    /* RX N_Cr Timeout Monitor */
    if (channel->rx_state == ISOTP_RX_STATE_WAIT_CF) {
        channel->rx_timer_ms += delta_ms;
        if (channel->rx_timer_ms >= ISOTP_DEFAULT_N_CR_MS) {
            channel->rx_state = ISOTP_RX_STATE_IDLE;
            channel->error_count++;
        }
    }

    /* TX State Machine Processing */
    if (channel->tx_state == ISOTP_TX_STATE_WAIT_FC) {
        channel->tx_timer_ms += delta_ms;
        if (channel->tx_timer_ms >= ISOTP_DEFAULT_N_BS_MS) {
            channel->tx_state = ISOTP_TX_STATE_IDLE;
            channel->error_count++;
        }
    } else if (channel->tx_state == ISOTP_TX_STATE_SEND_CF) {
        channel->tx_stmin_timer_ms += delta_ms;
        if (channel->tx_stmin_timer_ms >= channel->tx_client_stmin) {
            /* Dispatch next Consecutive Frame */
            can_frame_t frame;
            memset(&frame, 0, sizeof(frame));
            frame.id = channel->config.tx_id;
            frame.is_extended = false;
            frame.is_rtr = false;

            frame.data[0] = (uint8_t)((ISOTP_FRAME_CF << 4) | (channel->tx_seq_num & 0x0FU));
            uint16_t remaining = (uint16_t)(channel->tx_total_len - channel->tx_index);
            uint8_t chunk = (remaining > 7U) ? 7U : (uint8_t)remaining;

            memcpy(&frame.data[1], &channel->tx_buffer[channel->tx_index], chunk);
            frame.dlc = channel->config.enable_padding ? 8U : (uint8_t)(chunk + 1U);

            if (channel->config.enable_padding) {
                for (uint8_t i = (uint8_t)(chunk + 1U); i < 8U; i++) {
                    frame.data[i] = channel->config.padding_val;
                }
            }

            if (can_driver_transmit(&frame)) {
                channel->tx_frames_count++;
                channel->tx_index = (uint16_t)(channel->tx_index + chunk);
                channel->tx_seq_num = (uint8_t)((channel->tx_seq_num + 1U) & 0x0FU);
                channel->tx_stmin_timer_ms = 0U;

                if (channel->tx_index >= channel->tx_total_len) {
                    channel->tx_state = ISOTP_TX_STATE_IDLE;
                } else if (channel->tx_client_bs > 0U) {
                    channel->tx_block_counter++;
                    if (channel->tx_block_counter >= channel->tx_client_bs) {
                        channel->tx_block_counter = 0U;
                        channel->tx_timer_ms = 0U;
                        channel->tx_state = ISOTP_TX_STATE_WAIT_FC;
                    }
                }
            }
        }
    }
}

bool isotp_is_tx_busy(const isotp_channel_t *channel)
{
    return (channel != NULL && channel->tx_state != ISOTP_TX_STATE_IDLE);
}

void isotp_reset(isotp_channel_t *channel)
{
    if (channel != NULL) {
        channel->rx_state = ISOTP_RX_STATE_IDLE;
        channel->tx_state = ISOTP_TX_STATE_IDLE;
        channel->rx_index = 0U;
        channel->tx_index = 0U;
        channel->rx_total_len = 0U;
        channel->tx_total_len = 0U;
    }
}
