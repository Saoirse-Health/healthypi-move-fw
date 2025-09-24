// SPDX-License-Identifier: Apache-2.0

#include "hpi_openppg.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
LOG_MODULE_REGISTER(hpi_openppg, CONFIG_LOG_DEFAULT_LEVEL);

// OpenPPG core & proto (already in your tree)
#include <openppg/openppg_proto.h>
#include "openppg_internal.h"  // for openppg_gatt_notify_stream() via core
#include <openppg/openppg_api.h>  // publishes frames/status

#include "hpi_common_types.h"

// ---- Internal state ----

enum {
    OPPG_MODALITY_RED = 0,
    OPPG_MODALITY_IR = 1,
    OPPG_MODALITY_GREEN = 2,
};

enum {
    OPPG_QFMT_U20_RSHIFT4 = 0,
    OPPG_QFMT_S16 = 1,
    OPPG_QFMT_U24 = 2,
};

struct oppg_row {
    uint64_t timestamp_ms;
    int32_t  sample[HPI_OPPG_MAX_CH];
};

static K_MSGQ_DEFINE(s_row_q, sizeof(struct oppg_row), HPI_OPPG_ROW_QUEUE_LEN, 4);

static struct {
    bool     configured;
    bool     running;
    uint32_t rate_hz;
    uint8_t  n_channels;          // <= HPI_OPPG_MAX_CH
    uint8_t  qfmt[HPI_OPPG_MAX_CH];  // openppg qfmt enums
    bool     ch0_green_not_red;   // true: map GREEN->ch0, false: RED->ch0
    uint16_t n_samples_per_frame; // frame batching
    uint16_t frame_seq;
    uint8_t  map_seq;
} s_cfg = {
    .configured = false,
    .running = false,
    .rate_hz = 100,
    .n_channels = 2,
    .qfmt = { OPPG_QFMT_U24, OPPG_QFMT_U24 }, // wire will be quantized later by core/codec
    .ch0_green_not_red = true,
    .n_samples_per_frame = HPI_OPPG_NSAMPLES_PER_FRAME,
    .frame_seq = 0,
    .map_seq = 0,
};

// ---- Helpers ----

static inline void push_row_i2(int32_t ch0, int32_t ch1)
{
    if (!s_cfg.running) return;

    struct oppg_row r = {
        .timestamp_ms = (uint64_t)k_uptime_get(), // TODO: swap for true wall-clock epoch when available
        .sample = { ch0, ch1 }
    };
    if (k_msgq_put(&s_row_q, &r, K_NO_WAIT) != 0) {
        // Drop-oldest policy
        struct oppg_row throwaway;
        (void)k_msgq_get(&s_row_q, &throwaway, K_NO_WAIT);
        (void)k_msgq_put(&s_row_q, &r, K_NO_WAIT);
        // TODO: expose a drop counter via metrics/status
    }
}

// ---- Public API ----

int hpi_openppg_init(void)
{
    // msgq already defined statically; nothing else for now
    return 0;
}

int hpi_openppg_configure_ppg(uint32_t rate_hz, bool use_green_for_ch0)
{
    bool was_configured = s_cfg.configured;
    bool prior_green = s_cfg.ch0_green_not_red;

    s_cfg.rate_hz = (rate_hz == 0) ? 100 : rate_hz;
    s_cfg.ch0_green_not_red = use_green_for_ch0;
    s_cfg.n_channels = MIN((uint8_t)2U, (uint8_t)OPENPPG_FRAME_MAX_CHANNELS);
    s_cfg.qfmt[0] = OPPG_QFMT_U24;
    s_cfg.qfmt[1] = OPPG_QFMT_U24;
    s_cfg.configured = true;

    if (!was_configured || prior_green != use_green_for_ch0) {
        s_cfg.map_seq++;
    }

    return 0;
}

void hpi_openppg_push_ppg_wrist(const struct hpi_ppg_wr_data_t *b)
{
    if (!b || !s_cfg.configured) return;

    for (uint8_t i = 0; i < b->ppg_num_samples; ++i) {
        int32_t ch0 = 0;
        if (s_cfg.ch0_green_not_red) {
            ch0 = (int32_t)b->raw_green[i];
        } else {
            // If RED data becomes available, swap assignment here.
            ch0 = (int32_t)b->raw_green[i];
        }
        int32_t ch1 = (int32_t)b->raw_ir[i];
        push_row_i2(ch0, ch1);
    }
}

void hpi_openppg_push_ppg_fi(const struct hpi_ppg_fi_data_t *b)
{
    if (!b || !s_cfg.configured) return;

    for (uint8_t i = 0; i < b->ppg_num_samples; ++i) {
        push_row_i2(/*ch0=*/0, /*ch1=*/(int32_t)b->raw_ir[i]);
    }
}

void hpi_openppg_push_ecg_bioz(const struct hpi_ecg_bioz_sensor_data_t *b)
{
    // Not wired for v1: we’re streaming PPG. You can extend mapping here later.
    ARG_UNUSED(b);
}

// ---- OpenPPG core HW hooks (these satisfy externs in openppg_core.c) ----

// NOTE: Your current core polls for full frames via openppg_hw_sample(). We assemble one
// by draining s_row_q up to n_samples_per_frame rows into the schema-aligned
// openppg_stream_frame, populating header metadata, channel_map, and interleaved samples.

extern int openppg_gatt_notify_stream(const struct openppg_stream_frame *frame); // used by core

// Quantize raw int32 to chosen qfmt range (very basic; refine as needed)
static inline int32_t clip_s16(int32_t v) { return (v < -32768) ? -32768 : (v > 32767 ? 32767 : v); }
static inline uint32_t to_u24(int32_t v)  { if (v < 0) v = 0; if (v > 0xFFFFFF) v = 0xFFFFFF; return (uint32_t)v; }

int openppg_hw_sample(struct openppg_stream_frame *frame)
{
    if (!s_cfg.running || !s_cfg.configured || !frame) return -EAGAIN;

    memset(frame, 0, sizeof(*frame));
    frame->schema_id = OPENPPG_SCHEMA_FRAME;

    // Compute width of one interleaved sample row based on channel formats.
    size_t row_bytes = 0;
    for (uint8_t c = 0; c < s_cfg.n_channels; ++c) {
        switch (s_cfg.qfmt[c]) {
        case OPPG_QFMT_S16:
            row_bytes += 2U;
            break;
        case OPPG_QFMT_U24:
        case OPPG_QFMT_U20_RSHIFT4:
            row_bytes += 3U;
            break;
        default:
            row_bytes += 3U;
            break;
        }
    }

    if (row_bytes == 0U) {
        return -EINVAL;
    }

    const uint16_t target_rows = MIN(s_cfg.n_samples_per_frame, (uint16_t)OPENPPG_FRAME_MAX_SAMPLES);
    const uint16_t max_rows_by_bytes = MIN(target_rows, (uint16_t)(OPENPPG_FRAME_MAX_SAMPLE_BYTES / row_bytes));

    if (max_rows_by_bytes == 0U) {
        return -EINVAL;
    }

    frame->header.num_channels = s_cfg.n_channels;
    frame->channel_count = s_cfg.n_channels;

    for (uint8_t c = 0; c < s_cfg.n_channels; ++c) {
        struct openppg_channel_desc *desc = &frame->channel_map[c];
        desc->id = c;
        desc->qfmt = s_cfg.qfmt[c];
        switch (s_cfg.qfmt[c]) {
        case OPPG_QFMT_S16:
            desc->adc_bits = 16U;
            break;
        case OPPG_QFMT_U20_RSHIFT4:
            desc->adc_bits = 20U;
            break;
        case OPPG_QFMT_U24:
        default:
            desc->adc_bits = 24U;
            break;
        }
        desc->wavelength_nm = 0U; // TODO: populate with sensor-specific wavelength when available

        if (c == 0U) {
            desc->modality = s_cfg.ch0_green_not_red ? OPPG_MODALITY_GREEN : OPPG_MODALITY_RED;
        } else {
            desc->modality = OPPG_MODALITY_IR;
        }
    }

    uint16_t got = 0;
    uint64_t first_ts = 0;
    uint8_t *p = frame->samples;

    while (got < max_rows_by_bytes) {
        struct oppg_row r;
        if (k_msgq_get(&s_row_q, &r, K_NO_WAIT) != 0) {
            break;
        }

        if (got == 0U) {
            first_ts = r.timestamp_ms;
        }

        for (uint8_t c = 0; c < s_cfg.n_channels; ++c) {
            int32_t v = r.sample[c];

            switch (s_cfg.qfmt[c]) {
            case OPPG_QFMT_S16: {
                int16_t s = (int16_t)clip_s16(v);
                p[0] = (uint8_t)(s & 0xFF);
                p[1] = (uint8_t)((s >> 8) & 0xFF);
                p += 2;
            } break;
            case OPPG_QFMT_U20_RSHIFT4: {
                uint32_t u = (uint32_t)((v < 0) ? 0 : v) & 0x000FFFFF;
                p[0] = (uint8_t)(u & 0xFF);
                p[1] = (uint8_t)((u >> 8) & 0xFF);
                p[2] = (uint8_t)((u >> 16) & 0x0F);
                p += 3;
            } break;
            case OPPG_QFMT_U24:
            default: {
                uint32_t u = to_u24(v);
                p[0] = (uint8_t)(u & 0xFF);
                p[1] = (uint8_t)((u >> 8) & 0xFF);
                p[2] = (uint8_t)((u >> 16) & 0xFF);
                p += 3;
            } break;
            }
        }

        got++;
    }

    if (got == 0U) {
        return -EAGAIN;
    }

    frame->header.timestamp_ms = first_ts;
    frame->header.sequence_number = s_cfg.frame_seq++;
    frame->header.num_samples = got;
    frame->header.has_sample_rate_hz = true;
    frame->header.sample_rate_hz = s_cfg.rate_hz;
    frame->header.has_map_seq = true;
    frame->header.map_seq = s_cfg.map_seq;

    frame->samples_len = got * row_bytes;
    frame->feature_count = 0U;

    return 0;
}

void openppg_hw_on_start(enum openppg_stream_rate rate)
{
    ARG_UNUSED(rate);
    s_cfg.running = true;
}

void openppg_hw_on_stop(void)
{
    s_cfg.running = false;
    k_msgq_purge(&s_row_q);
}

int openppg_hw_fetch_status(struct openppg_status_update *status)
{
    if (!status) return -EINVAL;
    status->status = OPENPPG_STATUS_OK;
    status->warning_flags = 0;
    memset(status->reserved, 0, sizeof(status->reserved));
    return 0;
}
