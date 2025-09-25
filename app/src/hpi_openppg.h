// SPDX-License-Identifier: Apache-2.0
#ifndef HPI_OPENPPG_H_
#define HPI_OPENPPG_H_

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Public configuration knobs ----

// Max channels we’ll expose over OpenPPG from HPI data paths (adjust as needed)
#ifndef HPI_OPPG_MAX_CH
#define HPI_OPPG_MAX_CH  3   // GREEN, IR, RED
#endif

// Default: how many time-samples per OpenPPG frame (kept small to fit one MTU)
#ifndef HPI_OPPG_NSAMPLES_PER_FRAME
#define HPI_OPPG_NSAMPLES_PER_FRAME  10
#endif

// Row queue depth (producer: data_thread; consumer: openppg_core)
#ifndef HPI_OPPG_ROW_QUEUE_LEN
#define HPI_OPPG_ROW_QUEUE_LEN  256
#endif

// ---- Types from your HPI code ----
struct hpi_ppg_wr_data_t;
struct hpi_ppg_fi_data_t;
struct hpi_ecg_bioz_sensor_data_t;

// ---- Initialization & configuration ----

// Call once at boot (before openppg_init()).
int hpi_openppg_init(void);

// Configure channel mapping / qfmt for OpenPPG stream.
// For v1: wrist PPG (ch0=GREEN or RED, ch1=IR, ch2=remaining color when available).
int hpi_openppg_configure_ppg(uint32_t rate_hz, bool use_green_for_ch0);

// ---- Batch push helpers (call these from data_thread) ----

// Push a wrist PPG batch (GREEN/IR) into the OpenPPG row queue.
void hpi_openppg_push_ppg_wrist(const struct hpi_ppg_wr_data_t *b);

// Push a finger/IR PPG batch into the OpenPPG row queue.
void hpi_openppg_push_ppg_fi(const struct hpi_ppg_fi_data_t *b);

// (Optional) Push ECG/BioZ batches if you intend to stream them in OpenPPG later.
void hpi_openppg_push_ecg_bioz(const struct hpi_ecg_bioz_sensor_data_t *b);

#ifdef __cplusplus
}
#endif

#endif // HPI_OPENPPG_H_
