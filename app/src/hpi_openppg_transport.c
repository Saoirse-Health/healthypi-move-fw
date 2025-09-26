// SPDX-License-Identifier: Apache-2.0

#include <errno.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include <openppg/openppg_api.h>
#include <openppg/openppg_codec.h>
#include <openppg/openppg_proto.h>
#include <openppg/openppg_uuid.h>

/* Internal profiling hook from the OpenPPG core. */
void openppg_core_profile_on_transport_result(int err, size_t payload_len);

LOG_MODULE_DECLARE(hpi_openppg, CONFIG_LOG_DEFAULT_LEVEL);

/* Attribute indices mirrored from the OpenPPG GATT definition. */
enum openppg_attr_index {
    OPENPPG_ATTR_IDX_PRIMARY,
    OPENPPG_ATTR_IDX_META_DECL,
    OPENPPG_ATTR_IDX_META_VALUE,
    OPENPPG_ATTR_IDX_CONTROL_DECL,
    OPENPPG_ATTR_IDX_CONTROL_VALUE,
    OPENPPG_ATTR_IDX_STREAM_DECL,
    OPENPPG_ATTR_IDX_STREAM_VALUE,
    OPENPPG_ATTR_IDX_STREAM_CCC,
    OPENPPG_ATTR_IDX_FEATURES_DECL,
    OPENPPG_ATTR_IDX_FEATURES_VALUE,
    OPENPPG_ATTR_IDX_FEATURES_CCC,
};

extern const struct bt_gatt_service_static openppg_service;

#define STREAM_ENCODE_MAX (OPENPPG_FRAME_MAX_SAMPLE_BYTES + \
    OPENPPG_FRAME_MAX_CHANNELS * 48 + \
    OPENPPG_FRAME_MAX_FEATURES * (OPENPPG_FRAME_MAX_FEATURE_VALUE_LEN + 32) + 256)

static uint8_t stream_scratch[STREAM_ENCODE_MAX];
int openppg_transport_notify_stream(const struct openppg_stream_frame *frame)
{
    if (!frame) {
        return -EINVAL;
    }

    size_t encoded_len = 0U;
    int err = openppg_codec_encode_stream_frame(frame, stream_scratch,
                                                 sizeof(stream_scratch), &encoded_len);
    if (err) {
        LOG_ERR("Failed to encode stream frame (%d)", err);
        openppg_core_profile_on_transport_result(err, 0U);
        return err;
    }

    LOG_INF("OpenPPG stream payload %zu bytes", encoded_len);

    const struct bt_gatt_attr *attr = &openppg_service.attrs[OPENPPG_ATTR_IDX_STREAM_VALUE];

    err = bt_gatt_notify_uuid(NULL, OPENPPG_UUID_CHAR_OPPG_SAMPLES, attr,
                              stream_scratch, encoded_len);
    if (err == -ENOTCONN) {
        err = 0;
    }

    openppg_core_profile_on_transport_result(err, err ? 0U : encoded_len);

    return err;
}

int openppg_transport_notify_status(const struct openppg_status_update *status)
{
    if (!status) {
        return -EINVAL;
    }

    uint8_t encoded[64];
    size_t encoded_len = 0U;
    int err = openppg_codec_encode_status(status, encoded, sizeof(encoded), &encoded_len);
    if (err) {
        LOG_ERR("Failed to encode status (%d)", err);
        openppg_core_profile_on_transport_result(err, 0U);
        return err;
    }

    LOG_DBG("OpenPPG status payload %zu bytes", encoded_len);

    const struct bt_gatt_attr *attr = &openppg_service.attrs[OPENPPG_ATTR_IDX_FEATURES_VALUE];
    err = bt_gatt_notify_uuid(NULL, OPENPPG_UUID_CHAR_OPPG_FEATURES, attr, encoded, encoded_len);
    if (err == -ENOTCONN) {
        err = 0;
    }

    openppg_core_profile_on_transport_result(err, err ? 0U : encoded_len);

    return err;
}
