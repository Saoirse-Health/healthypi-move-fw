// SPDX-License-Identifier: Apache-2.0

#include <errno.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <openppg/openppg_api.h>
#include <openppg/openppg_codec.h>
#include <openppg/openppg_proto.h>
#include <openppg/openppg_uuid.h>

/* Internal profiling hook from the OpenPPG core. */
void openppg_core_profile_on_transport_result(int err, size_t payload_len);

LOG_MODULE_DECLARE(hpi_openppg, LOG_LEVEL_INF);

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

/* Size the pool to tolerate a few outstanding notifications at once. */
#define OPPG_TX_POOL_COUNT 8
#define OPPG_TX_THREAD_STACK_SIZE 2048
#define OPPG_TX_THREAD_PRIO K_PRIO_COOP(8)

struct oppg_tx_buf {
    void *fifo_reserved; /* Required by k_fifo */
    struct bt_gatt_notify_params params;
    size_t payload_len;
    uint8_t payload[STREAM_ENCODE_MAX];
};

K_MEM_SLAB_DEFINE(oppg_tx_slab, sizeof(struct oppg_tx_buf), OPPG_TX_POOL_COUNT, 4);
K_FIFO_DEFINE(oppg_tx_fifo);

static void oppg_notify_complete(struct bt_conn *conn, void *user_data)
{
    ARG_UNUSED(conn);

    struct oppg_tx_buf *buf = user_data;
    if (!buf) {
        return;
    }

    openppg_core_profile_on_transport_result(0, buf->payload_len);
    k_mem_slab_free(&oppg_tx_slab, buf);
}

static void oppg_tx_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        struct oppg_tx_buf *buf = k_fifo_get(&oppg_tx_fifo, K_FOREVER);
        if (!buf) {
            continue;
        }

        int err = bt_gatt_notify_cb(NULL, &buf->params);
        if (err == -ENOTCONN) {
            openppg_core_profile_on_transport_result(0, buf->payload_len);
            k_mem_slab_free(&oppg_tx_slab, buf);
            continue;
        }

        if (err) {
            LOG_WRN("OpenPPG notify failed (%d)", err);
            openppg_core_profile_on_transport_result(err, 0U);
            k_mem_slab_free(&oppg_tx_slab, buf);
        }
    }
}

K_THREAD_DEFINE(oppg_tx_thread_id, OPPG_TX_THREAD_STACK_SIZE, oppg_tx_thread,
                NULL, NULL, NULL, OPPG_TX_THREAD_PRIO, 0, 0);

int openppg_transport_notify_stream(const struct openppg_stream_frame *frame)
{
    if (!frame) {
        return -EINVAL;
    }

    struct oppg_tx_buf *buf;
    if (k_mem_slab_alloc(&oppg_tx_slab, (void **)&buf, K_NO_WAIT) != 0) {
        LOG_WRN("OpenPPG TX pool exhausted; dropping frame");
        openppg_core_profile_on_transport_result(-ENOMEM, 0U);
        return -ENOMEM;
    }

    size_t encoded_len = 0U;
    int err = openppg_codec_encode_stream_frame(frame, buf->payload,
                                                 sizeof(buf->payload), &encoded_len);
    if (err) {
        LOG_ERR("Failed to encode stream frame (%d)", err);
        openppg_core_profile_on_transport_result(err, 0U);
        k_mem_slab_free(&oppg_tx_slab, buf);
        return err;
    }

    buf->payload_len = encoded_len;
    buf->params = (struct bt_gatt_notify_params){
        .uuid = OPENPPG_UUID_CHAR_OPPG_SAMPLES,
        .attr = &openppg_service.attrs[OPENPPG_ATTR_IDX_STREAM_VALUE],
        .data = buf->payload,
        .len = (uint16_t)encoded_len,
        .func = oppg_notify_complete,
        .user_data = buf,
    };

    k_fifo_put(&oppg_tx_fifo, buf);

    return 0;
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
