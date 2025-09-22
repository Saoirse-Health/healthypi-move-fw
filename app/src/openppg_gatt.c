// SPDX-License-Identifier: Apache-2.0
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>

#include <openppg/openppg_api.h>
#include <openppg/openppg_proto.h>
#include <openppg/openppg_uuid.h>

LOG_MODULE_REGISTER(openppg_gatt, CONFIG_LOG_DEFAULT_LEVEL);

#ifndef CONFIG_OPENPPG_MAX_FRAME_BYTES
#define CONFIG_OPENPPG_MAX_FRAME_BYTES 128
#endif

static bool ccc_enabled;
static uint8_t frame_buf[CONFIG_OPENPPG_MAX_FRAME_BYTES];

/* Attribute table */
BT_GATT_SERVICE_DEFINE(openppg_svc,
    BT_GATT_PRIMARY_SERVICE(OPENPPG_UUID_SERVICE_OPENPPG_STREAM),

    /* Control characteristic: write-only */
    BT_GATT_CHARACTERISTIC(OPENPPG_UUID_CHAR_OPPG_CONTROL,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, control_write, NULL),

    /* Frame characteristic: notify-only */
    BT_GATT_CHARACTERISTIC(OPENPPG_UUID_CHAR_OPPG_SAMPLES,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);


/* Registered application callbacks */
static const struct openppg_callbacks *g_cbs;
static void *g_user_data;

/* Local stream state (app-visible control via API) */
static atomic_t g_streaming = ATOMIC_INIT(false);

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ccc_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("OpenPPG notify %s", ccc_enabled ? "ENABLED" : "disabled");
    if (!ccc_enabled) {
        /* Notify application that streaming should stop due to CCC off */
        if (g_cbs && g_cbs->on_stream_stopped) {
            g_cbs->on_stream_stopped(g_user_data);
        }
        atomic_clear(&g_streaming);
    }
}

/* Control write handler: expects struct openppg_control_command (opcode+param) */
static ssize_t control_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset != 0 || len < sizeof(struct openppg_control_command)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    const struct openppg_control_command *cmd = buf;
    /* First, forward the raw command to the application if requested */
    if (g_cbs && g_cbs->on_control_command) {
        g_cbs->on_control_command(cmd, g_user_data);
    }

    /* Then, drive the higher-level stream callbacks */
    switch (cmd->opcode) {
    case OPENPPG_CONTROL_OPCODE_START_STREAM:
        if (!ccc_enabled) {
            return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
        }
        if (g_cbs && g_cbs->on_stream_request) {
            g_cbs->on_stream_request((enum openppg_stream_rate)cmd->param, g_user_data);
        }
        atomic_set(&g_streaming, true);
        break;
    case OPENPPG_CONTROL_OPCODE_STOP_STREAM:
        if (g_cbs && g_cbs->on_stream_stopped) {
            g_cbs->on_stream_stopped(g_user_data);
        }
        atomic_clear(&g_streaming);
        break;
    case OPENPPG_CONTROL_OPCODE_SET_RATE:
        /* Leave policy to application via on_control_command */
        break;
    default:
        LOG_WRN("Unknown opcode: %u", cmd->opcode);
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }
    return len;
}

/* API: Initialize OpenPPG core with callbacks */
int openppg_init(const struct openppg_callbacks *cbs, void *user_data)
{
    g_cbs = cbs;
    g_user_data = user_data;
    atomic_clear(&g_streaming);
    ccc_enabled = false;
    return 0;
}

/* API: Start/stop streaming from application side */
int openppg_stream_start(enum openppg_stream_rate rate)
{
    ARG_UNUSED(rate);
    atomic_set(&g_streaming, true);
    return 0;
}

int openppg_stream_stop(void)
{
    atomic_clear(&g_streaming);
    return 0;
}

/* API: Publish a sample frame to the Frame characteristic */
int openppg_publish_sample(const struct openppg_stream_frame *frame)
{
    if (!ccc_enabled) {
        return -EACCES;
    }

    /* Conservatively send up to configured MTU-sized frame buffer */
    size_t nbytes = MIN(sizeof(*frame), sizeof(frame_buf));
    memcpy(frame_buf, frame, nbytes);

    /* Frame characteristic value attribute index: 0:svc,1:ctrl decl,2:ctrl val,3:frame decl,4:frame val,5:ccc */
    return bt_gatt_notify(NULL, &openppg_svc.attrs[4], frame_buf, nbytes);
}

/* API: Optionally publish status updates (not supported if no status char) */
int openppg_publish_status(const struct openppg_status_update *status)
{
    ARG_UNUSED(status);
    /* No dedicated status characteristic declared in this service definition */
    return -ENOTSUP;
}
