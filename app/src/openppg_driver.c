#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <openppg/openppg_api.h>
#include <openppg/openppg_proto.h>
#include <zephyr/sys/atomic.h>

#include <inttypes.h>

LOG_MODULE_REGISTER(openppg_driver, LOG_LEVEL_INF);

/*
 * This file implements the application-side glue for OpenPPG control events.
 * It registers callbacks with the OpenPPG core (GATT) and can start/stop the
 * sensor data path accordingly. Sample publishing should be driven by the
 * actual sensor pipeline via openppg_publish_sample().
 */

static atomic_t streaming = ATOMIC_INIT(false);
static struct k_work_delayable profile_work;

#define PROFILE_LOG_INTERVAL K_SECONDS(5)

static void profile_tick(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!atomic_get(&streaming)) {
        return;
    }

    struct openppg_stream_profile profile;

    openppg_stream_profile_get(&profile);

    if (profile.drop_enomem || profile.drop_eagain || profile.drop_other) {
        LOG_WRN("OpenPPG stream drops: eagain=%" PRIu64 ", enomem=%" PRIu64 ", other=%" PRIu64
            ", last_err=%d",
            profile.drop_eagain, profile.drop_enomem, profile.drop_other,
            profile.last_transport_err);
    }

    LOG_INF("OpenPPG stream stats: enq=%" PRIu64 ", deq=%" PRIu64 ", notif=%" PRIu64
        ", bytes=%" PRIu64 ", depth=%" PRIu32 "/%" PRIu32 ", max_payload=%" PRIu32,
        profile.enqueued_frames, profile.dequeued_frames, profile.notified_frames,
        profile.bytes_notified, profile.last_queue_depth, profile.max_queue_depth,
        profile.max_payload_len);

    (void)k_work_reschedule(&profile_work, PROFILE_LOG_INTERVAL);
}

static void cb_on_stream_request(enum openppg_stream_rate rate, void *user_data)
{
    ARG_UNUSED(user_data);

    /* Start application capture at the requested rate. The concrete sensor
     * pipeline should react to this flag; this module only logs the request.
     */
    atomic_set(&streaming, true);
    LOG_INF("OpenPPG stream requested (rate=%u)", (unsigned)rate);
    openppg_stream_profile_reset();
    (void)k_work_reschedule(&profile_work, K_NO_WAIT);
}

static void cb_on_stream_stopped(void *user_data)
{
    ARG_UNUSED(user_data);
    atomic_clear(&streaming);
    LOG_INF("OpenPPG stream stopped by remote");
    k_work_cancel_delayable(&profile_work);
}

static void cb_on_control_command(const struct openppg_control_command *cmd, void *user_data)
{
    ARG_UNUSED(user_data);
    LOG_INF("OpenPPG control opcode=%u param=%u", (unsigned)cmd->opcode, (unsigned)cmd->param);
}

static int openppg_driver_init(const struct device *unused)
{
    ARG_UNUSED(unused);

    static const struct openppg_callbacks cbs = {
        .on_stream_request = cb_on_stream_request,
        .on_stream_stopped = cb_on_stream_stopped,
        .on_control_command = cb_on_control_command,
    };

    int rc = openppg_init(&cbs, NULL);
    k_work_init_delayable(&profile_work, profile_tick);
    return rc;
}

SYS_INIT(openppg_driver_init, APPLICATION, 50);
