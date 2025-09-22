#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <openppg/openppg_api.h>
#include <openppg/openppg_proto.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(openppg_driver, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * This file implements the application-side glue for OpenPPG control events.
 * It registers callbacks with the OpenPPG core (GATT) and can start/stop the
 * sensor data path accordingly. Sample publishing should be driven by the
 * actual sensor pipeline via openppg_publish_sample().
 */

static atomic_t streaming = ATOMIC_INIT(false);

#if defined(CONFIG_OPENPPG_TEST_GEN)
static struct k_work_delayable test_work;

static void test_tick(struct k_work *work)
{
    ARG_UNUSED(work);
    static uint16_t sequence;
    static uint8_t payload_seq;

    struct openppg_stream_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.schema_id = OPENPPG_SCHEMA_FRAME;
    frame.header.timestamp_ms = k_uptime_get_32();
    frame.header.sequence_number = sequence++;
    frame.header.num_channels = 1;
    frame.header.num_samples = 1;
    frame.payload_len = 1;
    frame.payload[0] = payload_seq++;

    (void)openppg_publish_sample(&frame);

    /* Default ~25 Hz for testing */
    k_work_schedule(&test_work, K_MSEC(40));
}
#endif

static void cb_on_stream_request(enum openppg_stream_rate rate, void *user_data)
{
    ARG_UNUSED(user_data);

    /* Start application capture at the requested rate. The concrete sensor
     * pipeline should react to this flag; this module only logs the request.
     */
    atomic_set(&streaming, true);
    LOG_INF("OpenPPG stream requested (rate=%u)", (unsigned)rate);
#if defined(CONFIG_OPENPPG_TEST_GEN)
    k_work_schedule(&test_work, K_NO_WAIT);
#endif
}

static void cb_on_stream_stopped(void *user_data)
{
    ARG_UNUSED(user_data);
    atomic_clear(&streaming);
    LOG_INF("OpenPPG stream stopped by remote");
#if defined(CONFIG_OPENPPG_TEST_GEN)
    k_work_cancel_delayable(&test_work);
#endif
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
#if defined(CONFIG_OPENPPG_TEST_GEN)
    k_work_init_delayable(&test_work, test_tick);
#endif
    return rc;
}

SYS_INIT(openppg_driver_init, APPLICATION, 50);
