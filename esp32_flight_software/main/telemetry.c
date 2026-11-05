#include "telemetry.h"
#include "protocol.h"
#include "sensors.h"
#include "error_manager.h"
#include "mode_manager.h"
#include "crc16.h"
#include "uart_tcu.h"
#include "esp_timer.h"
#include <string.h>

/* Builds one packet per call and hands it to the RS-422 link.
 *
 * The packet is assembled here and nowhere else: no other module sees a
 * TelemetryPacket, which is why the three setters below exist. The
 * fields that come from sensors are PULLED from a snapshot rather than
 * pushed in one at a time, so every packet is internally consistent --
 * a packet can never mix temperatures from one cycle with pressures
 * from the next.
 *
 * There is no CRC implementation in this file. crc16_ccitt() in
 * components/links is the single copy; a second one here could drift
 * from it and from the Pi, and then every packet would fail its check
 * for a reason nobody could find. */

static TelemetryPacket packet;
static uint16_t  seq;
static int64_t   last_sent_us;

/* Cached values pushed in by the modules that own their meaning. */
static uint8_t cached_camera_status = CAM_UNKNOWN;
static uint8_t cached_ssd_free_gb   = SSD_VALUE_UNKNOWN;
static uint8_t cached_stepper_activation;

_Static_assert(sizeof(TelemetryPacket) == 32,
               "TelemetryPacket changed size: the ground-station unpack "
               "format string in the ICD must change with it");

void telemetry_init(void)
{
    memset(&packet, 0, sizeof packet);

    /* The constants are written once, here, rather than on every
       transmission. Only the varying fields are touched in the task. */
    packet.sync1          = TLM_SYNC1;
    packet.sync2          = TLM_SYNC2;
    packet.msg_id         = TLM_MSG_ID_TELEMETRY;
    packet.payload_length = TLM_PAYLOAD_LENGTH;

    seq          = 0;
    last_sent_us = INT64_MIN;
}

void telemetry_set_mode(software_mode_t mode)
{
    packet.mode = (uint8_t)mode;
}

void telemetry_set_camera_status(camera_status_t status)
{
    cached_camera_status = (uint8_t)status;
}

void telemetry_set_ssd_free_gb(uint8_t free_gb)
{
    cached_ssd_free_gb = free_gb;
}

void telemetry_set_stepper_activation(bool active)
{
    cached_stepper_activation = active ? 1u : 0u;
}

void telemetry_task(void)
{
    const int64_t now = esp_timer_get_time();

    /* Self-rate-limiting: the caller may run this from the control loop
       at whatever rate it likes, and the downlink still goes out at
       1 Hz. Putting the limit here rather than in the caller means the
       rate cannot drift if the loop period changes.
       There is deliberately no vTaskDelay here -- this function must
       never block the control loop. */
    if (last_sent_us != INT64_MIN
        && (now - last_sent_us) < (int64_t)POLL_PERIOD_MS * 1000LL) {
        return;
    }
    last_sent_us = now;

    sensor_snapshot_t s;
    sensors_get(&s);

    packet.seq_count    = seq++;
    packet.timestamp_ms = s.timestamp_ms;
    packet.mode         = (uint8_t)software_mode_get();

    packet.pt100_temp   = s.pt100_temp;
    packet.int_bme_temp = s.int_bme_temp;
    packet.ext_bme_temp = s.ext_bme_temp;
    packet.slf3s_flow   = s.slf3s_flow;
    packet.abp_pressure = s.abp_pressure;
    packet.voltage_mV   = s.voltage_mV;

    packet.stepper_motor_activation = cached_stepper_activation;
    packet.ssd_free_gb              = cached_ssd_free_gb;
    packet.camera_status            = cached_camera_status;
    packet.status_error_flag        = error_flags_get();

    /* CRC last, over everything except itself. TLM_CRC_COVERAGE_LEN is
       derived with sizeof, so adding a field cannot leave it stale. */
    packet.crc = crc16_ccitt((const uint8_t *)&packet, TLM_CRC_COVERAGE_LEN);

    (void)uart_tcu_send(&packet, sizeof packet);
}