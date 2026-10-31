#include "telemetry.h"
#include "crc16.h"
#include <string.h>
#include "error_manager.h"

static TelemetryPacket packet;          //encapsulation: telemetry packet is only handled by this file
static uint16_t sequence_count = 0;     //sequence count is only set by this file, and is incremented every time a packet is sent. No number errors from external modules

void telemetry_init(void) {
    sequence_count = 0;
    //TODO: initialize telem?
}

static void telemetry_update() {
    packet.sync1 = TLM_SYNC1;        //CCSDS_Formatting
    packet.sync2 = TLM_SYNC2;
    packet.msg_id = TLM_MSG_ID_TELEMETRY;
    packet.seq_count = sequence_count++;
    packet.payload_length = TLM_PAYLOAD_LENGTH;

    packet.timestamp_ms = 
    packet.mode = 
    
    uint8_t* data = (uint8_t*)&packet;

    packet.crc = checksum16_ccitt(data, len);  //crc calculation
}

static void telemetry_send(void) {
    //TODO: see how to send packets through UART
}

void telemetry_task(void) {
    telemetry_update();
    telemetry_send();
    vTaskDelay(pdMS_TO_TICKS(100));
}

uint16_t checksum16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void telemetry_set_mode(software_mode_t mode)
{
    packet.mode = (uint8_t)mode;
}

void telemetry_set_camera_status(camera_status_t status)
{
    packet.camera_status = (uint8_t)status;
}

void telemetry_set_ssd_free_gb(uint8_t free_gb)
{
    packet.ssd_free_gb = free_gb;
}