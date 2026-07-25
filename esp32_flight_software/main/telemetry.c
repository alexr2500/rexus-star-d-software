#include "telemetry.h"
#include <string.h>
#include "error_manager.h"

static TelemetryPacket packet;          //encapsulation: telemetry packet is only handled by this file
static uint16_t sequence_count = 0;

void telemetry_init(void) {
    sequence_count = 0;
    //TODO: initialize telem?
}

static void telemetry_update() {
    packet.header.packet_id = 0x01;         //CCSDS_Formatting
    packet.header.sequence_order++;
    packet.header.packet_content_length = sizeof(TelemetryPacket) - sizeof(CCSDS_Formatting);
    packet.packet_size = sizeof(TelemetryPacket);

    packet.voltage = read_voltage();        //packet contents / read data
    packet.board_temp = read_board_temp();
    packet.SSD_status = read_SSD_status();
    packet.camera_status = read_camera_status();
    packet.pv_temp = read_pv_temp();
    packet.general_temp = read_general_temp();
    packet.timestamp = get_timestamp_ms();
    strcpy(packet.message, "");                 //TODO: see how to link this to a given status/mode/error/message
    packet.error_flag = get_error_flags();     //returns the error flag so that we know what errors have occured if any
    size_t len = sizeof(TelemetryPacket);
    
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