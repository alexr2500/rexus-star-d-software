#include <Arduino.h>
extern "C" {
  #include "driver/spi_slave.h"
}

#define PIN_MISO 19   // D19
#define PIN_MOSI 23   // D23
#define PIN_SCLK 18   // D18
#define PIN_CS   5    // D5

static uint8_t rx = 0x00;
static uint8_t tx = 0x55;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\nBooting SPI slave...");

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = PIN_MOSI;
  buscfg.miso_io_num = PIN_MISO;
  buscfg.sclk_io_num = PIN_SCLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 4;

  spi_slave_interface_config_t slvcfg = {};
  slvcfg.mode = 0;               // SPI mode 0
  slvcfg.spics_io_num = PIN_CS;  // CS pin
  slvcfg.queue_size = 3;
  slvcfg.flags = 0;

  // IMPORTANT: we use VSPI_HOST with pins 18/19/23/5
  esp_err_t ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

  Serial.print("spi_slave_initialize ret = ");
  Serial.println((int)ret);
  if (ret != ESP_OK) {
    Serial.println("SPI init FAILED. Check pins / core ESP32.");
    while (true) delay(1000);
  }

  Serial.println("SPI Slave ready (waiting for master)");
}

void loop() {
  rx = 0x00;
  tx = 0x55;

  spi_slave_transaction_t t = {};
  t.length = 8;           // 8 bits
  t.rx_buffer = &rx;
  t.tx_buffer = &tx;

  // Wait for a master transaction
  esp_err_t r = spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);

  if (r == ESP_OK) {
    Serial.print("RX=0x");
    Serial.print(rx, HEX);
    Serial.print("  TX=0x");
    Serial.println(tx, HEX);
  } else {
    Serial.print("Transmit error: ");
    Serial.println((int)r);
  }
}
