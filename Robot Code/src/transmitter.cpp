#include <Arduino.h>
#include <SPI.h>

#include "dw3000.h"

// dwm3000 definitions
#define PIN_IRQ  6
#define PIN_RST  7
#define PIN_WAKE 8

#define DWM_ID 0xDECA0302

#define RECEIVER_ADDRESS    0x00000001
#define TRANSMITTER_ADDRESS 0x00000002

// spi definitions
#define SCK  2
#define MISO 0
#define MOSI 3
#define CS   1
#define SPI_CLK_SPEED 2000000 // 2MHz
// DWM3000 Recieves Data MSB First

#define READ_DUMMY 0x00

// joystick definitions
#define JOY_Y  29
#define JOY_X  28
#define JOY_SW 27 // give this an internal pull-up resistor

// auto mode toggle
#define AUTO 13 // give this an internal pull-down resistor

void printDWMDiagnostics();
void resetDWM();
void send(uint32_t dest_address, uint8_t data[], int data_size);
void initiate(uint8_t command);
void checkData();
void respond(uint32_t dest_address);

int max_distance;

bool initiated = false;
volatile bool uwb_irq = false;
uint16_t rx_len = 0;   
uint32_t rx_finfo;
uint8_t rx_data[64];

uint64_t t1; // timestamp when transmitter sent frame
uint64_t t4; // timestamp when transmitter recieved response
uint64_t t5; // timestamp when transmitted sends second frame

void dwm3000_isr() {
  uwb_irq = true;
}

dwt_config_t config = {
  5,
  DWT_PLEN_128,
  DWT_PAC8,
  9,
  9,
  1,
  DWT_BR_6M8,
  DWT_PHRMODE_STD,
  DWT_PHRRATE_STD,
  129,
  DWT_STS_MODE_OFF,
  DWT_STS_LEN_64,
  DWT_PDOA_M0
};

void setup() {
  Serial.begin(115200);

  uint32_t t = millis();
  while (!Serial && (millis() - t < 3000)); // wait for serial to connect, but if it takes more than 3s continue anyways

  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);
  pinMode(JOY_SW, INPUT_PULLUP);

  pinMode(AUTO, INPUT_PULLDOWN);
  
  // spi initialization
  SPI.setSCK(SCK); 
  SPI.setTX(MOSI);
  SPI.setRX(MISO);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  // DWM3000 initialization
  Serial.println("Beginning DWM3000 Initialization");

  resetDWM();

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(CS);

  delay(200); // needed for stable power up

  Serial.print("Waiting for IDLE_RC...");
  while (!dwt_checkidlerc()) {
    Serial.print(".");
    delay(10);
  }
  Serial.println(" Done");

  Serial.print("Performing soft reset...");
  dwt_softreset();
  delay(100);
  while (!dwt_checkidlerc()) {
    Serial.print("x");
    delay(10);
  }
  Serial.println(" Done");
  
  Serial.print("Initializing API... ");
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("Failed!");
    while (1);
  }
  Serial.println("Success!");

  Serial.print("Configuring PHY/MAC... ");
  if (dwt_configure(&config) == DWT_ERROR) {
    Serial.println("Failed!");
    while (1);
  }
  Serial.println("Success!");

  uint32_t dev_id = dwt_readdevid();
  Serial.printf("Dev ID: 0x%X\n", dev_id);
  if (dev_id == DWM_ID) {
    Serial.println("DWM3000 is online and ready");
  } else {
    Serial.println("Unrecognized device ID");
  }
  // End DWM3000 Initialization

  // setup interrupts
  dwt_setinterrupt(DWT_INT_RX | DWT_INT_TFRS, 0, DWT_ENABLE_INT_ONLY);
  // note that the pin_irq is already set up as an input in spiBegin()
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), dwm3000_isr, RISING);

  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  printDWMDiagnostics();
}

//uint32_t last_tx_time = 0;

void loop() {
  if (!digitalRead(AUTO) && initiated) {
    Serial.println("Sending 0x2D");
    initiate(0x2D);
    initiated = false;
  } else if (digitalRead(AUTO) && !initiated) {
    Serial.println("Sending Auto Mode");
    max_distance = 0;
    initiate(0xAA); 
    //last_tx_time = millis();
  }

  // if (initiated && (millis() - last_tx_time > 150)) {xx
  //   // timed out
  //   initiated = false;
  // }

  // poll for interrupts
  if (uwb_irq) {
    //last_tx_time = millis();
    checkData();
  }

  /*static uint32_t last_check = 0;
  if (millis() - last_check > 5000) {
      printDWMDiagnostics();
      last_check = millis();
      
      // ensure dwm is still active
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }*/
}

void checkData() {
  uwb_irq = false;

  delayMicroseconds(10);
  uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

  if (status & SYS_STATUS_RXFCG_BIT_MASK) {
    rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
    rx_len = (uint16_t) (rx_finfo & RX_FINFO_RXFLEN_BIT_MASK); // frame length is stored within the least significant 10 bits
    rx_len -=2; // removes the CRC bytes from the length

    if (rx_len > 64) {
      Serial.printf("[ERROR] rx length of %d is too long (max 64)\n", rx_len);
    } else if (rx_len < 5) { // must be at least 5 if it sent the 4 byte address and a message
      Serial.printf("[ERROR] rx length of %d is too short (min 5)\n", rx_len);
    } else {
      dwt_readrxdata(rx_data, rx_len, 0);

      uint32_t address = (uint32_t) rx_data[0] | ((uint32_t) rx_data[1] << 8) | ((uint32_t) rx_data[2] << 16) | ((uint32_t) rx_data[3] << 24);

      if (address == TRANSMITTER_ADDRESS) {
        Serial.printf("rx_data[4] = 0x%X\n", rx_data[4]);

        if (rx_data[4] == 0x2D) {
          int max_steps = (uint32_t) rx_data[5] | ((uint32_t) rx_data[6] << 8) |
                    ((uint32_t) rx_data[7] << 16) | ((uint32_t) rx_data[8] << 24);

          Serial.printf("Max Steps: %d\n", max_steps);
        } else if (rx_data[4] == 0xDD) {
          int max_reciever_distance = (uint32_t) rx_data[5] | ((uint32_t) rx_data[6] << 8) |
                    ((uint32_t) rx_data[7] << 16) | ((uint32_t) rx_data[8] << 24);

          Serial.printf("Max distance: %d\n", max_reciever_distance);
        }

        else if (rx_data[4] == 0xAA && digitalRead(AUTO)) { // if the receiver sends AA it means it is requesting we do the "initial" sequence again
          initiate(0xA0);
        } else if (rx_data[4] == 0x03) {
          // gather timestamp 4
          uint8_t ts4[5];
          dwt_readrxtimestamp(ts4);

          t4 = (uint64_t) ts4[0] | ((uint64_t) ts4[1] << 8) |
          ((uint64_t) ts4[2] << 16) | ((uint64_t) ts4[3] << 24) |
          ((uint64_t) ts4[4] << 32);
      
          respond(RECEIVER_ADDRESS);
        }
      }
    }
  } else if (status & (SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK | SYS_STATUS_RXFTO_BIT_MASK | SYS_STATUS_RXOVRR_BIT_MASK)) {
    dwt_forcetrxoff();
  }

  dwt_write32bitreg(SYS_STATUS_ID, status); // clear all status bits

  uint32_t leftover_status = dwt_read32bitreg(SYS_STATUS_ID);
  if (leftover_status) {
    dwt_write32bitreg(SYS_STATUS_ID, leftover_status);
  }

  uwb_irq = false;
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void initiate(uint8_t command) {
  initiated = true;

  // build the auto mode packet
  uint8_t tx_packet1[1] = {command};
  send(RECEIVER_ADDRESS, tx_packet1, 1);

  // read the 1st timestamp
  uint8_t ts1[5];
  dwt_readtxtimestamp(ts1);

  t1 = (uint64_t)  ts1[0] | ((uint64_t) ts1[1] << 8) |
       ((uint64_t) ts1[2] << 16) | ((uint64_t) ts1[3] << 24) |
       ((uint64_t) ts1[4] << 32);
}

void respond(uint32_t dest_address) {
  // build the t4 packet
  uint8_t tx_packet[6];
  tx_packet[0] = 0x04; // Byte 4: Indicate t4
  memcpy(&tx_packet[1], &t4, 5); // Bytes 5-9: Timestamp

  send(dest_address, tx_packet, 6);

  // capture timestamp 5
  uint8_t ts5[5];
  dwt_readtxtimestamp(ts5);

  t5 = (uint64_t)  ts5[0] | ((uint64_t) ts5[1] << 8) |
       ((uint64_t) ts5[2] << 16) | ((uint64_t) ts5[3] << 24) |
       ((uint64_t) ts5[4] << 32);

  delay(3);

  uint8_t tx_packet2[11];
  tx_packet2[0] = 0x15; // Byte 4: Indicate t1 and t5
  memcpy(&tx_packet2[1], &t1, 5); // Bytes 5-9: Timestamp 1
  memcpy(&tx_packet2[6], &t5, 5); // Bytes 10-14: Timestamp 

  send(dest_address, tx_packet2, 11);
}

void send(uint32_t dest_address, uint8_t data[], int data_size) {
  // force idle
  dwt_forcetrxoff(); 

  // build packet
  uint8_t tx_packet[4 + data_size];
  memcpy(&tx_packet[0], &dest_address, 4);

  //Serial.printf("data[0] = 0x%X\n", data[0]);

  for (int i = 0; i < data_size; i++) {
    tx_packet[4 + i] = data[i];
  }

  // write packet
  dwt_writetxdata(sizeof(tx_packet), tx_packet, 0);
  dwt_writetxfctrl(sizeof(tx_packet) + 2, 0, 0);
  
  // transmit packet
  if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
    Serial.println("Could not respond");
  }

  // wait until transmit finishes or times out
  uint32_t start_ms = millis();
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
     if (millis() - start_ms > 100) {
      Serial.println("TX Timeout");
      break;
     }
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
}

void resetDWM() {
  Serial.print("Resetting DWM3000... ");
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(10);
  digitalWrite(PIN_RST, HIGH);
  delay(20);
  Serial.println("Done");
}

void printDWMDiagnostics() {
  dwt_rxdiag_t diagnostics;
  dwt_readdiagnostics(&diagnostics);

  Serial.printf("Ipatov Peak: 0x%08X\n", diagnostics.ipatovPeak);
  Serial.printf("Ipatov Power: %u\n", diagnostics.ipatovPower);
  Serial.printf("Ipatov FP Index: %u\n", diagnostics.ipatovFpIndex);
}