#include <Arduino.h>
#include <SPI.h>

#include "dw3000.h"

// dwm3000 definitions
#define PIN_IRQ  6
#define PIN_RST  7
#define PIN_WAKE 8

#define DWM_ID 0xDECA0302

#define RECEIVER_ADDRESS    0x00000001
#define TRANSMITTER_ADDRESS 0x20000000

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

void checkData();
void calculateDistance();
void respond(uint32_t dest_address);

volatile bool uwb_irq = false;
uint16_t rx_len = 0;   
uint32_t rx_finfo;
uint8_t rx_data[64];

uint64_t t1; // timestamp when transmitter sent frame
uint64_t t2; // timestamp when receiver received frame
uint64_t t3; // timestamp when receiver sent response
uint64_t t4; // timestamp when transmitter recieved response

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
  
  /*// spi initialization
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

  // note that the pin_irq is already set up as an input in spiBegin()
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), dwm3000_isr, RISING);

  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  printDWMDiagnostics();*/
}

void loop () {
  if (digitalRead(AUTO)) Serial.println("AUTO Mode");

  /*// poll for interrupts
  if (uwb_irq) {
    Serial.println("Checking Data");
    checkData();
  }

  static uint32_t last_check = 0;
  if (millis() - last_check > 5000) {
      printDWMDiagnostics();
      last_check = millis();
      
      // ensure dwm is still active
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }*/
}

void checkData() {
  if (!uwb_irq) return;
  uwb_irq = false;

  uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

  if (status & SYS_STATUS_RXFCG_BIT_MASK) {
    rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
    rx_len = (uint16_t) (rx_finfo & RX_FINFO_RXFLEN_BIT_MASK); // frame length is stored within the least significant 10 bits

    rx_len -=2; // removes the CRC bytes from the length

    if (rx_len > 64) {
      Serial.printf("[ERROR] rx length of %d is too long (max 64)\n", rx_len);
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      return;
    }

    if (rx_len < 4) {
      Serial.printf("[ERROR] rx length of %d is too short (min 4)\n", rx_len);
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      return; // must be at least 4 if it sent the 4 byte address
    } 

    dwt_readrxdata(rx_data, rx_len, 0);

    uint32_t address = (uint32_t) rx_data[0] | ((uint32_t) rx_data[1] << 8) | ((uint32_t) rx_data[2] << 16) | ((uint32_t) rx_data[3] << 24);
    if (address != RECEIVER_ADDRESS) {
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      return; // not for me!
    }

    if (rx_len == 10) { // this means a timestamp was sent (timestamp is 5 bytes)
      uint64_t ts = (uint64_t) rx_data[5] | ((uint64_t) rx_data[6] << 8) |
       ((uint64_t) rx_data[7] << 16) | ((uint64_t) rx_data[8] << 24) |
       ((uint64_t) rx_data[9] << 32);

      if (rx_data[4] == 0x10) {
        // load timestamp value into t1, 0x01 means it is t1 that is being sent (0x01 was sent, so 0x10 will be received)
        t1 = ts;

        uint8_t ts2[5];
        dwt_readrxtimestamp(ts2);

        t2 = (uint64_t) ts2[0] | ((uint64_t) ts2[1] << 8) |
        ((uint64_t) ts2[2] << 16) | ((uint64_t) ts2[3] << 24) |
        ((uint64_t) ts2[4] << 32);
    
        respond(TRANSMITTER_ADDRESS);
      } else if (rx_data[4] == 0x40) {
        // load timestamp value into t4, 0x04 means it is t4 that is being sent (0x04 was sent, so 0x40 will be received)
        t4 = ts;

        // after receiving t4 we don't want to do anymore measurements, we have everything we need now
        calculateDistance();

        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
      }
    }
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }

  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void respond(uint32_t dest_address) {
  uint64_t data = (uint64_t) (dest_address) | ((uint64_t) (0x03) << 32);
  
  uint8_t data_arr[5];
  memcpy(data_arr, &data, sizeof(data_arr));

  dwt_writetxdata(sizeof(data_arr), data_arr, 0);
  dwt_writetxfctrl(sizeof(data_arr) + 2, 0, 0);
  
  int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

  if (ret != DWT_SUCCESS) {
    Serial.println("Could not respond");
  }

  // now we wait until the transmit finishes
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));

  uint8_t ts3[5];
  dwt_readtxtimestamp(ts3);

  t3 = (uint64_t) ts3[0] | ((uint64_t) ts3[1] << 8) |
        ((uint64_t) ts3[2] << 16) | ((uint64_t) ts3[3] << 24) |
        ((uint64_t) ts3[4] << 32);

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); // clear the transmit finish status bit by sending a 1 to it
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // this commented code is how the transmitter will send its two timestamps
  // for t1 it first sends 0xAA to indicate automode then it measures the TXTimestamp and sends at as so:

  // __uint128_t data = (__uint128_t) (dest_address) | ((__uint128_t) (0x01) << 32) | ((__uint128_t) (t1) << 40);

  // uint8_t data_arr[10];
  // memcpy(data_arr, &data, sizeof(data));

  // dwt_writetxdata(sizeof(data_arr), data_arr, 0);
  // dwt_writetxfctrl(sizeof(data_arr) + 2, 0, 0);
  
  // int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
}

void calculateDistance() {
  int16_t clock_offset = dwt_readclockoffset();
  double round_trip_time = (double) (t4 - t1) * (1 + clock_offset); // corrected to take the offset between the transmitter and receiver clocks into account
  double reply_time = (double) (t3 - t2);

  float ToF = 15.65E-12 * (round_trip_time - reply_time)/2; // time of flight in seconds
  float distance = ToF * 3E8 * 100; // multiply ToF by 100 to get the distance in cm
  distance -= 51.1; // this is supposed to account for the travel time through the pcb traces. change this as necessary if there is a constant offset noticed

  Serial.println("Distance: " + String(distance));
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