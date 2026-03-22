#include <Arduino.h>
#include <SPI.h>
#include <Wire.h> // I2C

#include "DW3000.h"
#include "Adafruit_VL53L0X.h"

#define RECEIVER_ADDRESS    0x10000000 // note that in the transmitter the reciever address will be 0x00000001
#define TRANSMITTER_ADDRESS 0x00000002

// dwm3000 definitions
#define PIN_RST 7
#define PIN_IRQ 6

// stepper definitions
#define DIR1  29
#define STEP1 28
#define EN1   13

#define DIR2  27
#define STEP2 26
#define EN2   12

// spi definitions
#define SCK  2
#define MISO 0
#define MOSI 3
#define CS   1
#define SPI_CLK_SPEED 2000000 // 2MHz

// tof definitions
#define TOF1_XSHUT 11
#define TOF2_XSHUT 10
#define TOF3_XSHUT 9
#define TOF1_OFFSET 20
#define TOF2_OFFSET 0
#define TOF3_OFFSET 0

// i2c definitions
#define SCL 15
#define SDA 14
#define TOF1_ADDR    0x30
#define TOF2_ADDR    0x31
#define TOF3_ADDR    0x32

// main register addresses
#define MAIN_REGISTER 0x00
#define RX_BUFFER0    0x12
#define TX_BUFFER     0x14

// sub register addresses
#define RX_FINFO   0x4C
#define RX_TIME    0x64
#define SYS_STATUS 0x44

// bit masks
#define RXFLEN_MASK 0x3FF

void checkData();
void calculateDistance();
void respond(uint32_t dest_address);
void moveStepper(int dir, int steps, int stepper);
void tofSensorDistance(uint8_t address);

volatile bool uwb_irq = false;
uint16_t rx_len = 0;   
uint32_t rx_info;
uint32_t rx_data[16];

uint64_t t1; // timestamp when transmitter sent frame
uint64_t t2; // timestamp when receiver received frame
uint64_t t3; // timestamp when receiver sent response
uint64_t t4; // timestamp when transmitter recieved response

Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();

void dwm3000_isr() {
  uwb_irq = true;
}

void setup() {
  delay(3000);

  Serial.begin(115200);

  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(EN1, OUTPUT);

  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(EN2, OUTPUT);

  pinMode(TOF1_XSHUT, OUTPUT);
  pinMode(TOF2_XSHUT, OUTPUT);
  pinMode(TOF3_XSHUT, OUTPUT);

  // assign addresses to TOF Sensors
  digitalWrite(TOF1_XSHUT, 0);
  digitalWrite(TOF2_XSHUT, 0);
  digitalWrite(TOF3_XSHUT, 0);
  delay(10);

  Wire1.setSDA(SDA); 
  Wire1.setSCL(SCL);
  Wire1.begin();

  digitalWrite(TOF1_XSHUT, 1);
  delay(10);
  if (!tof1.begin(TOF1_ADDR, false, &Wire1)) {
    Serial.println(F("Failed to boot Sensor 1"));
  }
  tof1.setMeasurementTimingBudgetMicroSeconds(33000);

  digitalWrite(TOF2_XSHUT, 1);
  delay(10);
  if (!tof2.begin(TOF2_ADDR, false, &Wire1)) {
    Serial.println(F("Failed to boot Sensor 2"));
  }
  tof1.setMeasurementTimingBudgetMicroSeconds(33000);

  digitalWrite(TOF3_XSHUT, 1);
  delay(10);
  if (!tof3.begin(TOF3_ADDR, false, &Wire1)) {
    Serial.println(F("Failed to boot Sensor 3"));
  }
  tof1.setMeasurementTimingBudgetMicroSeconds(33000);

  Serial.println(F("All sensors ready!"));
  
  /*SPI.setSCK(SCK); 
  SPI.setTX(MOSI);
  SPI.setRX(MISO);

  DW3000.begin(); // this does SPI.begin() for me
  while (!DW3000.init()); // wait until chip is successfully initialized
  Serial.println("DW3000 initialized");

  DW3000.setChannel(CHANNEL_5);
  DW3000.writeSysConfig();

  DW3000.write(0x00, 0x3C, 0x01); // enable interrupts on the dwm3000
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), dwm3000_isr, RISING);

  DW3000.clearSystemStatus(); 
  DW3000.standardRX(); // start listening
  Serial.println("Listening for UWB frames...");*/
}

void loop () {
  //checkData();
  //moveStepper(0, 100, 3);
  //delay(1000);
  tofSensorDistance(TOF1_ADDR);
  delay(1000);
}

void checkData() {
  if (!uwb_irq) return;
  uwb_irq = false;

  if (DW3000.receivedFrameSucc() != 1) {
    // interrupt was not the one we want
    DW3000.clearSystemStatus(); // clear interrupt
    DW3000.standardRX(); // restart listening
    return;
  }
  
  rx_info = DW3000.read(MAIN_REGISTER, RX_FINFO); // get information about the frame
  rx_len = (rx_info & RXFLEN_MASK) - 2; // frame length is stored within the least significant 10 bits. Length is always +2 for CRC
  
  if (rx_len > 64) {
    Serial.printf("[Error] rx length of %d is too long (max 64)", rx_len);
    Serial.println();
    return;
  }

  // since we only read 4 bytes at a time we need to read rx_len/4 times
  int i = 0;
  for (i = 0; i < rx_len/4; i++) {
    rx_data[i] = DW3000.read(RX_BUFFER0, i*4);
  }
  
  if (rx_len % 4 != 0) {
    uint32_t remaining_data = DW3000.read(RX_BUFFER0, i*4);

    uint32_t mask = 0;
    switch (rx_len % 4) {
      case 1:
        mask = 0xFF; // one extra byte
        break;
      case 2:
        mask = 0xFFFF; // two extra bytes
        break;
      case 3:
        mask = 0xFFFFFF; // three extra bytes
        break;
      default: break;
    }

    rx_data[i] = (remaining_data & mask);
  }

  Serial.println("Received Frame:");
  for (int i = 0; i < rx_len; i++) {
    Serial.print(rx_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if (rx_len < 4) return; // must be at least 4 if it sent the 4 byte address

  uint32_t address = rx_data[0];
  if (address != RECEIVER_ADDRESS) return; // not for me!

  if (rx_len == 13) { // this means a timestamp was sent (timestamp is 5 bytes)
    uint64_t ts = (rx_data[2]) | ((uint64_t)(rx_data[3] & 0xFF) << 32); // timestamp
    if (rx_data[1] == 0x10000000) {
      // load timestamp value into t1, 0x01 means it is t1 that is being sent (0x01 was sent, so 0x10000000 will be received)
      t1 = ts;

      uint64_t ts_low = DW3000.read(MAIN_REGISTER, RX_TIME);
      uint64_t ts_high = DW3000.read(MAIN_REGISTER, RX_TIME + 4) & 0xFF;
      t2 = (ts_high << 32) | ts_low; // this is the time when the receiver received the data
  
      respond(TRANSMITTER_ADDRESS);
    } else if (rx_data[1] == 0x40000000) {
      // load timestamp value into t4, 0x04 means it is t4 that is being sent (0x04 was sent, so 0x40000000 will be received)
      t4 = ts;

      // after receiving t4 we don't want to do anymore measurements, we have everything we need now
      calculateDistance();

      DW3000.clearSystemStatus(); // clear interrupt
      DW3000.standardRX(); // start listening again
      return;
    }
  }

  DW3000.clearSystemStatus(); // clean up including clearing interrupt
  DW3000.standardRX(); // start listening again
}

void respond(uint32_t dest_address) {
  DW3000.write(TX_BUFFER, 0x00, dest_address);
  DW3000.write(TX_BUFFER, 0x04, 0x03); // 0x03 means it is about to measure t3 (note that the transmitter will receive this as 0x03000000)
  DW3000.standardTX();

  // now we wait until the transmit finishes
  while (!(DW3000.read(MAIN_REGISTER, SYS_STATUS) & 0x80)); // Bit 7 of SYS_STATUS is the transmit sent bit
  t3 = DW3000.readTXTimestamp(); // this is the timestamp that the receiever sends its response
  DW3000.write(MAIN_REGISTER, SYS_STATUS, 0x80); // writing a one to bit 7 clears it

  // this commented code is how the transmitter will send it's two timestamps
  // for t1 it first sends 0x000000AA to indicate automode then it measures the TXTimestamp and sends at as so:
  // DW3000.write(TX_BUFFER, 0x00, dest_address);
  // DW3000.write(TX_BUFFER, 0x04, 0x03);
  // DW3000.write(TX_BUFFER, 0x08, t1 & 0xFFFFFFFF);
  // DW3000.write(TX_BUFFER, 0x0C, (t1 >> 32) & 0xFF);
  // DW3000.standardTX();
}

void calculateDistance() {
  long double clock_offset = DW3000.getClockOffset();
  double round_trip_time = (double) (t4 - t1) * (1 + clock_offset); // corrected to take the offset between the transmitter and receiver clocks into account
  double reply_time = (double) (t3 - t2);

  float ToF = 15.65E-12 * (round_trip_time - reply_time)/2; // time of flight in seconds
  float distance = ToF * 3E8 * 100; // multiply ToF by 100c to get the distance in cm
  distance -= 51.1; // this is supposed to account for the travel time through the pcb traces. change this as necessary if there is a constant offset noticed

  Serial.println("Distance: " + String(distance));
} 

void moveStepper(int dir, int steps, int stepper) {
  Serial.printf("Direction: %d, Steps: %d, Stepper: %d\n", dir, steps, stepper);

  if (stepper == 3) { // both steppers
    digitalWrite(DIR1, dir);
    digitalWrite(EN1, 0);

    digitalWrite(DIR2, dir);
    digitalWrite(EN2, 0);

    for (int i = 0; i < steps*8; i++) { // multiply steps by 8 because we are in microstepping mode where each step is 1/8th
      digitalWrite(STEP1, 1);
      digitalWrite(STEP2, 1);
      delayMicroseconds(100);
      digitalWrite(STEP1, 0);
      digitalWrite(STEP2, 0);
      delayMicroseconds(100);
    }

    digitalWrite(EN1, 1);
    digitalWrite(EN2, 1);
    return;
  }

  int dir_pin = DIR1;
  int step_pin = STEP1;
  int en_pin = EN1;

  if (stepper == 2) {
    dir_pin = DIR2;
    step_pin = STEP2;
    en_pin = EN2;
  }

  digitalWrite(dir_pin, dir);
  digitalWrite(en_pin, 0);

  for (int i = 0; i < steps*8; i++) { // multiply steps by 8 because we are in microstepping mode where each step is 1/8th
    digitalWrite(step_pin, 1);
    delayMicroseconds(100);
    digitalWrite(step_pin, 0);
    delayMicroseconds(100);
  }

  digitalWrite(en_pin, 1);
}

void tofSensorDistance(uint8_t address) {
  VL53L0X_RangingMeasurementData_t measure;
  uint16_t offset = 0;

  switch (address) {
    case TOF1_ADDR:
      tof1.rangingTest(&measure, false);
      offset = TOF1_OFFSET;
      break;
    case TOF2_ADDR:
      tof2.rangingTest(&measure, false);
      offset = TOF2_OFFSET;
      break;
    case TOF3_ADDR:
      tof3.rangingTest(&measure, false);
      offset = TOF3_OFFSET;
      break;
    default: break;
  }

  if (measure.RangeStatus != 4) { // 4 means distance is out of range
    Serial.printf("Distance: %d mm\n", measure.RangeMilliMeter - offset);
  } else {
    Serial.print("S1: Out of range\n");
  }
}