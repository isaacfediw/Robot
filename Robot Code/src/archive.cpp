#include "DWM_Constants.h"

/* This code was an attempt to communicate with the dwm3000 without a library. 
The SPI communication methods work, however the initialization proved too difficult.

  Headers and Definitions:
  // void getDWMDeviceID();
  // uint64_t fullAccessedRead(uint8_t base_reg, uint8_t sub_reg, int data_size = 8);
  // void fullAccessedWrite(uint8_t base_reg, uint8_t sub_reg, uint64_t data, int data_size);

  // #define BIT0 0x01
  // #define BIT1 0x02
  // #define BIT2 0x04
  // #define BIT3 0x08
  // #define BIT4 0x10
  // #define BIT5 0x20
  // #define BIT6 0x40
  // #define BIT7 0x80
  // #define BIT8 0x100
  End Headers and Definitions

  // resetDWM();

  // // ** force the device into idle **
  // SPI.beginTransaction(SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE0));
  // digitalWrite(CS, LOW);

  // SPI.transfer(FAST_CMD | (CMD_TRXOFF << 1));

  // digitalWrite(CS, HIGH);
  // SPI.endTransaction();
  // delay(5);

  // // ** check dev_id **
  // if (fullAccessedRead(MAIN_REGISTER, DEV_ID, 8) != DWM_ID) 
  //   Serial.println("[WARN] DEV_ID does not match");

  // // ** enable long frame mode **
  // fullAccessedWrite(MAIN_REGISTER, SYS_CFG, fullAccessedRead(MAIN_REGISTER, SYS_CFG, 4) | BIT4, 4);

  // old library stuff
  //DW3000.begin(); // SPI stuff, done
  // while (true) {
  //   Serial.println("Attempting Initialization");
     //if(DW3000.init()) break;
  // }; // wait until chip is successfully initialized
  // Serial.println("DW3000 initialized");

  // DW3000.setChannel(CHANNEL_5);
  // DW3000.writeSysConfig();

  // DW3000.write(0x00, 0x3C, 0x01); // enable interrupts on the dwm3000
  // attachInterrupt(digitalPinToInterrupt(PIN_IRQ), dwm3000_isr, RISING);

  // DW3000.clearSystemStatus(); 
  // DW3000.standardRX(); // start listening
  // Serial.println("Listening for UWB frames...");
  // end old library stuff

  // // ** write mac address for receiver **
  // fullAccessedWrite(MAIN_REGISTER, EUI_64, RECEIVER_ADDRESS, 8);
  // delay(10);

  // // ** set ldo_kick and bias_kick **
  // uint16_t otp_cfg = fullAccessedRead(OTP_REGISTER, OTP_CFG, 2);
  // otp_cfg |= (BIT7 | BIT8); // LDO_KICK and BIAS_KICK

  // fullAccessedWrite(OTP_REGISTER, OTP_CFG, otp_cfg, 2);
  // delay(2);

  // // verify latching
  // uint16_t verify_otp = fullAccessedRead(OTP_REGISTER, OTP_CFG, 2);
  // if (verify_otp != otp_cfg) Serial.println("[WARN] OTP Control was not changed successfully");

  // // ** gather and change the clock from the default one to the FAST_RC/4 clock **
  // uint32_t ctrl_data = fullAccessedRead(PMSC_CTRL, CLK_CTRL, 4);
  // Serial.printf("0x%08X\n", ctrl_data);
  // ctrl_data |= 0x01;
  // Serial.printf("0x%X\n", ctrl_data);

  // fullAccessedWrite(PMSC_CTRL, CLK_CTRL, ctrl_data, 4);
  // delay(5);

  // // verify latching
  // uint32_t verify_data = fullAccessedRead(PMSC_CTRL, CLK_CTRL, 4);
  // Serial.printf("Verified CLK_CTRL: 0x%08X\n", verify_data);

  // if (ctrl_data != verify_data) Serial.println("[WARN] Clock was not changed successfully");

  // // ** enable configuration retention across sleep states **
  // uint32_t aon_dig_cgf = fullAccessedRead(ALWAYS_ON, AON_DIG_CGF, 4);
  // aon_dig_cgf |= BIT0;
  // fullAccessedWrite(ALWAYS_ON, AON_DIG_CGF, aon_dig_cgf, 4);
  // delay(5);
  
  // // verify latching
  // uint16_t verify_aon_dig = fullAccessedRead(ALWAYS_ON, AON_DIG_CGF, 4);
  // if (verify_aon_dig != aon_dig_cgf) Serial.println("[WARN] Configuration retention was not changed successfully");*/

  /*void getDWMDeviceID() {
  Serial.println("Device ID Read");

  resetDWM();

  Serial.println("Attempting raw register read...");

  SPI.beginTransaction(SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);

  // Send the first byte, which includes the header and the base address
  uint8_t setup_byte = SHORT_READ | (MAIN_REGISTER << 1) | 0x00;
  SPI.transfer(setup_byte);

  // Shift in the 4 bytes of the Device ID register data payload
  // The payload is 0s just to give the device clock cycles to respond
  uint8_t byte1 = SPI.transfer(READ_DUMMY);
  uint8_t byte2 = SPI.transfer(READ_DUMMY);
  uint8_t byte3 = SPI.transfer(READ_DUMMY);
  uint8_t byte4 = SPI.transfer(READ_DUMMY);

  digitalWrite(CS, HIGH);
  SPI.endTransaction();

  Serial.printf("Device ID: 0x%X", 
    ((uint32_t)byte4 << 24) | 
    ((uint32_t)byte3 << 16) | 
    ((uint32_t)byte2 << 8)  | 
    byte1);
}*/

/*
uint64_t fullAccessedRead(uint8_t base_reg, uint8_t sub_reg, int data_size) {
  SPI.beginTransaction(SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);

  uint8_t setup_byte = FULL_READ | (base_reg << 1) | ((BIT6 & sub_reg) >> 6);
  uint8_t addr_byte = (sub_reg << 2) | 0x00;

  SPI.transfer(setup_byte);
  SPI.transfer(addr_byte);

  uint64_t data = 0;
  for (int i = 0; i < data_size; i++) {
    data |= ((uint64_t) SPI.transfer(READ_DUMMY) << (i*8));
  }

  digitalWrite(CS, HIGH);
  SPI.endTransaction();

  return data;
}

// assumes msb is the first index of the data array
void fullAccessedWrite(uint8_t base_reg, uint8_t sub_reg, uint64_t data, int data_size) {
  SPI.beginTransaction(SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);

  uint8_t setup_byte = FULL_WRITE | (base_reg << 1) | ((BIT6 & sub_reg) >> 6);
  uint8_t addr_byte = (sub_reg << 2) | 0x00;

  SPI.transfer(setup_byte);
  SPI.transfer(addr_byte);

  uint8_t data_arr[data_size];
  memcpy(data_arr, &data, data_size);

  for (int i = 0; i < data_size; i++) {
    // send the data least significant byte first (memcpy already flips it to be LSB first :D)
    SPI.transfer(data_arr[i]);
  }

  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}*/