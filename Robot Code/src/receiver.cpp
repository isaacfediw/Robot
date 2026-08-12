#include <Arduino.h>
#include <SPI.h>
#include <Wire.h> // I2C

#include "dw3000.h"
#include "Adafruit_VL53L0X.h"
#include "stepper.h"

// stepper definitions
#define LEFT_STEPPER_FORWARD   1
#define LEFT_STEPPER_BACKWARD  0
#define RIGHT_STEPPER_FORWARD  0
#define RIGHT_STEPPER_BACKWARD 1

#define TURNING_RADIUS 50

// left stepper
#define DIR1  29
#define STEP1 28
#define EN1   13

// right stepper
#define DIR2  27
#define STEP2 26
#define EN2   12

// tof definitions
#define TOF1_XSHUT 11 // front
#define TOF2_XSHUT 10 // right
#define TOF3_XSHUT 9  // left

#define TOF1_OFFSET 0 //20
#define TOF2_OFFSET 29
#define TOF3_OFFSET 52

#define OUT_OF_RANGE -20.0f

// i2c definitions
#define SCL 15
#define SDA 14
#define TOF1_ADDR    0x30
#define TOF2_ADDR    0x31
#define TOF3_ADDR    0x32

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
// DWM3000 Receives Data MSB First

#define READ_DUMMY 0x00

void printDWMDiagnostics();
void resetDWM();

void moveSteppers(int left_steps, int right_steps);
void checkData();
float calculateDistance();
void respond(uint32_t dest_address);
void send(uint32_t dest_address, uint8_t data[], int data_size);
float tofSensorDistance(uint8_t address);
void calibrateTof();

volatile bool uwb_irq = true;
uint16_t rx_len = 0;   
uint32_t rx_finfo;
uint8_t rx_data[64];

float d1, d2, d3, d4;    // d1 -> initial distance between robot and transmitter
                         // d2 -> distance between robot and transmitter after moving by L
#define L 25             // constant 25cm to move forward by when triangulating
#define L_STEPS 235      // L in steps
#define STEPS_360 470    // 470 steps for 360 degree turn
#define D_MULT 9.41      // multiply d (in cm) by 9.41 to get how many steps to move
#define WIDTH 12.1       // wheel to wheel outer width of robot is 12cm
#define LENGTH 13.57     // front of the car to the dwm3000 is 13.57cm
#define RADIUS 3.3825    // 3.3825cm radius of wheel (no tire)

float current_distance;
float max_distance;
int max_distance_steps; // how many steps it took to get to the max distance

enum class STATES {INITIAL, SECOND, THIRD, VERIFY, FINISH};
STATES STATE = STATES::INITIAL;

uint64_t t1; // timestamp when transmitter sent frame
uint64_t t2; // timestamp when receiver received frame
uint64_t t3; // timestamp when receiver sent response
uint64_t t4; // timestamp when transmitter received response
uint64_t t5; // timestamp when transmitted sends second frame
uint64_t t6; // timestamp when receiever received second frame

Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();

int leftPins[3] = {STEP1, DIR1, EN1};
int rightPins[3] = {STEP2, DIR2, EN2};
stepper leftStepper(leftPins);
stepper rightStepper(rightPins);

bool forward = false;
int turn_attempts = 0;

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
  tof2.setMeasurementTimingBudgetMicroSeconds(33000);

  digitalWrite(TOF3_XSHUT, 1);
  delay(10);
  if (!tof3.begin(TOF3_ADDR, false, &Wire1)) {
    Serial.println(F("Failed to boot Sensor 3"));
  }
  tof3.setMeasurementTimingBudgetMicroSeconds(33000);

  Serial.println(F("Booted sensors ready!"));
  
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

void loop() {
  // poll for interrupts
  if (uwb_irq) checkData();

  /*static uint32_t last_check = 0;
  if (millis() - last_check > 5000) {
      printDWMDiagnostics();
      last_check = millis();
      
      // ensure dwm is still active
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }*/

  /*leftStepper.stepperLoop();
  rightStepper.stepperLoop();

  if (forward) {
    // check that neither motor is busy
    if (!leftStepper.isBusy() && !rightStepper.isBusy()) {
      leftStepper.moveStepper(LEFT_STEPPER_FORWARD, 200);
      rightStepper.moveStepper(RIGHT_STEPPER_FORWARD, 200);

      turn_attempts = 0;
    }

    forward = false;
  } else {
    // check that neither motor is busy
    if (!leftStepper.isBusy() && !rightStepper.isBusy()) {
      // if turn_attempts is relatively high, we are probably stuck so we should try going forwards
      if (turn_attempts < 4) {
        // check if we need to turn
        float left_dist = tofSensorDistance(TOF3_ADDR);
        float right_dist = tofSensorDistance(TOF2_ADDR);

        // turning left
        if (right_dist == OUT_OF_RANGE || (left_dist - right_dist > 50)) {
          leftStepper.moveStepper(LEFT_STEPPER_BACKWARD, TURNING_RADIUS);
          rightStepper.moveStepper(RIGHT_STEPPER_FORWARD, TURNING_RADIUS);
        }

        // turning right
        if (left_dist == OUT_OF_RANGE || (left_dist - right_dist < -50)) {
          leftStepper.moveStepper(LEFT_STEPPER_FORWARD, TURNING_RADIUS);
          rightStepper.moveStepper(RIGHT_STEPPER_BACKWARD, TURNING_RADIUS);
        }

        turn_attempts ++;
      }

      // after turning continue going straight
      forward = true;
    }
  }*/
}

float theta_steps;
int total_steps = 0;

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

      if (address = RECEIVER_ADDRESS) {
        Serial.printf("rx_data[4] = 0x%X\n", rx_data[4]);

        if (rx_data[4] == 0x2D) {
          uint8_t data[5];
          data[0] = 0x2D;
          memcpy(&data[1], &max_distance_steps, 4);
          send(TRANSMITTER_ADDRESS, data, 5);

          STATE = STATES::FINISH;
          //Serial.printf("Total steps: %d\n", total_steps);
        }

        else if (rx_data[4] == 0xAA) { // AA means auto mode started from transmitter so we need to go back to initial state
          // moveSteppers(-310, 310);
          // return;

          STATE = STATES::INITIAL;
          total_steps = 0;
          max_distance = 0;

          // Capture t2 IMMEDIATELY for the poll packet (0xAA)
          uint8_t ts2[5];

          dwt_readrxtimestamp(ts2);

          t2 = (uint64_t) ts2[0] | ((uint64_t) ts2[1] << 8) |
          ((uint64_t) ts2[2] << 16) | ((uint64_t) ts2[3] << 24) |
          ((uint64_t) ts2[4] << 32);

          // respond and capture t3
          respond(TRANSMITTER_ADDRESS);
        } else if (rx_data[4] == 0xA0) { // A0 means another request was recieved at the transmitter so we want to keep the state as is
          // Capture t2 IMMEDIATELY for the poll packet (0xA0)
          uint8_t ts2[5];

          dwt_readrxtimestamp(ts2);

          t2 = (uint64_t) ts2[0] | ((uint64_t) ts2[1] << 8) |
          ((uint64_t) ts2[2] << 16) | ((uint64_t) ts2[3] << 24) |
          ((uint64_t) ts2[4] << 32);

          // respond and capture t3
          respond(TRANSMITTER_ADDRESS);
        } else if (rx_len == 10) { // this means a timestamp was sent (timestamp is 5 bytes)
          uint64_t ts = (uint64_t) rx_data[5] | ((uint64_t) rx_data[6] << 8) |
          ((uint64_t) rx_data[7] << 16) | ((uint64_t) rx_data[8] << 24) |
          ((uint64_t) rx_data[9] << 32);

          if (rx_data[4] == 0x04) {
            // load timestamp value into t4, 0x04 means it is t4 that is being sent
            t4 = ts;

            uint8_t ts6[5];

            dwt_readrxtimestamp(ts6);

            t6 = (uint64_t)  ts6[0] | ((uint64_t) ts6[1] << 8) |
                ((uint64_t) ts6[2] << 16) | ((uint64_t) ts6[3] << 24) |
                ((uint64_t) ts6[4] << 32);
          }
        } else if (rx_len == 15 && rx_data[4] == 0x15) { // two timestamps sent (t1 and t5)
          t1 = (uint64_t)  rx_data[5] | ((uint64_t) rx_data[6] << 8) |
              ((uint64_t) rx_data[7] << 16) | ((uint64_t) rx_data[8] << 24) |
              ((uint64_t) rx_data[9] << 32);

          t5 = (uint64_t)  rx_data[10] | ((uint64_t) rx_data[11] << 8) |
              ((uint64_t) rx_data[12] << 16) | ((uint64_t) rx_data[13] << 24) |
              ((uint64_t) rx_data[14] << 32);

          current_distance = calculateDistance();

          if (current_distance > max_distance) {
            max_distance = current_distance;
            max_distance_steps = total_steps;
          }

          // now that we have the final two timestamps we can calculate the distance
          moveSteppers(-10, 10);
          total_steps += 10;

          // once we've done the 360 turn and found the max distance we need to go the max_distance_steps
          // and then drive that max distance minus the length of the robot
          if (total_steps >= STEPS_360) {
            //moveSteppers(-max_distance_steps, max_distance_steps);
            delay(500);

            for (int i = 0; i < max_distance_steps; i+= 10) {
              moveSteppers(-10, 10);
            }

            int distance_steps = (max_distance - LENGTH) * D_MULT;
            moveSteppers(distance_steps, distance_steps);
            
            STATE = STATES::FINISH;
          }

          /*if (STATE == STATES::INITIAL) {
            d1 = calculateDistance();
            //Serial.printf("d1: %.2f\n", d1);

            // move forward by L
            moveSteppers(L_STEPS, L_STEPS);

            STATE = STATES::SECOND;
          } else if (STATE == STATES::SECOND) {
            d2 = calculateDistance();

            float theta = PI - acos(L*L + d2*d2 - d1*d1) / (2*L*d2);
            float theta_cm = WIDTH * sin(theta);
            theta_steps = theta_cm * 360/(2*PI*RADIUS) * 1/1.8;

            //Serial.printf("theta: %.2f degrees\n", degrees(theta));

            // turn right by theta
            moveSteppers(theta_steps, -theta_steps);

            // move forward by L
            moveSteppers(L_STEPS, L_STEPS);

            STATE = STATES::THIRD;        
          } else if (STATE == STATES::THIRD) {
            d3 = calculateDistance();
            //Serial.printf("d2: %.2f\n", d2);

            // subtract 13.57 because the dwm is 13.57cm from the front of the car
            // and while we are moving by d2_steps we are facing the transmitter
            int d2_steps = (d2 - 12) * D_MULT;
            int d3_steps = (d3 - 12) * D_MULT;

            // if we are now farther than d2 it means turning right was incorrect
            if (d3 > d2) {
              // backup by L
              moveSteppers(-L_STEPS, -L_STEPS);

              // turn left by 2*theta
              moveSteppers(-theta_steps*2, theta_steps*2);

              // move forward by d2
              moveSteppers(d2_steps, d2_steps);
            } else { // if we make it here turning right was correct
              moveSteppers(d3_steps, d3_steps);
            }

            // once this finishes we *should* be at the transmitter but we can double check
            STATE = STATES::VERIFY;
          } else if (STATE == STATES::VERIFY) {
            d4 = calculateDistance();

            // if we are not within L we can try again
            if (d4 < L) {
              STATE = STATES::FINISH;
            } else {
              STATE = STATES::INITIAL;
            }
          }

          if (STATE != STATES::FINISH) {
            uint8_t data[1] = {0xAA};
            send(TRANSMITTER_ADDRESS, data, 1);
          } else {
            STATE = STATES::INITIAL;
          }*/

          if (STATE != STATES::FINISH) {
            //Serial.println("Sending 0xAA");
            uint8_t data[1] = {0xAA};
            send(TRANSMITTER_ADDRESS, data, 1);
          } else {
            uint8_t data[5];
            data[0] = 0xDD;
            int sending_distance = (int) max_distance;
            memcpy(&data[1], &sending_distance, 4);
            send(TRANSMITTER_ADDRESS, data, 5);
          }
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

void respond(uint32_t dest_address) {
  uint8_t data[1] = {0x03};
  send(dest_address, data, 1);

  uint8_t ts3[5];
  dwt_readtxtimestamp(ts3);

  t3 = (uint64_t)  ts3[0] | ((uint64_t) ts3[1] << 8) |
       ((uint64_t) ts3[2] << 16) | ((uint64_t) ts3[3] << 24) |
       ((uint64_t) ts3[4] << 32);
}

float filtered_distance = 0;
bool initial_reading = true;

float calculateDistance() {
  // Serial.printf("t1: %d\n", t1);
  // Serial.printf("t2: %d\n", t2);
  // Serial.printf("t3: %d\n", t3);
  // Serial.printf("t4: %d\n", t4);
  // Serial.printf("t5: %d\n", t5);
  // Serial.printf("t6: %d\n", t6);

  double round_trip_d1 = (double) ((t4 - t1) & 0xFFFFFFFFFF);
  double reply_d1 = (double) ((t3 - t2) & 0xFFFFFFFFFF);
  double round_trip_d2 = (double) ((t6 - t3) & 0xFFFFFFFFFF);
  double reply_d2 = (double) ((t5 - t4) & 0xFFFFFFFFFF);

  // Serial.printf("Round trip d1: %d\n", round_trip_d1);
  // Serial.printf("Reply d1: %d\n", reply_d1);
  // Serial.printf("Round trip d2: %d\n", round_trip_d2);
  // Serial.printf("Reply d2: %d\n", reply_d2);

  double tof = ((round_trip_d1 * round_trip_d2) - (reply_d1 * reply_d2)) / (round_trip_d1 + round_trip_d2 + reply_d1 + reply_d2);
  double tof_seconds = tof * 15.65E-12;
  float distance = tof_seconds * 3E8 * 100; // distance in cm
  distance += 15; // correct for measured offset

  // // Apply Exponential Moving Average (EMA) filter
  // if (initial_reading) {
  //   filtered_distance = distance;
  //   initial_reading = false;
  // } else {
  //   float alpha = 0.2f; // Smoothing factor: 0.1 = very smooth/slow, 0.5 = twitchy/fast
  //   filtered_distance = (alpha * distance) + ((1.0f - alpha) * filtered_distance);
  // } 

  //return filtered_distance;

  Serial.printf("Distance: %.2f\n", distance);
  return distance;
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
}

float tofSensorDistance(uint8_t address) {
  VL53L0X_RangingMeasurementData_t measure;
  uint16_t offset = 0;
  bool applyAngleCorrection = false;

  switch (address) {
    case TOF1_ADDR:
      tof1.rangingTest(&measure, false);
      offset = TOF1_OFFSET;
      break;
    case TOF2_ADDR:
      tof2.rangingTest(&measure, false);
      offset = TOF2_OFFSET;
      applyAngleCorrection = true;
      break;
    case TOF3_ADDR:
      tof3.rangingTest(&measure, false);
      offset = TOF3_OFFSET;
      applyAngleCorrection = true;
      break;
    default: break;
  }

  // if it's out of range sometimes it gives the status, other times it measures 8191 but this is consistant behaviour
  if (measure.RangeStatus != 4 && measure.RangeMilliMeter != 8191) { // 4 means distance is out of range
    int16_t final_calculated_dist = (int16_t) (measure.RangeMilliMeter - offset);

    if (applyAngleCorrection) {
      final_calculated_dist *= 0.9548f; // cos(17.3 deg)
    }

    Serial.printf("Distance: %d mm\n", final_calculated_dist);
    return final_calculated_dist;
  } else {
    Serial.print("Out of range\n");
    return OUT_OF_RANGE;
  }
}

// to use, place object 100mm away from and perpendicular to the sensor
void calibrateTof() {
  float sum = 0.0f;
  for (int i = 0; i < 10; i++) {
    sum += tofSensorDistance(TOF2_ADDR);
  }

  Serial.printf("AVG: %.2f\n", sum/10.0f);
  delay(500);
}

// moves both steppers (not just a scheduler)
// put steps negative to be backwards, postive for forwards
void moveSteppers(int left_steps, int right_steps) {
  int left_dir = left_steps < 0 ? LEFT_STEPPER_BACKWARD : LEFT_STEPPER_FORWARD;
  int right_dir = right_steps < 0 ? RIGHT_STEPPER_BACKWARD : RIGHT_STEPPER_FORWARD;

  leftStepper.moveStepper(left_dir, abs(left_steps));        
  rightStepper.moveStepper(right_dir, abs(right_steps));  

  while (leftStepper.isBusy() || rightStepper.isBusy()) {
    leftStepper.stepperLoop();
    rightStepper.stepperLoop();
  }
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