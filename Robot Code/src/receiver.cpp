#include <Arduino.h>
#include <SPI.h>
#include <vector>

#include "dw3000.h"
#include "Adafruit_VL53L0X.h"
#include "stepper.h"

#include "perlin_noise_walking.cpp"


// define statements
// dwm3000 definitions
#define PIN_IRQ 6
#define PIN_RST 7
#define PIN_WAKE 8

#define DWM_ID 0xDECA0302

#define MASK_40BIT 0x00FFFFFFFFFF

#define RECEIVER_ADDRESS    0x00000001
#define TRANSMITTER_ADDRESS 0x00000002

#define TX_ANT_DLY 16351
#define RX_ANT_DLY 16351

// #define TX_ANT_DLY 16385
// #define RX_ANT_DLY 16385

#define POLL_TX_TO_RESP_RX_DLY_UUS 600

#define NLOS_THRESHOLD_DB 6.0f // Threshold in dBm; values > 6.0 indicate an obstructed path

// spi definitions
#define SCK 2
#define MISO 0
#define MOSI 3
#define CS 1
#define SPI_CLK_SPEED 2000000

// i2c definitions
#define SCL 15
#define SDA 14
#define TOF1_ADDR    0x30
#define TOF2_ADDR    0x31
#define TOF3_ADDR    0x32

// tof definitions
#define TOF1_XSHUT 11 // front
#define TOF2_XSHUT 10 // right
#define TOF3_XSHUT 9  // left

#define TOF1_OFFSET 0 //20
#define TOF2_OFFSET 29
#define TOF3_OFFSET 52

#define OUT_OF_RANGE -20.0f
#define TOO_CLOSE 50 // 50 mm

// distance definitions
#define LENGTH 31
#define UPSAMPLE_FACTOR 4

// stepper definitions
#define LEFT_STEPPER_FORWARD   1
#define LEFT_STEPPER_BACKWARD  0
#define RIGHT_STEPPER_FORWARD  0
#define RIGHT_STEPPER_BACKWARD 1

#define MIN_RANDOM_MODE_STEPS  20
#define MAX_RANDOM_MODE_STEPS  100

// left stepper
#define DIR1  29
#define STEP1 28
#define EN1   13

// right stepper
#define DIR2  27
#define STEP2 26
#define EN2   12


// function definitions
void resetDWM();
double convolve(float h[], double x[]) ;
void checkData();
double calculateDistance();
bool send(uint32_t dest_address, uint8_t data[], int data_size);
void moveSteppers(int left_steps, int right_steps);
float tofSensorDistance(uint8_t address);

void serviceSensorsNonBlocking();


// global variables
int leftPins[3] = {STEP1, DIR1, EN1};
int rightPins[3] = {STEP2, DIR2, EN2};
stepper leftStepper(leftPins);
stepper rightStepper(rightPins);

Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();

std::vector<int> dirs; // directions vector for random mode
StepperNoiseDriver driver;

enum class STATES {INITIAL, MOVING, FINISH};
STATES STATE;

uint64_t t1;
uint64_t t2;
uint64_t t3;
uint64_t t4;
uint64_t t5;
uint64_t t6;

double x[LENGTH];

float h[LENGTH] = { 
    1.6583E-03,1.9273E-03,1.8793E-03,7.1205E-04,-2.4593E-03,-7.7846E-03,
    -1.3926E-02,-1.7847E-02,-1.5438E-02,-2.9032E-03,2.1541E-02,5.6360E-02,
    9.6409E-02,1.3395E-01,1.6071E-01,1.7042E-01,1.6071E-01,1.3395E-01,
    9.6409E-02,5.6360E-02,2.1541E-02,-2.9032E-03,-1.5438E-02,-1.7847E-02,
    -1.3926E-02,-7.7846E-03,-2.4593E-03,7.1205E-04,1.8793E-03,1.9273E-03,
    1.6583E-03
};

volatile bool uwb_irq = false;

extern dwt_txconfig_t txconfig_options;

dwt_config_t config = {
    5,
    DWT_PLEN_1024,
    DWT_PAC32,
    9,
    9,
    1,
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (1025 + 8 - 32),
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};


// function implementations
void dwm3000_isr() {
  uwb_irq = true;
}

void setup() {
    // serial monitor initialization
    Serial.begin(115200);

    uint32_t start = millis();
    while (!Serial && (millis() - start < 3000));
    delay(5);

    // spi initialization
    SPI.setSCK(SCK);
    SPI.setTX(MOSI);
    SPI.setRX(MISO);

    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);

    // dwm3000 initialization
    resetDWM();

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(CS);

    delay(2);
    
    dwt_softreset();
    delay(2);

    while(!dwt_checkidlerc()) delay(10);

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        Serial.println("Could not initialize API");
        while (1);
    }

    if (dwt_configure(&config) == DWT_ERROR) {
        Serial.println("Could not configure device");
        while (1);
    }

    uint32_t dev_id = dwt_readdevid();
    if (dev_id != DWM_ID) {
        Serial.println("Incorrect device ID");
        while (1);
    }

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);

    // setup interrupts
    dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT_ONLY);

     // note that the pin_irq is already set up as an input in spiBegin()
    attachInterrupt(digitalPinToInterrupt(PIN_IRQ), dwm3000_isr, RISING);

    STATE = STATES::INITIAL;

    for (int i = 0; i < LENGTH; i++) {
        x[i] = 0;
    }

    // IT IS VITAL THAT TOF INITIALIZATION HAPPENS AFTER DWM INITIALIZATION
    // tof sensor initialization
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


    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    Serial.println("All initialization complete (receiver)");
}

void loop() {
    if (uwb_irq) checkData();
}

void checkData() {
    Serial.println("Checking data");
    delayMicroseconds(20);

    uwb_irq = false;

    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
    uint16_t rx_len = (uint16_t) (rx_finfo & RX_FINFO_RXFLEN_BIT_MASK) - 2;

    if (rx_len < 5 || rx_len > 64) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
    
    uint8_t rx_data[64];
    dwt_readrxdata(rx_data, rx_len, 0);

    uint32_t address;
    memcpy(&address, &rx_data[0], 4);

    if (address != RECEIVER_ADDRESS) {
        //Serial.println("Address is not that of the receiever");
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
    
    uint8_t data[1];
    double distance;
    
    int left_steps, right_steps;

    float left_sensor_distance, right_sensor_distance;
    bool left_clear, right_clear;

    int choice;

    Serial.printf("Received header 0x%X\n", rx_data[4]);

    switch (rx_data[4]) {
        case 0xAB: // manual mode header

            memcpy(&left_steps, &rx_data[5], 1);
            memcpy(&right_steps, &rx_data[7], 1);

            if (rx_data[6]  == 0x00) left_steps = -left_steps;
            if (rx_data[8] == 0x00) right_steps = -right_steps;

            moveSteppers(left_steps, right_steps);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            break;
        case 0xAC: // random mode header
            /*// in this mode we just move the robot around randomly, accounting for obstacles using the tof sensors
            steps = random(MIN_RANDOM_MODE_STEPS, MAX_RANDOM_MODE_STEPS + 1);

            left_sensor_distance = tofSensorDistance(TOF3_ADDR);
            right_sensor_distance = tofSensorDistance(TOF2_ADDR);

            left_clear = left_sensor_distance == OUT_OF_RANGE || left_sensor_distance > TOO_CLOSE;
            right_clear = right_sensor_distance == OUT_OF_RANGE || right_sensor_distance > TOO_CLOSE;
            // also do same for forward when it's set up

            dirs.clear();

            // always valid till front sensor is added
            // going forward should have a higher chance so the robot actually moves with 'purpose'
            dirs.push_back(0);
            dirs.push_back(0);
            dirs.push_back(0);

            dirs.push_back(1); // always valid
            if (left_clear) dirs.push_back(2);
            if (right_clear) dirs.push_back(3);

            choice = dirs.at(random(0, dirs.size()));

            switch (choice) {
                case 0:
                    left_steps = steps;
                    right_steps = steps;
                    break;
                case 1:
                    left_steps = -steps;
                    right_steps = -steps;
                    break;
                case 2:
                    left_steps = -steps;
                    right_steps = steps;
                    break;
                case 3:
                    left_steps = steps;
                    right_steps = -steps;
                    break;
                default: break;
            }*/

            left_sensor_distance = tofSensorDistance(TOF3_ADDR);
            right_sensor_distance = tofSensorDistance(TOF2_ADDR);

            driver.update();

            left_steps = driver.left_steps;
            right_steps = driver.right_steps;

            // check if driver wants to go left
            if (left_steps < right_steps) {
                if (left_sensor_distance != OUT_OF_RANGE && left_sensor_distance <= TOO_CLOSE) {
                    left_steps *= -1;
                }
            // check if the driver wants to go right
            } else if (left_steps > right_steps) {
                if (right_sensor_distance != OUT_OF_RANGE && right_sensor_distance <= TOO_CLOSE) {
                    right_steps *= -1;
                }
            }

            moveSteppers(left_steps, right_steps);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            
            break;
        case 0xFF:
            STATE = STATES::FINISH;
            //Serial.println("Received 0xFF");
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            
            break;
        case 0xAA:
            //Serial.println("Received 0xAA");

            // capture t2
            t2 = 0;
            uint8_t ts2[5];
            dwt_readrxtimestamp(ts2);
            memcpy(&t2, &ts2[0], 5);

            // respond and capture t3
            data[0] = 0x03;
            if (!send(TRANSMITTER_ADDRESS, data, 1)) {
                //Serial.println("Failed to send 0x03");
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                return;
            }

            t3 = 0;
            uint8_t ts3[5];
            dwt_readtxtimestamp(ts3);
            memcpy(&t3, &ts3[0], 5);

            break;
        case 0x15:
            if (rx_len < 20) {
                //Serial.println("Got 0x15 as a header but rx_len < 20");               
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                return;
            }

            //Serial.println("Received 0x15");

            t1 = 0;
            t4 = 0;
            t5 = 0;
            memcpy(&t1, &rx_data[5], 5);
            memcpy(&t4, &rx_data[10], 5);
            memcpy(&t5, &rx_data[15], 5);

            t6 = 0;
            // capture t6
            uint8_t ts6[5];
            dwt_readrxtimestamp(ts6);
            memcpy(&t6, &ts6[0], 5);

            distance = calculateDistance();

            // shift every input up by one and put distance into x[0]
            for (int i = LENGTH - 1; i > 0; i--) {
                x[i] = x[i - 1];
            }
            x[0] = distance;

            distance = convolve(h, x);
            
            Serial.printf("Distance: %.2fcm\n", distance);

            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            break;
        default:
            //Serial.println("Did not get a header matching any valid options");
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

bool send(uint32_t dest_address, uint8_t data[], int data_size) {
    dwt_forcetrxoff(); // force into idle state. this is to prevent attempted transmitting while the receiver is enabled

    uint8_t tx_packet[4 + data_size];
    memcpy(&tx_packet[0], &dest_address, 4);

    for (int i = 0; i < data_size; i++) {
        tx_packet[4 + i] = data[i];
    }

    dwt_writetxdata(sizeof(tx_packet), tx_packet, 0);
    dwt_writetxfctrl(sizeof(tx_packet) + 2, 0, 0);

    bool complete = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS;

    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));

    delayMicroseconds(2); // just making sure that the dwm has enough time to re-enable it's receiver before I clear the bit
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

    return complete;
}

double calculateDistance() {
    int64_t round_trip_d1 = (int64_t) ((t4 - t1) & MASK_40BIT); 
    int64_t reply_d1 = (int64_t) ((t3 - t2) & MASK_40BIT);
    
    int64_t round_trip_d2 = (int64_t) ((t6 - t3) & MASK_40BIT);
    int64_t reply_d2 = (int64_t) ((t5 - t4) & MASK_40BIT);

    int64_t tof_num = round_trip_d1 * round_trip_d2 - reply_d1 * reply_d2;
    int64_t tof_denom = round_trip_d1 + round_trip_d2 + reply_d1 + reply_d2;

    double tof = (double) tof_num / (double) tof_denom;
    double tof_seconds = tof * DWT_TIME_UNITS;

    return tof_seconds * SPEED_OF_LIGHT * 100;
}

// moves both steppers (not just a scheduler)
// put steps negative to be backwards, postive for forwards
void moveSteppers(int left_steps, int right_steps) {
    Serial.printf("left_steps: %d, right_steps: %d\n", left_steps, right_steps);

    int left_dir = left_steps < 0 ? LEFT_STEPPER_BACKWARD : LEFT_STEPPER_FORWARD;
    int right_dir = right_steps < 0 ? RIGHT_STEPPER_BACKWARD : RIGHT_STEPPER_FORWARD;

    leftStepper.moveStepper(left_dir, abs(left_steps));        
    rightStepper.moveStepper(right_dir, abs(right_steps));  

    while (leftStepper.isBusy() || rightStepper.isBusy()) {
        leftStepper.stepperLoop();
        rightStepper.stepperLoop();
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

double convolve(float h[], double x[]) {
    double y = 0;

    for (int i = 0; i <= LENGTH - 1; i++) {
        y += x[i]*h[i]; // x is already shifted by the fact that we put new data into the array every cycle
    }

    return y;
}

void resetDWM() {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(10);
    digitalWrite(PIN_RST, HIGH);
    delay(20);
}


enum TofState { IDLE, MEASURING };
TofState tof_state = IDLE;
unsigned long tof_timer = 0;
float left_sensor_distance, right_sensor_distance;

void serviceSensorsNonBlocking() {
    switch (tof_state) {
        case IDLE:
            // 1. Kick off measurement without waiting in a while-loop
            tof2.startRangeContinuous();
            tof3.startRangeContinuous();
            
            tof_timer = millis();
            tof_state = MEASURING;
            break;

        case MEASURING:
            // 2. Allow RP2040 CPU to process UWB interrupts for 35ms
            if (millis() - tof_timer >= 35) {
                // 3. Read results non-blockingly
                if (tof3.isRangeComplete()) {
                    left_sensor_distance = tofSensorDistance(TOF3_ADDR);
                }
                if (tof2.isRangeComplete()) {
                    right_sensor_distance = tofSensorDistance(TOF2_ADDR);
                }

                // 4. Put sensors back into idle state
                tof2.stopRangeContinuous();
                tof3.stopRangeContinuous();
                
                tof_state = IDLE;
            }
            break;
    }
}
