#include <Arduino.h>
#include <SPI.h>

#include "Adafruit_VL53L0X.h"
#include "dw3000.h"

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

#define POLL_TX_TO_RESP_RX_DLY_UUS 600

// spi definitions
#define SCK 2
#define MISO 0
#define MOSI 3
#define CS 1
#define SPI_CLK_SPEED 2000000

// i2c definitions
#define SCL 15
#define SDA 14
#define TOF1_ADDR  0x30
#define TOF2_ADDR  0x31

// tof definitions
#define TOF1_XSHUT 11 // front
#define TOF2_XSHUT 10 // right
#define TOF3_XSHUT 9  // left

#define TOF1_OFFSET 0 //20
#define TOF2_OFFSET 29
#define TOF3_OFFSET 52

#define OUT_OF_RANGE -20.0f

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

Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();

float tofSensorDistance(uint8_t address);
void resetDWM();

void dwm3000_isr() {
  uwb_irq = true;
}

void setup() {
    // serial monitor initialization
    Serial.begin(115200);

    uint32_t start = millis();
    while (!Serial && (millis() - start < 3000));
    delay(5);

    Serial.println("Hello World.");

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

    digitalWrite(TOF2_XSHUT, 1);
    delay(50);
    
    if (!tof2.begin(TOF2_ADDR, false, &Wire1)) {
        Serial.println(F("Failed to boot Sensor 2"));
        while (1);
    }
    tof2.setMeasurementTimingBudgetMicroSeconds(33000);

    Serial.println("Booted sensors ready!");

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    Serial.println("All initialization complete");
}

void loop() {
    Serial.printf("Distance: %f mm\n", tofSensorDistance(TOF2_ADDR));
}

float tofSensorDistance(uint8_t address) {
    VL53L0X_RangingMeasurementData_t measure;
    uint16_t offset = 0;
    bool applyAngleCorrection = false;

    switch (address) {
        case TOF2_ADDR:
            tof2.rangingTest(&measure, false);
            offset = TOF2_OFFSET;
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

        //Serial.printf("Distance: %d mm\n", final_calculated_dist);
        return final_calculated_dist;
    } else {
        Serial.print("Out of range\n");
        return OUT_OF_RANGE;
    }
}

void resetDWM() {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(10);
    digitalWrite(PIN_RST, HIGH);
    delay(20);
}