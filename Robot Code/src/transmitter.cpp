#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

#include "dw3000.h"

extern "C" {
    void dwt_readsystime(uint8_t *timestamp);
}


// define statements
// on board led
#define ON_BOARD_LED 16
#define NUM_PIXELS   1

// joystick definitions
#define JOY_Y  29
#define JOY_X  28
#define JOY_SW 27 // give this an internal pull-up resistor

#define UPPER_ADC_VALUE 260  // adjusted for resting_pos of 540
#define LOWER_ADC_VALUE -240 // adjusted for resting_pos of 540
#define JOY_DEADZONE    50
#define RESTING_POS     540 

#define MAX_STEPS       100
#define MIN_STEPS       10

// dwm3000 definitions
#define PIN_IRQ 6
#define PIN_RST 7
#define PIN_WAKE 8

#define DWM_ID 0xDECA0302

#define RECEIVER_ADDRESS    0x00000001
#define TRANSMITTER_ADDRESS 0x00000002

#define TX_ANT_DLY 16350
#define RX_ANT_DLY 16350

// #define TX_ANT_DLY 16385
// #define RX_ANT_DLY 16385

#define POLL_TX_TO_RESP_RX_DLY_UUS 600
#define RESP_RX_TIMEOUT_UUS 4000

// spi definitions
#define SCK  2
#define MISO 0
#define MOSI 3
#define CS   1
#define SPI_CLK_SPEED 2000000

#define AUTO 13


// function definitions
void resetDWM();
void checkData();
bool send(uint32_t dest_address, uint8_t  data[], int data_size, bool expect_response = true);
void manualControl();

// global variables
uint64_t t1;
uint64_t t4;
uint64_t t5;

bool initiated = false;
volatile bool uwb_irq = false;

Adafruit_NeoPixel pixel(NUM_PIXELS, ON_BOARD_LED, NEO_GRB + NEO_KHZ800);

extern dwt_txconfig_t txconfig_options;

dwt_config_t config =  {
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


// function definitions
void dwm3000_isr() {
  uwb_irq = true;
}

void setup() {
    // serial monitor initialization
    Serial.begin(115200);

    uint32_t start = millis();
    while (!Serial && (millis() - start < 3000));

    // spi initialization
    SPI.setSCK(SCK);
    SPI.setTX(MOSI);
    SPI.setRX(MISO);

    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);

    // joystick initialization
    pinMode(JOY_X, INPUT);
    pinMode(JOY_Y, INPUT);
    pinMode(JOY_SW, INPUT_PULLUP);

    // auto button initialization
    pinMode(AUTO, INPUT_PULLDOWN);

    // neo-pixel initialization
    pixel.begin();

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
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    // setup interrupts
    dwt_setinterrupt(DWT_INT_RX, 0, DWT_ENABLE_INT_ONLY);

    // note that the pin_irq is already set up as an input in spiBegin()
    attachInterrupt(digitalPinToInterrupt(PIN_IRQ), dwm3000_isr, RISING);

    Serial.println("All initialization complete (transmitter)");
}

void loop() {
    if (!digitalRead(AUTO) && initiated) {
        uint8_t data[1] = {0xFF};
        send(RECEIVER_ADDRESS, data, 1, false);

        pixel.setPixelColor(0, pixel.Color(0, 0, 0));
        pixel.show();

        Serial.println("Sent 0xFF");

        initiated = false;
    } else if (digitalRead(AUTO) && !initiated) {
        uint8_t data[1] = {0xAA};
        if (send(RECEIVER_ADDRESS, data, 1)) {
            initiated = true;

            t1 = 0;
            // capture t1
            uint8_t ts1[5];
            dwt_readtxtimestamp(ts1);
            memcpy(&t1, &ts1[0], 5);

            //Serial.println("Sent 0xAA");
        } else {
            //Serial.println("Failed to send 0xAA");
        }
    }

    // auto mode
    if (digitalRead(AUTO) && uwb_irq && initiated) {
        uwb_irq = false;

        // this checks if bit 17 is set in SYS_STATUS register
        // bit 17 marks if the interrupt event was caused by receive wait timeout
        // if this receiver timed out it means we didn't get a packet from the robot
        // so we will need to send 0xAA again to start up the loop again
        // the result of this should appear as a seamless loop to the user
        uint32_t sys_status = dwt_read32bitreg(SYS_STATUS_ID);

        if (sys_status & SYS_STATUS_RXFTO_BIT_MASK) {
            dwt_write32bitreg(SYS_STATUS_ID, sys_status & SYS_STATUS_RXFTO_BIT_MASK);
            initiated = false;
        } else checkData();

    // manual mode
    } else if (!digitalRead(AUTO)) {
        manualControl();
    }
}

void checkData() {
    //Serial.println("Checking Data");

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

    if (address != TRANSMITTER_ADDRESS) {
        //Serial.printf("Address is not that of the transmitter");
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
    
    uint8_t data[1];
    uint32_t sys_time, delayed_time;
    uint32_t dest_address;

    switch (rx_data[4]) {
        case 0x03:
            // capture t4 and respond
            t4 = 0;
            uint8_t ts4[5];
            dwt_readrxtimestamp(ts4);
            memcpy(&t4, &ts4[0], 5);

            //Serial.println("Received 0x03");

            // capture t5 and respond with t1 t4 and t5
            dwt_readsystime((uint8_t*) &sys_time);
        
            delayed_time = sys_time + (uint32_t) (((uint64_t) UUS_TO_DWT_TIME * (2000)) >> 8); // shift right by 8 to convert to 4ns time

            dwt_setdelayedtrxtime(delayed_time);
            
            t5 = (((uint64_t) delayed_time) << 8) + TX_ANT_DLY;

            uint8_t final_payload[16];
            final_payload[0] = 0x15;
            memcpy(&final_payload[1], &t1, 5);
            memcpy(&final_payload[6], &t4, 5);
            memcpy(&final_payload[11], &t5, 5);

            uint8_t tx_packet[4 + sizeof(final_payload)];
            dest_address = RECEIVER_ADDRESS;
            memcpy(&tx_packet[0], &dest_address, 4);
            memcpy(&tx_packet[4], final_payload, sizeof(final_payload));

            dwt_writetxdata(sizeof(tx_packet), tx_packet, 0);
            dwt_writetxfctrl(sizeof(tx_packet) + 2, 0, 0);

            initiated = false; // send 0xAA again on next loop iteration

            if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
                //Serial.println("Delayed TX failed (timing window missed)");
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                delay(2);
                return;
            }

            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));

            delayMicroseconds(2);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

            delay(2);

            break;
        default:
            //Serial.println("Did not get a header matching any valid options");
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

bool send(uint32_t dest_address, uint8_t data[], int data_size, bool expect_response) {
    dwt_forcetrxoff(); // force into idle state. this is to prevent attempted transmitting while the receiver is enabled

    uint8_t tx_packet[4 + data_size];
    memcpy(&tx_packet[0], &dest_address, 4);

    for (int i = 0; i < data_size; i++) {
        tx_packet[4 + i] = data[i];
    }

    dwt_writetxdata(sizeof(tx_packet), tx_packet, 0);
    dwt_writetxfctrl(sizeof(tx_packet) + 2, 0, 0);

    bool complete;
    if (expect_response) complete = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS;
    else complete = dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS;

    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK));

    delayMicroseconds(2); // just making sure that the dwm has enough time to re-enable it's receiver before I clear the bit
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    
    return complete;
}

void manualControl() {
    int xVal = analogRead(JOY_X) - RESTING_POS;
    int yVal = analogRead(JOY_Y) - RESTING_POS;

    xVal = abs(xVal) <= JOY_DEADZONE ? 0 : xVal;
    yVal = abs(yVal) <= JOY_DEADZONE ? 0 : yVal;
    
    if (xVal == 0 && yVal == 0) return;

    bool turning = abs(xVal) > abs(yVal);

    uint8_t data[3]; // first bit -> manual header, second bit -> direction, third bit -> steps
    data[0] = 0xAB;

    int steps = 0;

    if (turning) {
        if (xVal > 0) data[1] = 0x10; // left
        else data[1] = 0x11; // right

        steps = map(abs(xVal), LOWER_ADC_VALUE, UPPER_ADC_VALUE, MIN_STEPS, MAX_STEPS);
    } else {
        if (yVal > 0) data[1] = 0x00; // forward
        else data[1] = 0x01; // backward

        steps = map(abs(yVal), LOWER_ADC_VALUE, UPPER_ADC_VALUE, MIN_STEPS, MAX_STEPS);
    }

    memcpy(&data[2], &steps, 1);

    send(RECEIVER_ADDRESS, data, 3, false);
}

void resetDWM() {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(10);
    digitalWrite(PIN_RST, HIGH);
    delay(20);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the system time
 *
 * input parameters
 * @param timestamp - a pointer to a 4-byte buffer which will store the read system time
 *
 * output parameters
 * @param timestamp - the timestamp buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readsystime(uint8_t * timestamp)
{
    dwt_readfromdevice(SYS_TIME_ID, 0, SYS_TIME_LEN, timestamp);
}