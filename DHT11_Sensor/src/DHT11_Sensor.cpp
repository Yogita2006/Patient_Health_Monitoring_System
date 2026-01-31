#include <Arduino.h>
#include "DHT11_Sensor.h"

// init variables for processing DHT11 peripheral
uint16_t* DHT11::data = &DHT11_data.ack;
unsigned long DHT11::referenceRead = 0; 
unsigned long DHT11::triggerTime = 0;
unsigned long DHT11::deltaTime = 0;
bool DHT11::firstRead = true;
uint16_t DHT11::bitMask = 0x0001;
uint8_t DHT11::dataSelect = ACK;

// init the struct for DHT11 data
DHT11::dht_data DHT11::DHT11_data = {0,0,0,0,0};

// init result flag
volatile uint8_t DHT11::result = DHT11_NOT_INIT;


DHT11::DHT11(int pin) {}


void DHT11::reset(void) {
    // reset all variables back to known states
    detachInterrupt(digitalPinToInterrupt(3));
    delayMicroseconds(250); // allow new GPIO pin change to settle

    result = DHT11_BUSY;
    DHT11_data.exitCode = DHT11_BUSY;
    dataSelect = ACK;
    DHT11_data.temp = 0;
    DHT11_data.rh = 0;
    DHT11_data.checksum = 0;
    data = &DHT11_data.ack;
    firstRead = true;
    bitMask = 0x0001;
}


void DHT11::begin(void) {

    DHT11::reset();

    // Send DHT11 initial ~20ms low pulse (per datasheet) 
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);
    delay(20);
    pinMode(3, INPUT);

    // suppress potential transient response
    delayMicroseconds(40); 

    // attach interrupt to enable DHT11 data reception
    attachInterrupt(digitalPinToInterrupt(3), IRQ_Handler, FALLING);
    
}


inline void DHT11::dhtDataStateMachine(void) {
    // if bitMask is finished, move to next data element
    if(bitMask == 0) {
        // choose new data element
        switch(dataSelect) {
            case ACK:
                // successful ack from DHT11
                if(DHT11_data.ack > 0) { bitMask = 0x8000; data = &DHT11_data.rh; dataSelect = RH;}
                else { result = DHT11_NO_ACK ; dataSelect = FAIL;}
                break;
            case RH:
                bitMask = 0x8000;
                data = &DHT11_data.temp;
                dataSelect = TEMP;
                break;
            case TEMP:
                bitMask = 0x80;
                data = &DHT11_data.checksum;
                dataSelect = CHECKSUM;
                break;
            case CHECKSUM:
                result = (DHT11_data.temp & 0x00FF) + (DHT11_data.temp >> 8) + (DHT11_data.rh & 0x00FF) + (DHT11_data.rh >> 8);
                if(result == DHT11_data.checksum) {result = DHT11_SUCCESS; }
                else { result = DHT11_CHECKSUM_FAILED; dataSelect = FAIL; }
                break;
            case FAIL:
                // force ISR to stall in this condition until timeout is reached
                bitMask = 0x00;
                dataSelect = FAIL;
                break;
            default:
                break;
        } 
    }
}


void DHT11::IRQ_Handler(void) {

    // calculate first reference point
    if(firstRead) {
        referenceRead = micros();
        firstRead = false;
        return;
    }

    // record timestamp of edge detect
    triggerTime = micros();

    // calculate deltaTime in us (and then shift reference)
    deltaTime = triggerTime - referenceRead;
    referenceRead = triggerTime; 
    
    // process data
    if(deltaTime >= LOGIC_LVL_THRESHOLD) *data |= bitMask;
    bitMask >>= 1;

    DHT11::dhtDataStateMachine();

}


dht_error_t DHT11::isAvailable(void) {
    return result;
}


DHT11::dht_data DHT11::getData(void) {

    DHT11::begin();

    volatile unsigned long currTime = millis();
    volatile unsigned long prevTime = millis();

    // wait for error msg, succes msg, or 100ms timeout
    while((DHT11::isAvailable() == DHT11_BUSY) && (currTime - prevTime < TIMEOUT_INTERVAL)) {
        currTime = millis();
    }

    // if timeout condition is met, return TIMEOUT ERROR
    if(currTime - prevTime >= TIMEOUT_INTERVAL) {
        DHT11_data.exitCode = DHT11_TIMEOUT;
        return DHT11_data;
    } 
    else {
        DHT11_data.exitCode = DHT11::result;
        return DHT11_data;
    }  
};