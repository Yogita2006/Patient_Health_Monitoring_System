#ifndef DHT11_SENSOR_H_
#define DHT11_SENSOR_H_

// Author: Jonathan Zurita
// Version: 1.0.0
// Any feedback is welcome! :)

const unsigned int LOGIC_LVL_THRESHOLD = 95;  // (in milliseconds)
const unsigned int TIMEOUT_INTERVAL    = 100; // (in milliseconds)
typedef unsigned int dht_error_t;

enum DHT11_STATE_CODES {
    ACK = 0,
    TEMP,
    RH,
    CHECKSUM,
    FAIL
};

enum DHT11_ERROR_CODES {
    DHT11_SUCCESS = 0,
    DHT11_TIMEOUT,
    DHT11_CHECKSUM_FAILED,
    DHT11_NO_ACK,
    DHT11_NOT_INIT,
    DHT11_BUSY
};

class DHT11 {

    public:
        
        typedef struct dht_data {
            uint8_t exitCode;
            uint16_t temp;
            uint16_t rh;
            uint16_t checksum;
            uint16_t ack;
        } dht_data;

        static dht_data DHT11_data;

        DHT11(int pin); // constructor
        inline static void dhtDataStateMachine(void); // state machine to consider next data element to collect
        void begin(void); // start data initiation
        void reset(void); // reset all data variables
        dht_error_t isAvailable(void); // probe if DHT11 is finished 
        dht_data getData(void); // Initiate, parse, and deliver temp/rh data to user

    private:
        
        static void IRQ_Handler(void); // static method for ISR

        // static variables for IRQ to process the DHT11 peripheral
        static uint16_t* data;
        static unsigned long referenceRead;
        static unsigned long triggerTime;
        static unsigned long deltaTime;
        static bool firstRead;
        static uint16_t bitMask;
        static uint8_t dataSelect;
        volatile static uint8_t result;
};


#endif