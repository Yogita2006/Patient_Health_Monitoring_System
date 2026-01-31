#include <Arduino.h>
#include "DHT11_Sensor.h"

DHT11 dht_sensor(3);

void setup(void) {
    Serial.begin(115200);
}

void loop(void) {

    DHT11::dht_data recvData = dht_sensor.getData();

    Serial.print("exitCode: ");
    Serial.println(recvData.exitCode, HEX);
    Serial.print("Ack: ");
    Serial.println(recvData.ack, HEX);
    Serial.print("temp: ");
    Serial.println(recvData.temp, HEX);
    Serial.print("rh: ");
    Serial.println(recvData.rh, HEX);
    Serial.print("cs: ");
    Serial.println(recvData.checksum, HEX);

    // recommend at least a 1100ms delay between readings ( datasheet says MINIMUM 1 second between readings )
    delay(1100);

}