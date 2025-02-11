
#include "AM2315C.h"

// Set the default I2C address
const uint8_t AM2315C_ADDRESS = 0x38;

// Constructor for AM2315C_Sensor class
AM2315C_Sensor::AM2315C_Sensor(AM2315C* am2315CSensor, int8_t powerPin, uint8_t i2cAddress)
    : Sensor("AM2315C_Sensor", 2, 100, 500, 500, powerPin, -1, i2cAddress),
      _am2315C(am2315CSensor) {}

// Setup method to initialize the sensor
bool AM2315C_Sensor::setup() {
    return _am2315C->begin();
}

// Method to read the temperature or humidity
bool AM2315C_Sensor::addSingleMeasurementResult(int varNum) {
    if (_am2315C->read() == 0) {  // Assuming read() returns 0 on success
        if (varNum == 0) {
            float temp = _am2315C->getTemperature();
            return Sensor::addSingleMeasurementResult(temp);
        } else if (varNum == 1) {
            float hum = _am2315C->getHumidity();
            return Sensor::addSingleMeasurementResult(hum);
        }
    }
    return false;
}
