
#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "Sensor.h"  // Include the ModularSensors Sensor class

#define AM2315C_LIB_VERSION                    (F("0.2.1"))

#define AM2315C_OK                             0
#define AM2315C_ERROR_CHECKSUM                -10
#define AM2315C_ERROR_CONNECT                 -11
#define AM2315C_MISSING_BYTES                 -12
#define AM2315C_ERROR_BYTES_ALL_ZERO          -13
#define AM2315C_ERROR_READ_TIMEOUT            -14
#define AM2315C_ERROR_LASTREAD                -15

class AM2315C_Sensor : public Sensor  // Inherit from Sensor class
{
public:
    AM2315C_Sensor(AM2315C* am2315CSensor, int8_t powerPin = -1, uint8_t i2cAddress = 0x38);  // Constructor
    bool setup() override;  // Override setup function
    bool addSingleMeasurementResult(int varNum) override;  // Implement the reading function

private:
    AM2315C* _am2315C;  // Pointer to the core sensor object
};
