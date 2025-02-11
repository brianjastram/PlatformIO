/**
 * @file AOSongAM2315.cpp
 * @copyright 2017-2022 Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library for Arduino
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Implements the AOSongAM2315 class.
 */

#include <AM2315C.h>
//#include "AOSongAM2315.h"


// The constructor - because this is I2C, only need the power pin
// This sensor has a set I2C address of 0XB8
AOSongAM2315::AOSongAM2315(TwoWire* theI2C, int8_t powerPin,
                                                     uint8_t measurementsToAverage)
        : Sensor("AOSongAM2315", AM2315_NUM_VARIABLES, AM2315_WARM_UP_TIME_MS,
                         AM2315_STABILIZATION_TIME_MS, AM2315_MEASUREMENT_TIME_MS, powerPin,
                         -1, measurementsToAverage),
            _i2c(theI2C) {
        am2315ptr = new AM2315C(_i2c);
}
AOSongAM2315::AOSongAM2315(int8_t powerPin, uint8_t measurementsToAverage)
        : Sensor("AOSongAM2315", AM2315_NUM_VARIABLES, AM2315_WARM_UP_TIME_MS,
                         AM2315_STABILIZATION_TIME_MS, AM2315_MEASUREMENT_TIME_MS, powerPin,
                         -1, measurementsToAverage, AM2315_INC_CALC_VARIABLES),
            _i2c(&Wire) {
        am2315ptr = new AM2315C(_i2c);
}
AOSongAM2315::~AOSongAM2315() {}


String AOSongAM2315::getSensorLocation(void) {
    return F("I2C_0xB8");
}


bool AOSongAM2315::begin() {
    if (!am2315ptr->begin()) {
        Serial.println("Could not find a valid AM2315C sensor, check wiring!");
        return false;
    }
    return true;
}


void AOSongAM2315::addSingleMeasurementResult() {
    float temp = am2315c.getTemperature();
    float hum = am2315c.getHumidity();

    if (temp == -9999 || hum == -9999) {
        Serial.println("Failed to read from AM2315C sensor!");
        return;
    }

    verifyAndAddMeasurementResult(AM2315_TEMP_VAR_NUM, temp_val);
    verifyAndAddMeasurementResult(AM2315_HUMIDITY_VAR_NUM, humid_val);

    // Unset the time stamp for the beginning of this measurement
    _millisMeasurementRequested = 0;
    // Unset the status bits for a measurement request (bits 5 & 6)
    _sensorStatus &= 0b10011111;

    return ret_val;
}