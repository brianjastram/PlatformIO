/**
 * @file AOSongAM2315.h
 * @copyright 2017-2022 Stroud Water Research Center
 * Part of the EnviroDIY ModularSensors library for Arduino
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 *
 * @brief Contains the AOSongAM2315 sensor subclass and the variable subclasses
 * AOSongAM2315_Humidity and AOSongAM2315_Temp.
 *
 * These are used for the AOSong AM2315 capacitive humidity and temperature
 * sensor.
 *
 * This file is dependent on the Adafruit AM2315 Library.
 */
/* clang-format off */
/**
 * @defgroup sensor_am2315 AOSong AM2315
 * Classes for the AOSong AM2315 encased I2C capacitive humidity and
 * temperature sensor.
 *
 * @ingroup the_sensors
 *
 * @tableofcontents
 * @m_footernavigation
 *
 * @section sensor_am2315_notes Quick Notes
 * - Applies to both the AOSong AM2315 and CM2311 capacitive relative humidity
 * and temperature sensors
 * - Depends on the [Adafruit AM2315 Library](https://github.com/adafruit/Adafruit_AM2315).
 * - Communicates via I2C
 *   - only one address possible, 0xB8
 * - **Only 1 can be connected to a single I2C bus at a time**
 * - Requires a 3.3 - 5.5V power source
 *
 * @note Software I2C is *not* supported for the AM2315.
 * A secondary hardware I2C on a SAMD board is supported.
 *
 * @section sensor_am2315_datasheet Sensor Datasheet
 * [Datasheet](https://github.com/EnviroDIY/ModularSensors/wiki/Sensor-Datasheets/AOSong-AM2315-Product-Manual.pdf)
 *
 * @section sensor_am2315_ctor Sensor Constructors
 * {{ @ref AOSongAM2315::AOSongAM2315(int8_t, uint8_t) }}
 * {{ @ref AOSongAM2315::AOSongAM2315(TwoWire*, int8_t, uint8_t) }}
 *
 * @section sensor_am2315_examples Example Code
 *
 * The AM2315 is used in the [double logger](@ref double_logger.ino)
 * and @menulink{ao_song_am2315} example
 *
 * @menusnip{ao_song_am2315}
 */
/* clang-format on */

// Header Guards
#ifndef SRC_SENSORS_AOSONGAM2315_H_
#define SRC_SENSORS_AOSONGAM2315_H_

#ifndef AOSongAM2315_H
#define AOSongAM2315_H

// Debugging Statement
// #define MS_AOSONGAM2315_DEBUG

#ifdef MS_AOSONGAM2315_DEBUG
#define MS_DEBUGGING_STD "AOSongAM2315"
#endif

// Included Dependencies
#include "ModSensorDebugger.h"
#undef MS_DEBUGGING_STD
#include "VariableBase.h"
#include "SensorBase.h"
#include <Adafruit_AM2315.h>
#include "AM2315C.h"
#include <Adafruit_I2CDevice.h>

/** @ingroup sensor_am2315 */
/**@{*/

// Sensor Specific Defines
/// @brief Sensor::_numReturnedValues; the AM2315 can report 2 values.
#define AM2315_NUM_VARIABLES 2
/// @brief Sensor::_incCalcValues; we don't calculate any additional values.
#define AM2315_INC_CALC_VARIABLES 0

/**
 * @anchor sensor_am2315_timing
 * @name Sensor Timing
 * The sensor timing for an AOSong AM2315
 */
/**@{*/
/// @brief Sensor::_warmUpTime_ms; the AM2315 warms up in 500ms (estimated).
#define AM2315_WARM_UP_TIME_MS 500
/// @brief Sensor::_stabilizationTime_ms; the AM2315 is stable after 500ms
/// (estimated).
#define AM2315_STABILIZATION_TIME_MS 500
/// @brief Sensor::_measurementTime_ms; the AM2315 takes 2000ms (2s) to complete
/// a measurement.
#define AM2315_MEASUREMENT_TIME_MS 2000
/**@}*/

/**
 * @anchor sensor_am2315_humidity
 * @name Humidity
 * The humidity variable from an AOSong AM2315
 * - Range is 0 to 100% RH
 * - Accuracy is ± 2 % RH at 25°C
 *
 * {{ @ref AOSongAM2315_Humidity::AOSongAM2315_Humidity }}
 */
/**@{*/
/// @brief Decimals places in string representation; humidity should have 1 (0.1
/// % RH for the 16 bit sensor).
#define AM2315_HUMIDITY_RESOLUTION 1
/// @brief Sensor variable number; humidity is stored in sensorValues[0].
#define AM2315_HUMIDITY_VAR_NUM 0
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "relativeHumidity"
#define AM2315_HUMIDITY_VAR_NAME "relativeHumidity"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/); "percent"
/// (percent relative humidity)
#define AM2315_HUMIDITY_UNIT_NAME "percent"
/// @brief Default variable short code; "AM2315Humidity"
#define AM2315_HUMIDITY_DEFAULT_CODE "AM2315Humidity"
/**@}*/

/**
 * @anchor sensor_am2315_temperature
 * @name Temperature
 * The temperature variable from an AOSong AM2315
 * - Range is -40°C to +125°C
 * - Accuracy is ±0.1°C
 *
 * {{ @ref AOSongAM2315_Temp::AOSongAM2315_Temp }}
 */
/**@{*/
/// @brief Decimals places in string representation; temperature should have 1.
/// (0.1°C for the 16 bit sensor)
#define AM2315_TEMP_RESOLUTION 1
/// @brief Sensor variable number; temperature is stored in sensorValues[1].
#define AM2315_TEMP_VAR_NUM 1
/// @brief Variable name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/variablename/);
/// "temperature"
#define AM2315_TEMP_VAR_NAME "temperature"
/// @brief Variable unit name in
/// [ODM2 controlled vocabulary](http://vocabulary.odm2.org/units/);
/// "degreeCelsius" (°C)
#define AM2315_TEMP_UNIT_NAME "degreeCelsius"
/// @brief Default variable short code; "AM2315Temp"
#define AM2315_TEMP_DEFAULT_CODE "AM2315Temp"

/**
 * @brief The Variable sub-class used for the
 * [relative humidity output](@ref sensor_am2315_humidity) from an
 * [AOSong AM2315](@ref sensor_am2315).
 */
/* clang-format on */
// AM2315C Sensor Class
class AM2315C_Sensor : public Sensor {
public:
    AM2315C_Sensor(TwoWire* wire, int8_t powerPin, uint8_t i2cAddress = AM2315_I2CADDR);
    bool setup();
    bool addSingleMeasurementResult();

private:
    Adafruit_AM2315 _am2315;
};

// Constructor for AM2315C_Sensor
AM2315C_Sensor::AM2315C_Sensor(TwoWire* wire, int8_t powerPin, uint8_t i2cAddress)
    : Sensor("AM2315C", 2,  // Sensor name and number of variables
             100, 500, 500, // Warm-up, stabilization, and measurement times in milliseconds
             powerPin, -1, i2cAddress),
      _am2315(wire) {}

bool AM2315C_Sensor::setup() {
    if (!_am2315.begin()) {
        Serial.println(F("AM2315C failed to initialize at I2C address 0x38. Check connections."));
        return false;
    }
    Serial.println("AM2315C sensor initialized successfully at I2C address 0x38!");
    return true;
}

bool AM2315C_Sensor::addSingleMeasurementResult() {
    float humidity, temperature;
    bool success = _am2315.readTemperatureAndHumidity(&temperature, &humidity);

    if (success) {
        sensorValues[0] = humidity;
        sensorValues[1] = temperature;
    }

    return success;
}

// AM2315C Humidity Variable Class
class AM2315C_Humidity : public Variable {
public:
  AM2315C_Humidity(AM2315C_Sensor* parentSense, const char* uuid = "",
                   const char* varCode = AM2315_HUMIDITY_DEFAULT_CODE)
    : Variable(parentSense, AM2315_HUMIDITY_VAR_NUM,
               AM2315_HUMIDITY_RESOLUTION,
               AM2315_HUMIDITY_VAR_NAME, AM2315_HUMIDITY_UNIT_NAME,
               varCode, uuid) {}
};

// AM2315C Temperature Variable Class
class AOSongAM2315_Temp : public Variable {
public:
  AOSongAM2315_Temp(AM2315C_Sensor* parentSense, const char* uuid = "",
                    const char* varCode = AM2315_TEMP_DEFAULT_CODE)
    : Variable(parentSense, AM2315_TEMP_VAR_NUM,
               AM2315_TEMP_RESOLUTION,
               AM2315_TEMP_VAR_NAME, AM2315_TEMP_UNIT_NAME,
               varCode, uuid) {}
};

#endif // AOSongAM2315_H
#endif // SRC_SENSORS_AOSONGAM2315_H_