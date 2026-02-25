/*
 * I2CTemperatureSensor.h
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_I2CTEMPERATURESENSOR_H_
#define SRC_HEATING_SENSORS_I2CTEMPERATURESENSOR_H_

#include <RepRapFirmware.h>

#if NUM_I2C_CHANNELS != 0

#include "TemperatureSensor.h"
#include <Hardware/SharedI2CClient.h>

class I2CTemperatureSensor : public TemperatureSensor
{
protected:
	I2CTemperatureSensor(unsigned int sensorNum, const char *name, unsigned int i2cNumber, uint16_t address);

	SharedI2CClient device;
};

#endif

#endif /* SRC_HEATING_SENSORS_I2CTEMPERATURESENSOR_H_ */
