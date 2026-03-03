/*
 * I2CTemperatureSensor.cpp
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#include "I2CTemperatureSensor.h"

#if NUM_I2C_CHANNELS != 0

#include <Platform/Platform.h>

I2CTemperatureSensor::I2CTemperatureSensor(unsigned int sensorNum, const char *name, unsigned int i2cNumber, uint16_t address) noexcept
	: TemperatureSensor(sensorNum, name), device(Platform::GetSharedI2C(i2cNumber), address)
{
	//TODO
}

#endif

// End
