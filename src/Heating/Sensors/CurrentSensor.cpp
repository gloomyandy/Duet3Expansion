/*
 * HeaterCurrentSensor.cpp
 *
 *  Created on: 5 Jan 2026
 *      Author: David
 */

#include "CurrentSensor.h"

#if NUM_CURRENT_SENSORS != 0

#include <Platform/Platform.h>

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor CurrentSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new CurrentSensor(sensorNum); } );

CurrentSensor::CurrentSensor(unsigned int sensorNum) noexcept : TemperatureSensor(sensorNum, "Current sensor")
{
}

void CurrentSensor::Poll() noexcept
{
	SetResult(Platform::GetCurrentSensorReading(currentSensorNumber), TemperatureError::ok);
}

#endif
