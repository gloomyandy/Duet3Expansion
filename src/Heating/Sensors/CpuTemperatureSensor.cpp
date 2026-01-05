/*
 * CpuTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "CpuTemperatureSensor.h"
#include <Platform/Platform.h>

#if HAS_CPU_TEMP_SENSOR

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor CpuTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new CpuTemperatureSensor(sensorNum); } );

CpuTemperatureSensor::CpuTemperatureSensor(unsigned int sensorNum) noexcept : TemperatureSensor(sensorNum, "MCU embedded temperature sensor")
{
}

void CpuTemperatureSensor::Poll() noexcept
{
	SetResult(Platform::GetMcuTemperatures().current, TemperatureError::ok);
}

#endif

// End
