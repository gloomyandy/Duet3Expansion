/*
 * HeaterCurrentSensor.cpp
 *
 *  Created on: 5 Jan 2026
 *      Author: David
 */

#include "CurrentSensor.h"

#if NUM_CURRENT_SENSORS != 0

#include <Platform/Platform.h>
#include <CanMessageGenericParser.h>

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor CurrentSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new CurrentSensor(sensorNum); } );

CurrentSensor::CurrentSensor(unsigned int sensorNum) noexcept : TemperatureSensor(sensorNum, TypeName)
{
}

GCodeResult CurrentSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	if (parser.HasParameter('Y'))
	{
		// This is a new sensor so we need to configure it
		String<StringLength20> sensorSource;
		if (parser.GetStringParam('P', sensorSource.GetRef()))
		{
			for (size_t i = 0; i < NUM_CURRENT_SENSORS; ++i)
			{
				if (ReducedStringEquals(sensorSource.c_str(), Platform::GetCurrentSensorName(i)))
				{
					currentSensorIndex = i;
					return GCodeResult::ok;
				}
			}
			reply.printf("Unknown current sensor source '%s'", sensorSource.c_str());
			return GCodeResult::error;
		}
		else
		{
			reply.printf("Missing parameter 'P'");
			return GCodeResult::error;
		}
	}

	// Else reporting on an existing sensor
	CopyBasicDetails(reply);
	reply.catf(", source '%s'", Platform::GetCurrentSensorName(currentSensorIndex));
	return GCodeResult::ok;
}

void CurrentSensor::Poll() noexcept
{
	SetResult(Platform::GetCurrentSensorReading(currentSensorIndex), TemperatureError::ok);
}

#endif
