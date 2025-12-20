/*
 * TPiS_1T_1086_L5_5.cpp
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#include "TPiS_1T_1086_L5_5.h"

#if SUPPORT_TPiS_1T_1086_L5_5

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor TPiS_1T_1086_L5_5::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TPiS_1T_1086_L5_5(sensorNum); } );

TPiS_1T_1086_L5_5::TPiS_1T_1086_L5_5(unsigned int sensorNum)
	: I2CTemperatureSensor(sensorNum, "Thermopile (TPiS_1T_1086_L5_5)", TPiS_I2CChannel, TPiS_I2CAddress)
{
}

// Configure this temperature sensor
GCodeResult TPiS_1T_1086_L5_5::Configure(const CanMessageGenericParser& parser, const StringRef& reply)
{
	bool seen = false;
	if (false)			//TODO call the configuration function
	{
		return GCodeResult::error;
	}

	if (seen)
	{
		// Initialise the sensor
		//TODO
	}
	else
	{
		CopyBasicDetails(reply);
	}
	return GCodeResult::ok;
}

void TPiS_1T_1086_L5_5::Poll()
{
	//TODO
}

#endif

// End

