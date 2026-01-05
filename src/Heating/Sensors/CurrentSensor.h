/*
 * HeaterCurrentSensor.h
 *
 *  Created on: 5 Jan 2026
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_CURRENTSENSOR_H_
#define SRC_HEATING_SENSORS_CURRENTSENSOR_H_

#include "TemperatureSensor.h"

#if NUM_CURRENT_SENSORS != 0

class CurrentSensor : public TemperatureSensor
{
public:
	CurrentSensor(unsigned int sensorNum) noexcept;

	static constexpr const char *TypeName = "current";

	void Poll() noexcept override;

private:
	static SensorTypeDescriptor typeDescriptor;

	size_t currentSensorNumber;					// which current sensor, must be less than NUM_CURRENT_SENSORS
};

#endif

#endif /* SRC_HEATING_SENSORS_CURRENTSENSOR_H_ */
