/*
 * ResistiveHeater.h
 *
 *  Created on: 28 Dec 2025
 *      Author: David
 */

#ifndef SRC_HEATING_RESISTIVEHEATER_H_
#define SRC_HEATING_RESISTIVEHEATER_H_

#include "LocalHeater.h"

class ResistiveHeater : public LocalHeater
{
public:
	ResistiveHeater(unsigned int heaterNum) noexcept;
	~ResistiveHeater();

	GCodeResult ReportDetails(const StringRef& reply) const noexcept override;
	GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) noexcept override;
	GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept override;

protected:
	void SetHeater(float power) const noexcept;					// Power is a fraction in [0,1]

private:
	PwmPort ports[MaxPortsPerHeater];							// The port(s) that drive the heater
};

#endif /* SRC_HEATING_RESISTIVEHEATER_H_ */
