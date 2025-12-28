/*
 * ResistiveHeater.cpp
 *
 *  Created on: 28 Dec 2025
 *      Author: David
 */

#include "ResistiveHeater.h"

ResistiveHeater::ResistiveHeater(unsigned int heaterNum) noexcept
	: LocalHeater(heaterNum)
{
	ResistiveHeater::SetHeater(0.0);							// set up the pin even if the heater is not enabled (for PCCB)
}

ResistiveHeater::~ResistiveHeater()
{
	LocalHeater::SwitchOff();
	for (auto& port : ports)
	{
		port.Release();
	}
}

// Configure the heater port and the sensor number
GCodeResult ResistiveHeater::ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) noexcept
{
	if constexpr (MaxPortsPerHeater == 1)
	{
		if (!ports[0].AssignPort(portName, reply, PinUsedBy::heater, PinAccess::pwm))
		{
			return GCodeResult::error;
		}
	}
	else
	{
		PinAccess access[MaxPortsPerHeater];
		IoPort* portAddrs[MaxPortsPerHeater];
		for (size_t i = 0; i < MaxPortsPerHeater; ++i)
		{
			access[i] = PinAccess::pwm;
			portAddrs[i] = &ports[i];
		}
		if (IoPort::AssignPorts(portName, reply, PinUsedBy::heater, MaxPortsPerHeater, portAddrs, access) == 0)
		{
			return GCodeResult::error;
		}
	}

	return ConfigureSensor(sn, reply);
}

GCodeResult ResistiveHeater::ReportDetails(const StringRef& reply) const noexcept
{
	reply.printf("Heater %u pin(s) ", GetHeaterNumber());
	ports[0].AppendPinName(reply);
	if constexpr (MaxPortsPerHeater > 1)
	{
		for (size_t i = 1; i < MaxPortsPerHeater && ports[i].IsValid(); ++i)
		{
			reply.cat('+');
			ports[i].AppendPinName(reply, false);
		}
	}

	ports[0].AppendFrequency(reply);

	return LocalHeater::ReportDetails(reply);
}

GCodeResult ResistiveHeater::SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept
{
	for (auto& port : ports)
	{
		port.SetFrequency(freq);
	}
	return GCodeResult::ok;
}

void ResistiveHeater::SetHeater(float power) const noexcept
{
	for (auto& port : ports)
	{
		port.WriteAnalog(power);
	}
}

// End
