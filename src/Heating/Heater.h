/*
 * Heater.h
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_HEATER_H_
#define SRC_HEATING_HEATER_H_

#include <RepRapFirmware.h>
#include "HeaterMonitor.h"
#include "FOPDT.h"

#include <CanId.h>

class HeaterMonitor;
class CanMessageGenericParser;
class CanMessageSetHeaterTemperature;
class CanMessageHeaterModelV2;
class CanMessageSetHeaterMonitors;
class CanMessageHeaterTuningCommand;
class CanMessageSetHeaterFaultDetectionParameters;
class CanMessageHeaterFeedForwardV1;

class Heater
{
public:
	Heater(unsigned int num) noexcept;
	virtual ~Heater();

	// Configuration methods
	virtual GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) noexcept = 0;
	virtual GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept = 0;
	virtual GCodeResult ReportDetails(const StringRef& reply) const noexcept = 0;

	virtual float GetTemperature() const noexcept = 0;					// Get the current temperature
	virtual float GetAveragePWM() const noexcept = 0;					// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	virtual void ResetFault() noexcept = 0;								// Reset a fault condition - only call this if you know what you are doing
	virtual void SwitchOff() noexcept;
	virtual void Spin() noexcept = 0;
	virtual void Suspend(bool sus) noexcept = 0;						// Suspend the heater to conserve power or while doing Z probing
	virtual float GetAccumulator() const noexcept = 0;					// get the inertial term accumulator
	virtual GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept = 0;
	virtual GCodeResult ApplyFeedForward(const CanMessageHeaterFeedForwardV1& msg, const StringRef& reply) noexcept = 0;

	GCodeResult SetTemperature(const CanMessageSetHeaterTemperature& msg, const StringRef& reply) noexcept;

	unsigned int GetHeaterNumber() const noexcept { return heaterNumber; }

	GCodeResult SetFaultDetectionParameters(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply) noexcept;
	GCodeResult SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply) noexcept;

	void SetHeaterMonitoring(HeaterMonitor *h) noexcept;

	const FopDt& GetModel() const noexcept { return model; }			// Get the process model
	GCodeResult SetModel(const CanMessageHeaterModelV2& msg, const StringRef& reply) noexcept;

	bool IsHeaterEnabled() const noexcept								// Is this heater enabled?
		{ return model.IsEnabled(); }

	bool IsTuning() const noexcept { return GetMode() >= HeaterMode::firstTuningMode; }
	uint8_t GetModeByte() const noexcept { return (uint8_t)GetMode(); }

protected:
	virtual void ResetHeater() noexcept;
	virtual HeaterMode GetMode() const noexcept = 0;
	virtual GCodeResult SwitchOn(const StringRef& reply) noexcept = 0;
	virtual GCodeResult UpdateModel(const StringRef& reply) noexcept = 0;

	int GetSensorNumber() const noexcept { return sensorNumber; }
	void SetSensorNumber(int sn) noexcept { sensorNumber = sn; }
	float GetMaxTemperatureExcursion() const noexcept { return maxTempExcursion; }
	float GetMaxHeatingFaultTime() const noexcept { return maxHeatingFaultTime; }
	uint32_t GetMaxBadTemperatureCount() const noexcept { return maxBadTemperatureCount; }
	float GetTargetTemperature() const noexcept { return requestedTemperature; }
	bool IsBedOrChamber() const noexcept { return isBedOrChamber; }
	float GetHighestTemperatureLimit() const noexcept;

	HeaterMonitor monitors[MaxMonitorsPerHeater];	// embedding them in the Heater uses less memory than dynamic allocation
	volatile float lastFanPwm;						// The fan PWM when we last calculated heater feedforward for the fan
	volatile float lastExtrusionPwmBoost;			// The last value of extrusion boost we applied
	volatile float extrusionTemperatureBoost;		// the amount of extrusion temperature boost we are currently applying

private:
	FopDt model;

	unsigned int heaterNumber;
	int sensorNumber;								// the sensor number used by this heater
	float requestedTemperature;						// the required temperature
	float maxTempExcursion;							// the maximum temperature excursion permitted while maintaining the setpoint
	float maxHeatingFaultTime;						// how long a heater fault is permitted to persist before a heater fault is raised
	uint32_t maxBadTemperatureCount;				// the number of consecutive bad sensor readings we allow before raising a fault
	bool isBedOrChamber;							// true if this was a bed or chamber heater when it was switched on
};

#endif /* SRC_HEATING_HEATER_H_ */
