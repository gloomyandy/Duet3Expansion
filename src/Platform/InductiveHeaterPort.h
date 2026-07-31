/*
 * InductiveHeaterPort.h
 *
 *  Created on: 29 Dec 2025
 *      Author: David
 */

#ifndef SRC_PLATFORM_INDUCTIVEHEATERPORT_H_
#define SRC_PLATFORM_INDUCTIVEHEATERPORT_H_

#include <RepRapFirmware.h>

#if SUPPORT_INDUCTIVE_HEATER

#include <RTOSIface/RTOSIface.h>

class InductiveHeaterPort
{
public:
	InductiveHeaterPort() noexcept;
	void Init() noexcept;
	void SetPwm(float pwm) noexcept;											// pwm is a fraction in [0,1]
	GCodeResult StartCalibration(const StringRef& reply) noexcept;				// start calibrating the heater
	GCodeResult CheckCalibrationComplete(const StringRef& reply) noexcept;		// check whether heater calibration is complete
	[[noreturn]] void CalibrationTaskFunc() noexcept;

private:
	enum class CalibrationState : uint8_t
	{
		idle = 0, start, calibrating, complete
	};

	// Constants to define the oscillator parameters
	static constexpr uint32_t OscMinFirstOnTime = 180;
	static constexpr uint32_t OscDefaultLaterOnTime = 400;
	static constexpr uint32_t OscDefaultOffTime = 600;

	static constexpr uint32_t PwmFrequencyDivisor = 512;						// high enough for good resolution, low enough for fast response (PWM frequency = 120000/512 = 234Hz)

	// Heater tuning limits. Values are in clocks of the timer used to generate the FET drive.
	static constexpr uint32_t MinFirstCyceLength = 180;							// safe starting value
	static constexpr uint32_t CycleLengthStep = 2;
	static constexpr uint32_t MaxCycleLength = 1080;
	static constexpr uint32_t StartingOffTime = 840;							// long enough to always contain a full half cycle

	// Calibration task
	static constexpr unsigned int CalibrationTaskStackWords = 300;				// Calibration task stack size
	static Task<CalibrationTaskStackWords> *_ecv_null calibrationTask;

	[[noreturn]] static void CalibrationTaskEntry(void *pv) noexcept;

	void SetupOscillator() noexcept;											// set up the timing parameters from firstCycleLength, laterCycleLength and offTime
	void CalibrateHeater() noexcept;

	// Parameters to set the oscillator on- and off-times
	uint32_t firstOnTime = OscMinFirstOnTime;
	uint32_t mainOnTime = OscDefaultLaterOnTime;
	uint32_t offTime = OscDefaultOffTime;
	uint32_t pwmTimerPeriod;

	// Derived parameters

	volatile CalibrationState calState = CalibrationState::idle;
};

#endif

#endif /* SRC_PLATFORM_INDUCTIVEHEATERPORT_H_ */
