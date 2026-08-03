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
	GCodeResult Calibrate(bool start, const StringRef& reply) noexcept;			// start calibrating the heater or check whether calibration has completed

private:
	enum class CalibrationState : uint8_t
	{
		idle = 0, start, calibrating, failed, success
	};

	// Constants to define the oscillator parameters
	static constexpr uint32_t OscMinFirstOnTime = 180;
	static constexpr uint32_t OscDefaultLaterOnTime = 300;
	static constexpr uint32_t OscMaxOnTime = 500;
	static constexpr uint32_t OscDefaultOffTime = 600;
	static constexpr uint32_t OscOnTimeStep = 2;
	static constexpr uint32_t StartingOffTime = 840;							// long enough to always contain a full half cycle

	static constexpr uint32_t PwmFrequencyDivisor = 512;						// high enough for good resolution, low enough for fast response (PWM frequency = 120000/512 = 234Hz)

	// Calibration task
	static constexpr unsigned int CalibrationTaskStackWords = 300;				// Calibration task stack size
	static Task<CalibrationTaskStackWords> *_ecv_null calibrationTask;

	[[noreturn]] static void CalibrationTaskEntry(void *pv) noexcept;
	[[noreturn]] void CalibrationTaskFunc() noexcept;							// function executed by the calibration task

	void SetupOscillator(uint32_t pwmOnCount) noexcept;							// set up the timing parameters from firstCycleLength, laterCycleLength and offTime
	void CalibrateHeater() noexcept;
	void SetBurst(uint32_t burstLength) noexcept;
	void TurnOff() noexcept;

	// Parameters to set the oscillator on- and off-times
	uint32_t firstOnTime = OscMinFirstOnTime;
	uint32_t mainOnTime = OscDefaultLaterOnTime;
	uint32_t offTime = OscDefaultOffTime;
	uint32_t pwmTimerPeriod;

	// Calibration variables
	volatile CalibrationState calState = CalibrationState::idle;
	const char *volatile calibrationFailedReason = "calibration not run";
};

#endif

#endif /* SRC_PLATFORM_INDUCTIVEHEATERPORT_H_ */
