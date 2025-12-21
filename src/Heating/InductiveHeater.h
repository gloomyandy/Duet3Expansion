/*
 * InductiveHeater.h
 *
 *  Created on: 21 Dec 2025
 *      Author: David
 *
 *  Support for the inductive heater in the Bondtech INDX system
 */

#ifndef SRC_HEATING_INDUCTIVEHEATER_H_
#define SRC_HEATING_INDUCTIVEHEATER_H_

#include <RepRapFirmware.h>

#if SUPPORT_INDUCTIVE_HEATER

class InductiveHeater
{
public:
	InductiveHeater() noexcept;
	void Init() noexcept;												// set up the timers etc. and turn the output off
	void SetPwm(float pwm) noexcept;									// set the PWM value in the range 0..1

private:
	static constexpr uint32_t ResonantFrequency = 120'000;				//TODO set the correct value here
	static constexpr uint32_t PwmFrequencyDivisor = 1000;				// high enough for good resolution, low enough for fast response

	uint32_t oscTimerTop;
	uint32_t pwmTimerTop;
};

#endif

#endif /* SRC_HEATING_INDUCTIVEHEATER_H_ */
