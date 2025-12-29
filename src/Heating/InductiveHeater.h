/*
 * InductiveHeater.h
 *
 *  Created on: 21 Dec 2025
 *      Author: David
 *
 *  Support for inductive heater
 */

#ifndef SRC_HEATING_INDUCTIVEHEATER_H_
#define SRC_HEATING_INDUCTIVEHEATER_H_

#include <RepRapFirmware.h>

#if SUPPORT_INDUCTIVE_HEATER

#include "LocalHeater.h"

class InductiveHeater final : public LocalHeater
{
public:
	InductiveHeater(unsigned int heaterNum) noexcept;

protected:
	void SetHeater(float power) noexcept override;						// Power is a fraction in [0,1]

private:
	static constexpr uint32_t OscResonantFrequency = 120'000;			//TODO set the correct value here
	static constexpr uint32_t PwmFrequencyDivisor = 512;				// high enough for good resolution, low enough for fast response
	static constexpr float OscMarkSpaceRatio = 0.4;						// the mark/space ratio for the heater FET drive

	uint32_t oscTimerPeriod;
	uint32_t pwmTimerPeriod;
};

#endif

#endif /* SRC_HEATING_INDUCTIVEHEATER_H_ */
