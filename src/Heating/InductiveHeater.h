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
	void Init() noexcept;

private:
	static constexpr uint32_t ResonantFrequency = 120'000;		//TODO set the correct value here
};

#endif

#endif /* SRC_HEATING_INDUCTIVEHEATER_H_ */
