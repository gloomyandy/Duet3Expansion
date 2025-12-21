/*
 * InductiveHeater.cpp
 *
 *  Created on: 21 Dec 2025
 *      Author: David
 *
 *  Support for the inductive heater in the Bondtech INDX system
 */

#include "InductiveHeater.h"

#if SUPPORT_INDUCTIVE_HEATER

InductiveHeater::InductiveHeater() noexcept
{
	// TODO Auto-generated constructor stub

}

void InductiveHeater::Init() noexcept
{
	// Set up the oscillator TC
	//TODO
	// Set up the PWM TCC
	//TODO
	// Set up the CCL to get them together
	//TODO
	SetPinFunction(InductiveHeaterCCLOutPin, InductiveHeaterCCLOutPinPeriphMode);
}

#endif

// End
