/*
 * LP5817.cpp
 *
 *  Created on: 26 Dec 2025
 *      Author: David
 */

#include "LP5817.h"

#if SUPPORT_LP5817

LP5817::LP5817(SharedI2CMaster& dev) noexcept
	: SharedI2CClient(dev, LP5817_I2CAddress)
{
}

bool LP5817::Init(uint8_t inputPins, uint8_t initialOutputs) noexcept
{
	outputRegister = initialOutputs;
	uint8_t val;
	return (   Write8(LP5817_Register::output, outputRegister)				// ensure that the LEDs are off
			&& Write8(LP5817_Register::polarityInversion, 0)				// don't invert anything
			&& Write8(LP5817_Register::config, inputPins)					// configure I/O
			&& Read8(LP5817_Register::input, inputRegister)					// read the initial inputs
			&& Read8(LP5817_Register::config, val)							// check that the config register reads back correctly
			&& val == inputPins
	   	   );
}

bool LP5817::Read8(LP5817_Register reg, uint8_t& val) noexcept
{
	uint8_t data[2];
	data[0] = (uint8_t)reg;
	const bool ok = Transfer(data, data + 1, 1, 1, LP5817_I2CTimeout);
	if (ok)
	{
		val = data[1];
	}
	return ok;
}

void LP5817::SetOneOutputBitState(unsigned int bitnum, bool on) noexcept
{
	uint8_t newOutputRegister = outputRegister;
	if (on)
	{
		newOutputRegister &= ~(1u << bitnum);
	}
	else
	{
		newOutputRegister |= (1u << bitnum);
	}
	if (newOutputRegister != outputRegister)
	{
		outputRegister = newOutputRegister;
		outputNeedsUpdating = true;
	}
}

void LP5817::SetOutputBitsState(uint8_t bitsToSet, uint8_t mask) noexcept
{
	const uint8_t newOutputRegister = (outputRegister & (~mask)) | bitsToSet;
	if (newOutputRegister != outputRegister)
	{
		outputRegister = newOutputRegister;
		outputNeedsUpdating = true;
	}
}

void LP5817::Poll() noexcept
{
	if (outputNeedsUpdating)
	{
		outputNeedsUpdating = false;
		Write8(LP5817_Register::output, outputRegister);
	}
	(void)Read8(LP5817_Register::input, inputRegister);			// this will update the saved input register if it succeeds
}

bool LP5817::Write8(LP5817_Register reg, uint8_t val) noexcept
{
	uint8_t data[2] = { (uint8_t)reg, val };
	return Transfer(data, nullptr, 2, 0, LP5817_I2CTimeout);
}

#endif
