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
	currentColour = black;
	uint8_t temp;
	return Write8(LP5817_Register::reset_cmd, 0xCC)
		&& Write8(LP5817_Register::out0_dc, CurrentSetting[0])
		&& Write8(LP5817_Register::out1_dc, CurrentSetting[1])
		&& Write8(LP5817_Register::out2_dc, CurrentSetting[2])
		&& Write8(LP5817_Register::chip_en, 0x01)
		&& Read8(LP5817_Register::out1_dc, temp)
		&& temp == CurrentSetting[1]
		;
}

// Set new LED colours next time Poll is called
void LP5817::SetColour(LedColour colour) noexcept
{
	if (colour != currentColour)
	{
		Write8(LP5817_Register::dev_config1, (uint8_t)colour);
		currentColour = colour;
	}
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

bool LP5817::Write8(LP5817_Register reg, uint8_t val) noexcept
{
	uint8_t data[2] = { (uint8_t)reg, val };
	return Transfer(data, nullptr, 2, 0, LP5817_I2CTimeout);
}

#endif
