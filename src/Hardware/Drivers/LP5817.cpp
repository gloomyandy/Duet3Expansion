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
	(void)Write8(LP5817_Register::reset_cmd, 0xCC);						// try to reset the device
}

// Set new LED colours next time Poll is called
void LP5817::SetColour(uint32_t colour) noexcept
{
	if (colour != currentColour)
	{
		if (Take(LP5817_I2CTimeout))									// get ownership of the I2C bus, this also saves having a mutex for this device
		{
			if (!initialised)
			{
				uint8_t temp;
				initialised = Write8(LP5817_Register::reset_cmd, 0xCC)						// reset the device
							&& Write8(LP5817_Register::chip_en, 0x01)						// enable IC, must do this before update_cmd will work
							&& Write8(LP5817_Register::out0_dc, CurrentSetting[0])			// set R,G,B currents
							&& Write8(LP5817_Register::out1_dc, CurrentSetting[1])
							&& Write8(LP5817_Register::out2_dc, CurrentSetting[2])
							// The manual PWM registers default to 0 after reset, so no need to write them
							&& Write8(LP5817_Register::dev_config1, 0x07)					// enable R,G,B
							&& Write8(LP5817_Register::update_cmd, 0x01)					// update dev_config1
							&& Read8(LP5817_Register::out1_dc, temp)
							&& temp == CurrentSetting[1]
							;
			}
			if (initialised)
			{
				success = Write8(LP5817_Register::out0_manual_pwm, (uint8_t)colour)
						&& Write8(LP5817_Register::out1_manual_pwm, (uint8_t)(colour >> 8))
						&& Write8(LP5817_Register::out2_manual_pwm, (uint8_t)(colour >> 16));
				currentColour = colour;
			}
			Release();													// release the bus
		}
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
