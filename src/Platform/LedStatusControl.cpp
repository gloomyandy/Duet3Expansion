/*
 * LedStatusControl.cpp
 *
 *  Created on: 20 Feb 2026
 *      Author: David
 */

#include "LedStatusControl.h"

#if SUPPORT_LP5817

#include <Platform/Platform.h>
#include <Hardware/Drivers/LP5817.h>

LedStatusControl::LedStatusControl(unsigned int i2cChannel) noexcept
{
	ledDriver = new LP5817(Platform::GetSharedI2C(i2cChannel));
}

// Function to set colours for testing the LEDs
void LedStatusControl::SetTestColour(uint32_t colour) noexcept
{
	ledDriver->SetColour(colour);
}

#endif
