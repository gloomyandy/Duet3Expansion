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
#include <Platform/TaskPriorities.h>

namespace LedColour
{
	constexpr uint32_t off = 0;
	constexpr uint32_t red = 0x0000FF;
	constexpr uint32_t green = 0x00FF00;
	constexpr uint32_t blue = 0xFF0000;
	constexpr uint32_t amber = 0x0040FF;
	constexpr uint32_t purple = 0xFF00FF;
	constexpr uint32_t white = 0xFFFFFF;
}

extern "C" void LedTaskStart(void* pvParameters) noexcept
{
	((LedStatusControl*)pvParameters)->TaskStart();
}

// Adjust the brightness of a colour. 'brightness' must be in the range 0.0 to 1.0.
static uint32_t AdjustBrightness(uint32_t colour, float brightness) noexcept
{
	return ((uint32_t)(((colour >> 16) & 0x0000FFu) * brightness) << 16)
		 | ((uint32_t)(((colour >> 8) & 0x0000FFu) * brightness) << 8)
		 | (uint32_t)((colour & 0x0000FFu) * brightness);
}

LedStatusControl::LedStatusControl(unsigned int i2cChannel) noexcept
{
	ledDriver = new LP5817(Platform::GetSharedI2C(i2cChannel));
	ledTask = new Task<TaskStackSize>;
	ledTask->Create(LedTaskStart, "LED", (void*)this, TaskPriority::Led);
}

// Function to set colours for testing the LEDs
void LedStatusControl::SetTestColour(uint32_t colour) noexcept
{
	ledDriver->SetColour(colour);
}

// Set a temporary or permanent status code, or an error code. 'minFactor' must be in the range 0.0 to 1.0.
void LedStatusControl::SetStatus(LedStatusCode status, float minFactor) noexcept
{
	lowerBreathingLimit = minFactor;
	newStatus = status;
}

void LedStatusControl::SetErrorOrTransient(LedStatusCode status) noexcept
{
		currentErrorCodes |= (1u << (unsigned int)status);
}

// Clear an error code
void LedStatusControl::ClearError(LedStatusCode status) noexcept
{
	switch(status)
	{
	// Transient and error status codes
	case LedStatusCode::loadCellTransient:
	case LedStatusCode::irSensorFail:
	case LedStatusCode::heatingFault:
		currentErrorCodes &= ~(1u << (unsigned int)status);
		break;

	default:
		break;
	}
}

void LedStatusControl::TaskStart() noexcept
{
	uint32_t previousWakeTime = xTaskGetTickCount();
	for (;;)
	{

		xTaskDelayUntil(&previousWakeTime, LedPollInterval);

		// 1. Are we supposed to be displaying an error or transient code?
		const uint32_t wantedErrorCodes = currentErrorCodes.load();
		if (wantedErrorCodes != 0)
		{
			// There is an error or transient code to display
			const LedStatusCode highestPriorityErrorCode = (LedStatusCode)LowestSetBit(wantedErrorCodes);
			if (highestPriorityErrorCode != statusBeingDisplayed)
			{
				statusBeingDisplayed = highestPriorityErrorCode;
				whenCodeStarted = millis();

				// Start displaying a new error code
				if (highestPriorityErrorCode == LedStatusCode::loadCellTransient)
				{
					ledDriver->SetColour(LedColour::purple);
				}
				else
				{
					currentErrorCodes &= ~(1u << (unsigned int)LedStatusCode::loadCellTransient);	// the transient code will be out of date by the time we finish displaying the error code
					ledDriver->SetColour(LedColour::red);				// turn LED on at the start of displaying an error code
					errorLedOn = true;
					switch (highestPriorityErrorCode)
					{
					// Error codes
					case LedStatusCode::irSensorFail:
						numFastBlinks = 5;
						numSlowBlinks = 3;
						break;

					case LedStatusCode::heatingFault:
						numFastBlinks = 1;
						numSlowBlinks = 5;
						break;

					default:
						break;
					}
				}
			}
			else if (statusBeingDisplayed == LedStatusCode::loadCellTransient)
			{
				if (millis() - whenCodeStarted >= 190u)					// we display purple for 200ms
				{
					currentErrorCodes &= ~(1u << (unsigned int)LedStatusCode::loadCellTransient);
					// On the next call we will display the regular status
				}
			}
			else
			{
				// Continue displaying the existing error code
				const uint32_t now = millis();
				const uint32_t elapsedTime = now - whenCodeStarted;
				bool wantLedOn;
				if (elapsedTime < numSlowBlinks * 2000u)
				{
					wantLedOn = ((elapsedTime / 1000u) & 1u) == 0;
				}
				else if (elapsedTime < numSlowBlinks * 2000u + numFastBlinks * 500u)
				{
					wantLedOn = (((elapsedTime - numFastBlinks * 2000u) / 250u) & 1u) == 0;
				}
				else if (elapsedTime < numFastBlinks * 500u + numSlowBlinks * 2000u + 1000u)
				{
					wantLedOn = false;							// leave the LED off for 1 second after the pattern
				}
				else
				{
					whenCodeStarted = now;
					wantLedOn = true;
				}
				if (wantLedOn != errorLedOn)
				{
					errorLedOn = wantLedOn;
					ledDriver->SetColour((wantLedOn) ? LedColour::red : LedColour::off);
				}
			}
		}
		else
		{
			// Display the normal status code
			if (statusBeingDisplayed != newStatus)
			{
				statusBeingDisplayed = newStatus;
				switch (statusBeingDisplayed)
				{
				case LedStatusCode::cold:
					ledDriver->SetColour(LedColour::green);
					break;

				case LedStatusCode::heating:
					currentBreathingValue = lowerBreathingLimit;
					breathingIn = true;
					ledDriver->SetColour(AdjustBrightness(LedColour::amber, currentBreathingValue));
					break;

				case LedStatusCode::cooling:
					ledDriver->SetColour(LedColour::blue);
					break;

				case LedStatusCode::atTarget:
					ledDriver->SetColour(LedColour::amber);
					break;

				default:											// this should not happen
					ledDriver->SetColour(LedColour::white);
					break;
				}
			}
			else if (statusBeingDisplayed == LedStatusCode::heating)
			{
				float newBreathingValue;
				if (breathingIn && (newBreathingValue = currentBreathingValue + 0.01) <= 1.0)
				{
					currentBreathingValue = newBreathingValue;
				}
				else
				{
					newBreathingValue = currentBreathingValue - 0.01;
					if (newBreathingValue >= lowerBreathingLimit)
					{
						currentBreathingValue = newBreathingValue;
						breathingIn = false;
					}
					else
					{
						currentBreathingValue = min<float>(lowerBreathingLimit + 0.01, 1.0);
						breathingIn = true;
					}
				}
				ledDriver->SetColour(AdjustBrightness(LedColour::amber, currentBreathingValue));
			}
		}
	}
}

#endif
