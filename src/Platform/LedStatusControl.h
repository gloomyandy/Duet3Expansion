/*
 * LedStatusControl.h
 *
 *  Created on: 20 Feb 2026
 *      Author: David
 */

#ifndef SRC_PLATFORM_LEDSTATUSCONTROL_H_
#define SRC_PLATFORM_LEDSTATUSCONTROL_H_

#include <RepRapFirmware.h>

#if SUPPORT_LP5817

#include <RTOSIface/RTOSIface.h>
#include <atomic>

enum class LedStatusCode : uint8_t
{
	noCode = 0,

	// Normal codes that indicate heater status
	cold, heating, atTarget, cooling,

	// Error codes that are permanent until cleared and transient codes, highest priority ones first
	irSensorFail,
	heatingFault,
	loadCellTransient,
};

class LP5817;

class LedStatusControl
{
public:
	LedStatusControl(unsigned int i2cChannel) noexcept;
	void SetStatus(LedStatusCode status, float minFactor = 0.0) noexcept;
	void SetErrorOrTransient(LedStatusCode status) noexcept;
	void ClearError(LedStatusCode status) noexcept;
	void SetTestColour(uint32_t colour) noexcept;

	void TaskStart() noexcept;

private:
	static constexpr uint32_t TaskStackSize = 200;
	static constexpr uint32_t LedPollInterval = 10;

	LP5817 *ledDriver = nullptr;
	Task<TaskStackSize> *ledTask = nullptr;

	// These variables are written by the app and read by the task
	LedStatusCode newStatus = LedStatusCode::noCode;				// normal status code that the app wants to set

	// This is updated by both the app and the task
	std::atomic<uint32_t> currentErrorCodes = 0;					// error and transient codes that the app wants shown

	// These variables are used by the task only
	uint32_t whenCodeStarted;										// when the current transient or error code display was started
	float lowerBreathingLimit = 0.0;
	float currentBreathingValue;
	LedStatusCode statusBeingDisplayed = LedStatusCode::noCode;		// what code we are displaying
	uint8_t numFastBlinks, numSlowBlinks;							// if we are displaying an error code then these are how many blinks we want
	bool errorLedOn;
	bool breathingIn;
};

#endif

#endif /* SRC_PLATFORM_LEDSTATUSCONTROL_H_ */
