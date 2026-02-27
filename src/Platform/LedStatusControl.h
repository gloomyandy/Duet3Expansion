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

enum class LedStatusCode : uint8_t
{
	// Normal codes that indicated heater status
	cold = 0, heating, atTarget, cooling,
	// Codes that cause a temporary change that reverts after a short time
	loadCellTransient,
	// Error codes that are permanent until cleared
	irSensorFail, heatingFault,
};

class LP5817;

class LedStatusControl
{
public:
	LedStatusControl(unsigned int i2cChannel) noexcept;
	void SetStatus(LedStatusCode status) noexcept;
	void ClearError(LedStatusCode status) noexcept;
	void SetTestColour(uint32_t colour) noexcept;

private:
	LP5817 *ledDriver = nullptr;

};

#endif

#endif /* SRC_PLATFORM_LEDSTATUSCONTROL_H_ */
