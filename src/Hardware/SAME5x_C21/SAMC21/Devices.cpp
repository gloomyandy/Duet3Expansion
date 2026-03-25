/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>
#include <Platform/TaskPriorities.h>

#if SAMC21

#include <AnalogIn.h>
#include <AnalogOut.h>
#include <RTOSIface/RTOSIface.h>

// Analog input support
constexpr size_t AnalogInTaskStackWords = 170;				// was 120 but we got a stack overflow
static Task<AnalogInTaskStackWords> analogInTask;

void DeviceInit() noexcept
{
	AnalogIn::Init(DmacChanAdc0Rx, DmacPrioAdcRx,
#ifdef SZP
					true				// SZP uses a 2.5V external reference due to nonlinearity of ADC
#else
					false
#endif
		);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);
}

#endif

// End
