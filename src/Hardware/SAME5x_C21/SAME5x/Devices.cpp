/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME5x

#include <AnalogIn.h>
#include <AnalogOut.h>
#include <Platform/TaskPriorities.h>
#include <Platform/Platform.h>

// Analog input support
constexpr size_t AnalogInTaskStackWords = 300;
static Task<AnalogInTaskStackWords> analogInTask;

void DeviceInit() noexcept
{
#if defined(EXP1HCL) || defined(M23CL) || defined(TOOLINDX)
	SetPinMode(TmcClockPin, OUTPUT_LOW);			// default the TMC clock to its internal clock until we program the clock generator
#endif
	AnalogIn::Init(NvicPriorityAdc);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);
}

#endif

// End
