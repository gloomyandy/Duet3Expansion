/*
 * ScanningSensorHandler.cpp
 *
 *  Created on: 16 Jun 2023
 *      Author: David
 */

#include "ScanningSensorHandler.h"

#if SUPPORT_LDC1612

#include <InputMonitors/InputMonitor.h>
#include <Hardware/SharedI2CMaster.h>
#include <Hardware/LDC1612.h>
#include <CanMessageFormats.h>
#include <AnalogIn.h>
#include <Movement/StepTimer.h>
#include <Platform/TaskPriorities.h>
#include <Platform/AveragingFilter.h>

constexpr unsigned int ResultBitsDropped = 8;		// we drop this number of least significant bits in the result

static LDC1612 *sensor = nullptr;
static AnalogIn::AdcTaskHookFunction *oldHookFunction = nullptr;
static volatile uint32_t lastReading = 0;
static volatile bool isCalibrating = false;
static uint32_t lastReadingTakenAt = 0;
static uint32_t offset = 0;
static volatile AnalogInCallbackFunction callbackFunction = nullptr;
static CallbackParameter callbackParameter;

namespace TouchMode
{
//private:
	static bool enabled = false;
	static uint16_t sensitivity;
	static uint32_t startTime;					// the time we started taking touch mode readings, in step clocks
	static uint32_t lastReadingTime;			// the last reading time of the sensor, in step clocks
	static uint32_t lastReading;
	static unsigned int numBadReadings;
	static InputMonitor *inputMonitor = nullptr;
	static AveragingFilter<16> speedFilter;

//public:
	static void Start(InputMonitor& p_monitor, uint32_t sens) noexcept;
	static void Stop() noexcept;
	static void ProcessReading(uint32_t reading) noexcept;
	static bool IsEnabled() noexcept { return enabled; }
};

void TouchMode::Start(InputMonitor& p_monitor, uint32_t sens) noexcept
{
	inputMonitor = &p_monitor;
	sensitivity = (uint16_t)sens;
	lastReading = 0;
	numBadReadings = 0;
	startTime = StepTimer::GetTimerTicks();
	speedFilter.Init(0);
	enabled = true;
}

void TouchMode::Stop() noexcept
{
	enabled = false;
	inputMonitor = nullptr;
}

// Process a sensor reading when we are in touch mode
// A typical probing speed is 5mm/sec. At this speed, a processing interval of 1ms will give us a probing resolution of 5um.
void TouchMode::ProcessReading(uint32_t reading) noexcept
{
	if ((lastReading & 0xF0000000) != 0)			// if it's a bad reading
	{
		++numBadReadings;
		if (numBadReadings == 3)					// if we get 3 bad readings in a row, give up
		{
			if (inputMonitor != nullptr)
			{
				inputMonitor->SetTriggered();
			}
			Stop();
		}
	}
	else
	{
		numBadReadings = 0;
		const uint32_t now = StepTimer::GetTimerTicks();
		const uint32_t interval = now - lastReadingTime;

		// We expect the speed to fit in 16 bits normally
		const uint16_t newSpeed = (uint16_t)constrain<int32_t>((((int32_t)reading - (int32_t)lastReading) * 16)/(int32_t)interval, 0, 65535);

		if (now - startTime >= StepTimer::StepClockRate/20)		// if we've been probing for at least 50ms
		{
			const uint32_t speedSum = speedFilter.GetSum();
			if (((uint32_t)newSpeed * (65536 * speedFilter.NumAveraged())) < speedSum * sensitivity)
			{
				inputMonitor->SetTriggered();
				Stop();
			}
			debugPrintf("Speed %u/%u\n", newSpeed, (unsigned int)(speedSum/speedFilter.NumAveraged()));
		}

		speedFilter.ProcessReading(newSpeed);
		lastReading = reading;
		lastReadingTime = now;
	}
}

// This hook function is called by the AnalogIn task
static void LDC1612TaskHook() noexcept
{
	// Read the sensor status at most once every millisecond, otherwise the AnalogIn task tends to hog the I2C bus and the accelerometer can't be read
	if (!isCalibrating && !digitalRead(LDC1612InterruptPin) && millis() != lastReadingTakenAt)
	{
		PriorityBoost booster(TaskPriority::LDC1612Reading);

		uint32_t val;
		if (sensor->GetChannelResult(0, val))		// if no error
		{
			lastReading = val;						// save all 28 bits of data + 4 error bits
			lastReadingTakenAt = millis();			// record when we took it
			if (TouchMode::IsEnabled())
			{
				TouchMode::ProcessReading(val);
			}
			else
			{
				TaskCriticalSectionLocker lock;
				const AnalogInCallbackFunction fn = callbackFunction;
				if (fn != nullptr)
				{
					fn(callbackParameter, ScanningSensorHandler::GetReading());
				}
			}
		}
		else if (millis() - lastReadingTakenAt > 5)	// we get occasional reading errors, so don't report a bad reading unless it's 5ms since we had a good reading
		{
			lastReading = 0;
		}
	}

	if (oldHookFunction != nullptr)
	{
		oldHookFunction();
	}
}

void ScanningSensorHandler::Init(SharedI2CMaster& i2cDevice) noexcept
{
	// Set up the external clock to the LDC1612.
	// The higher the better, but the maximum is 40MHz
#if defined(SAMMYC21) || defined(TOOL1LC)
	// Assume we are using a LDC1612 breakout board with its own crystal, so we don't need to generate a clock
#elif defined(SZP)
	// We can use the 96MHz DPLL output divided by 3 to get 32MHz but it is probably better to use 25MHz from the crystal directly for better stability.
	static_assert(LDC1612::ClockFrequency == 25.0 || LDC1612::ClockFrequency == 32.0);
	if constexpr(LDC1612::ClockFrequency == 25.0)
	{
		ConfigureGclk(GclkNumPA23, GclkSource::xosc, 1, true);
	}
	else if constexpr(LDC1612::ClockFrequency == 32.0)
	{
		ConfigureGclk(GclkNumPA23, GclkSource::dpll, 3, true);
	}
	SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::H);
#elif defined(TOOL1RR)
	// We use the 120MHz DPLL output divided by 4 to get 30MHz. It might be better to use 25MHz from the crystal directly for better stability.
	static_assert(LDC1612::ClockFrequency == 25.0 || LDC1612::ClockFrequency == 30.0);
	if constexpr(LDC1612::ClockFrequency == 25.0)
	{
		ConfigureGclk(GclkNumPB11, GclkSource::xosc0, 1, true);
	}
	else if constexpr(LDC1612::ClockFrequency == 30.0)
	{
		ConfigureGclk(GclkNumPB11, GclkSource::dpll0, 4, true);
	}
	SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::M);
#else
# error LDC support not implemented for this processor
#endif

	sensor = new LDC1612(i2cDevice);
	if (sensor->CheckPresent())
	{
		sensor->SetDefaultConfiguration(0, false);
		pinMode(LDC1612InterruptPin, PinMode::INPUT);
		oldHookFunction = AnalogIn::SetTaskHook(LDC1612TaskHook);
	}
	else
	{
		DeleteObject(sensor);
#if defined(TOOL1LC) || defined(SZP)
		ClearPinFunction(LDC1612ClockGenPin);
#endif
	}
}

bool ScanningSensorHandler::IsPresent() noexcept
{
	return sensor != nullptr;
}

uint32_t ScanningSensorHandler::GetReading() noexcept
{
	if ((lastReading & 0xF0000000) != 0) { return ScanningSensorBadReadingVal; }
	const uint32_t reading = lastReading >> ResultBitsDropped;
	return (reading > offset) ? reading - offset : 0;
}

GCodeResult ScanningSensorHandler::SetOrCalibrateCurrent(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept
{
	if (sensor == nullptr)
	{
		reply.copy("scanning probe not present");
	}
	else
	{
		if (param == CanMessageChangeInputMonitorNew::paramAutoCalibrateDriveLevelAndReport)
		{
			isCalibrating = true;
			bool ok = sensor->CalibrateDriveCurrent(0);
			if (ok)
			{
				extra = sensor->GetDriveCurrent(0);
				delay(4);											// give time for a reading to become available
				uint32_t val;
				ok = sensor->GetChannelResult(0, val);
				isCalibrating = false;
				if (ok && (val & 0xF0000000) == 0)
				{
					val >>= ResultBitsDropped;
					offset = val - (val >> 4);						// set the offset to 15/16 of the reading
					reply.printf("Calibration successful, sensor drive current is %u, offset is %" PRIu32, extra, offset);
					return GCodeResult::ok;
				}
			}
			isCalibrating = false;
			reply.copy("failed to calibrate sensor drive current");
		}
		else if (param == CanMessageChangeInputMonitorNew::paramReportDriveLevel)
		{
			extra = sensor->GetDriveCurrent(0);
			reply.printf("Sensor drive current is %u, offset is %" PRIu32, extra, offset);
			return GCodeResult::ok;
		}
		else
		{
			const uint16_t driveCurrent = param & CanMessageChangeInputMonitorNew::paramDriveLevelMask;
			const uint32_t newOffset = param >> CanMessageChangeInputMonitorNew::paramOffsetShift;
			isCalibrating = true;
			const bool ok = sensor->SetDriveCurrent(0, driveCurrent);
			isCalibrating = false;
			if (ok)
			{
				offset = newOffset;
				extra = param;
				return GCodeResult::ok;
			}
			reply.copy("failed to set sensor drive current");
		}
	}
	extra = 0xFF;
	return GCodeResult::error;
}

GCodeResult ScanningSensorHandler::SelectTouchMode(InputMonitor& monitor, uint32_t param, const StringRef& reply, uint8_t& extra) noexcept
{
	if (sensor == nullptr)
	{
		reply.copy("scanning probe not present");
	}
	else
	{
		TouchMode::Start(monitor, param);
		return GCodeResult::ok;
	}
	extra = 0xFF;
	return GCodeResult::error;
}

void ScanningSensorHandler::ClearTouchMode() noexcept
{
	TouchMode::Stop();
}

bool ScanningSensorHandler::SetCallback(AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall) noexcept
{
	callbackParameter = param;
	callbackFunction = fn;
	// We ignore the ticksPerCall parameter
	return true;
}

// Return the oscillation frequency in MHz
float ScanningSensorHandler::GetFrequency() noexcept
{
	return ldexpf((lastReading & 0x0FFFFFFF) * LDC1612::FRef, -28);
}

void ScanningSensorHandler::AppendDiagnostics(const StringRef& reply) noexcept
{
	reply.lcat("Inductive sensor: ");
	if (IsPresent())
	{
		// Append diagnostic data to string
		const uint32_t val = lastReading;
		if (val != 0)
		{
			reply.catf("raw value %" PRIu32 ", frequency %.2fMHz, current setting %u", val & 0x0FFFFFFF, (double)GetFrequency(), sensor->GetDriveCurrent(0));
			if ((val >> 28) == 0)
			{
				reply.cat(", ok");
			}
			else
			{
				if ((val >> 28) & LDC1612::ERR_UR0)
				{
					reply.cat(", under-range error");
				}
				if ((val >> 28) & LDC1612::ERR_OR0)
				{
					reply.cat(", over-range error");
				}
				if ((val >> 28) & LDC1612::ERR_WD0)
				{
					reply.cat(", watchdog error");
				}
				if ((val >> 28) & LDC1612::ERR_AE0)
				{
					reply.cat(", amplitude error");
				}
			}
		}
		else
		{
			reply.cat("error retrieving data from LDC1612");
		}
	}
	else
	{
		reply.cat("not found");
	}
}

#endif

// End
