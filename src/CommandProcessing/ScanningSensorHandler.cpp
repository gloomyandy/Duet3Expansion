/*
 * ScanningSensorHandler.cpp
 *
 *  Created on: 16 Jun 2023
 *      Author: David
 *
 *  This file may be distributed under the terms of the GNU GPLv3 license.
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
#include <AppNotifyIndices.h>
#include <Interrupts.h>

#define USE_BUTTERWORTH_FILTER		1

constexpr unsigned int ResultBitsDropped = 8;		// we drop this number of least significant bits in the result

constexpr unsigned int LdcTaskStackWords = 150;		// 100 was too little

static Task<LdcTaskStackWords> *ldcTask = nullptr;

static LDC1612 *sensor = nullptr;
static volatile uint32_t lastReading = 0;
static volatile bool isCalibrating = false;
static uint32_t lastReadingTakenAt = 0;
static uint32_t offset = 0;
static InputMonitor *inputMonitor = nullptr;		// when the sensor is active this point to the associated input monitor; when inactive it is null

namespace TouchMode
{
//private:
#if USE_BUTTERWORTH_FILTER
	// Butterworth bandpass filter code and coefficients borrowed from https://github.com/vvuk/klipper/blob/vlad/eddy-ng/src/sensor_ldc1612_ng.c
	static const size_t sosSections = 2;
	static float sosState[sosSections][2];
	static constexpr float sosButterworthFilter500[sosSections][6] =
			{
				{	0.013359200027856505,
					0.02671840005571301,
					0.013359200027856505,
					1.0,
					-1.686278256753083,
					0.753714473246724
				 },
				 {	1.0,
					-2.0,
					1.0,
					1.0,
					-1.9250515947328444,
					0.9299234737648037
				 }
			};
	static float SosFilter(float value, const float filter[][6], float state[][2]) noexcept;
	static float baseFreq;
	static float lastValue;
	static float startValue;
	static bool falling;
	static size_t goodCnt;						// for debug use
	static float threshold;
#else
	static AveragingFilter<16> speedFilter;
	static uint32_t lastReadingTime;			// the previous reading time of the sensor, in step clocks
	static uint16_t lastSpeed, lastSpeedMinus1, lastSpeedMinus2;
#endif
	static bool enabled = false;
	static uint16_t sensitivity;
	static uint32_t startTime;					// the time we started taking touch mode readings, in step clocks
	static uint32_t lastReading;				// the previous reading
	static unsigned int numBadReadings;

//public:
	static void Start(uint32_t sens) noexcept;
	static void Stop() noexcept;
	static void ProcessReading(uint32_t reading) noexcept;
	static bool IsEnabled() noexcept { return enabled; }
};

void TouchMode::Start(uint32_t sens) noexcept
{
	sensitivity = (uint16_t)sens;				// we only send a 16-bit sensitivity
	lastReading = 0;
	numBadReadings = 0;
	startTime = StepTimer::GetTimerTicks();
#if USE_BUTTERWORTH_FILTER
	enabled = true;
	for(size_t i = 0; i < sosSections; i++)
	{
		sosState[i][0] = 0.0f;
		sosState[i][1] = 0.0f;
	}
	baseFreq = 0.0f;
	lastValue = 0.0f;
	startValue = 0.0f;
	falling = false;
	threshold = 20000.0f * (1 - ((float)sensitivity/65536));
	goodCnt = 0;
#else
	speedFilter.Init(0);
#endif
	enabled = true;
}

void TouchMode::Stop() noexcept
{
	enabled = false;
}

#if USE_BUTTERWORTH_FILTER

// Butterworth bandpass filter code and coefficients borrowed from https://github.com/vvuk/klipper/blob/vlad/eddy-ng/src/sensor_ldc1612_ng.c
float TouchMode::SosFilter(float value, const float filter[][6], float state[][2]) noexcept
{
	for (size_t i = 0; i < sosSections; i++)
	{
		const float w1 = state[i][0];
		const float w2 = state[i][1];
		const float w0 = value - filter[i][4]*w1 - filter[i][5]*w2;
		value = filter[i][0]*w0 + filter[i][1]*w1 + filter[i][2]*w2;
		state[i][0] = w0;
		state[i][1] = w1;
	}
	return value;
}

#endif

// Process a sensor reading when we are in touch mode
// A typical probing speed is 5mm/sec. At this speed, a processing interval of 1ms will give us a probing resolution of 5um.
void TouchMode::ProcessReading(uint32_t reading) noexcept
{
	if ((reading & 0xE0000000) != 0)				// if it's a bad reading (ignoring amplitude errors)
	{
		++numBadReadings;
		if (numBadReadings == 3)					// if we get 3 bad readings in a row, give up
		{
			if (inputMonitor != nullptr)
			{
				inputMonitor->SetTriggered();
			}
//			debugPrintf("Bad reading %08" PRIx32 "\n", reading);
			Stop();
		}
	}
	else
	{
		const uint32_t now = StepTimer::GetTimerTicks();

#if USE_BUTTERWORTH_FILTER
		// Butterworth bandpass filter code and coefficients borrowed from see https://github.com/vvuk/klipper/blob/vlad/eddy-ng/src/sensor_ldc1612_ng.c
		const float freq = (float)reading;							// no need to convert to an actual frequency here
		if (now - startTime >= StepTimer::StepClockRate/10)			// allow for the movement start delay and some more
		{
			const float value = SosFilter(freq - baseFreq, sosButterworthFilter500, sosState);
			//debugPrintf("%d F %f V %f\n", goodCnt++, (double)(freq - baseFreq), (double)value);
			// allow filter to stabilise
			if (now - startTime >= StepTimer::StepClockRate/5)
			{
				if (value < lastValue)
				{
					falling = true;
				}
				else if (value > lastValue)
				{
					if (falling)
					{
						if (startValue - lastValue >= threshold)
						{
							inputMonitor->SetTriggered();
							Stop();
							//delay(500);
							//debugPrintf("%d Trig F %f V %f LV %f SV %f BV %f TH %f\n", goodCnt++, (double)freq, (double)value, (double)lastValue, (double)startValue, (double)baseFreq, (double)threshold);
						}
					}
					falling = false;
					startValue = value;
				}
				lastValue = value;
			}
		}
		else
		{
			baseFreq = freq;
		}
		numBadReadings = 0;
#else
		const uint32_t interval = now - lastReadingTime;

		// We expect the speed to fit in 16 bits normally
		const uint16_t currentSpeed = (uint16_t)constrain<int32_t>((((int32_t)reading - (int32_t)lastReading) * 256)/(int32_t)interval, 0, 65535);

		if (now - startTime >= StepTimer::StepClockRate/10)		// allow for the movement start delay and some more
		{
			// Average the most recent 4 readings
			const uint32_t recentSpeed = (uint32_t)currentSpeed + lastSpeed + lastSpeedMinus1 + lastSpeedMinus2;
			const uint32_t speedSum = speedFilter.GetSum();
//			debugPrintf("R %u I%u S %u/%u\n", (unsigned int)reading, (unsigned int)interval, currentSpeed, (unsigned int)(speedSum/speedFilter.NumAveraged()));
			if ((recentSpeed * ((65536/4) * speedFilter.NumAveraged())) < speedSum * sensitivity)
			{
				inputMonitor->SetTriggered();
				Stop();
//				debugPrintf("Speed %u/%u\n", (unsigned int)(recentSpeed/4), (unsigned int)(speedSum/speedFilter.NumAveraged()));
			}
		}

		speedFilter.ProcessReading(lastSpeedMinus2);
		lastSpeedMinus2 = lastSpeedMinus1;
		lastSpeedMinus1 = lastSpeed;
		lastSpeed = currentSpeed;
		lastReading = reading;
		lastReadingTime = now;
		numBadReadings = 0;
#endif
	}
}

[[noreturn]] static void LdcTaskLoop(void* param) noexcept;

// Activate the scanning sensor returning true if successful
bool ScanningSensorHandler::Activate(InputMonitor& monitor) noexcept
{
	if (sensor == nullptr)
	{
		return false;
	}

	if (ldcTask == nullptr)
	{
		ldcTask = new Task<LdcTaskStackWords>;
		ldcTask->Create(LdcTaskLoop, "ScanSens", nullptr, TaskPriority::LdcTask);
	}
	inputMonitor = &monitor;
	return true;
}

void ScanningSensorHandler::Deactivate()
{
	inputMonitor = nullptr;
}

// Align this on a cache line boundary for SAMC21
__attribute__ ((aligned (8))) static void Ldc1612Interrupt(CallbackParameter) noexcept
{
	TaskBase::GiveFromISR(ldcTask, NotifyIndices::LDC1612);
	DisablePinInterrupt(LDC1612InterruptPin);
}

// Function executed by the LDC task
[[noreturn]] static void LdcTaskLoop(void* param) noexcept
{
	AttachPinInterrupt(LDC1612InterruptPin, Ldc1612Interrupt, InterruptMode::low, CallbackParameter(), false);
	for (;;)
	{
		if (inputMonitor == nullptr || isCalibrating)
		{
			delay(5);
		}
		else
		{
			if (sensor->IsChannelReady(0))					// this also clears the interrupt
			{
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
						inputMonitor->AnalogInterrupt(ScanningSensorHandler::GetReading());
					}
				}
				else if (millis() - lastReadingTakenAt > 5)	// we get occasional reading errors, so don't report a bad reading unless it's 5ms since we had a good reading
				{
					lastReading = 0;
				}
			}

			EnablePinInterrupt(LDC1612InterruptPin);
			TaskBase::TakeIndexed(NotifyIndices::LDC1612, 5);
		}
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
		SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::H);
	}
	else if constexpr(LDC1612::ClockFrequency == 32.0)
	{
		ConfigureGclk(GclkNumPA23, GclkSource::dpll, 3, true);
		SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::H);
	}
#elif defined(TOOL1RR)
	// We use the 120MHz DPLL output divided by 4 to get 30MHz. It might be better to use 25MHz from the crystal directly for better stability.
	static_assert(LDC1612::ClockFrequency == 25.0 || LDC1612::ClockFrequency == 30.0);
	if constexpr(LDC1612::ClockFrequency == 25.0)
	{
		ConfigureGclk(GclkNumPB11, GclkSource::xosc0, 1, true);
		SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::M);
	}
	else if constexpr(LDC1612::ClockFrequency == 30.0)
	{
		ConfigureGclk(GclkNumPB11, GclkSource::dpll0, 4, true);
		SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::M);
	}
#else
# error LDC support not implemented for this processor
#endif

	sensor = new LDC1612(i2cDevice);
	if (sensor->CheckPresent())
	{
		sensor->SetDefaultConfiguration(0, false);
		pinMode(LDC1612InterruptPin, PinMode::INPUT);
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
			delay(2);												// avoid race with LDC task
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
			delay(2);												// avoid race with LDC task
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

GCodeResult ScanningSensorHandler::SelectTouchMode(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept
{
	if (sensor == nullptr)
	{
		reply.copy("scanning probe not present");
	}
	else
	{
		TouchMode::Start(param);
		return GCodeResult::ok;
	}
	extra = 0xFF;
	return GCodeResult::error;
}

void ScanningSensorHandler::ClearTouchMode() noexcept
{
	TouchMode::Stop();
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
		if (ldcTask == nullptr)
		{
			reply.cat("never activated");
		}
		else if (inputMonitor == nullptr)
		{
			reply.cat("not currently active");
		}
		else
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
	}
	else
	{
		reply.cat("not found");
	}
}

#endif

// End
