/*
 * InductiveHeaterPort.cpp
 *
 *  Created on: 29 Dec 2025
 *      Author: David
 */

#include "InductiveHeaterPort.h"

#if SUPPORT_INDUCTIVE_HEATER

#include <Platform/Platform.h>
#include <Timers.h>
#include <Platform/TaskPriorities.h>
#include <AppNotifyIndices.h>
#include <atomic>

#if SAME5x
# include <hri_tc_e54.h>
# include <hri_tcc_e54.h>
# include <hri_mclk_e54.h>
#elif SAMC21
# include <hri_tc_c21.h>
# include <hri_tcc_c21.h>
# include <hri_mclk_c21.h>
#else
# error Unsupported processor
#endif

Task<InductiveHeaterPort::CalibrationTaskStackWords> *_ecv_null InductiveHeaterPort::calibrationTask = nullptr;

InductiveHeaterPort::InductiveHeaterPort()
{
	// Nothing to do here
}

void InductiveHeaterPort::Init() noexcept
{
	// Set up the oscillator TCC to generate a square wave at the coil resonant frequency
	EnableTccClock(InductiveHeaterOscTccDeviceNumber, GclkNum120MHz);	// use the 120MHz GCLK to get the best frequency setting resolution
	const uint32_t oscPrescaler = 0;									// 16-or 24-bit TCC with a 120MHz clock and prescaler 1 gives us frequencies from 1.8kHz upwards
	oscTimerPeriod = 120'000'000/OscResonantFrequency;
	const uint32_t oscMarkCount = (uint32_t)((float)oscTimerPeriod * OscMarkSpaceRatio);

	volatile Tcc *const tccosc = Timers::TccDevices[InductiveHeaterOscTccDeviceNumber];
	hri_tcc_clear_CTRLA_ENABLE_bit(tccosc);
	hri_tcc_set_CTRLA_SWRST_bit(tccosc);
	tccosc->CTRLA.bit.PRESCALER = oscPrescaler;
	tccosc->CTRLA.bit.RESOLUTION = 0;
	hri_tcc_write_WAVE_WAVEGEN_bf(tccosc, TCC_WAVE_WAVEGEN_NPWM_Val);
	tccosc->PERBUF.bit.PERBUF = oscTimerPeriod - 1;
	tccosc->PER.bit.PER = oscTimerPeriod - 1;
	tccosc->CCBUF[InductiveHeaterOscTccOutputNumber].bit.CCBUF = oscMarkCount - 1;
	tccosc->CC[InductiveHeaterOscTccOutputNumber].bit.CC = oscMarkCount - 1;

	hri_tcc_set_CTRLA_ENABLE_bit(tccosc);

	// Set up the PWM TCC to generate the PWM.
	// We sync it to the oscillator TCC to avoid short pulses at the start or end of a PWM period.
	// To do this we use the same GCLK and prescaler as the oscillator TCC and just multiple the period by a suitable integer.
	// Then we always set the counter match value to a multiple of the oscillator period. Because of this, we need a 24-bit TCC.
	EnableTccClock(InductiveHeaterPwmTccDeviceNumber, GclkNum120MHz);
	const uint32_t pwmPrescaler = 0;
	pwmTimerPeriod = oscTimerPeriod * PwmFrequencyDivisor;

	volatile Tcc *const tccpwm = Timers::TccDevices[InductiveHeaterPwmTccDeviceNumber];
	hri_tcc_clear_CTRLA_ENABLE_bit(tccpwm);
	hri_tcc_set_CTRLA_SWRST_bit(tccpwm);

	tccpwm->CTRLA.bit.PRESCALER = pwmPrescaler;
	tccpwm->CTRLA.bit.RESOLUTION = 0;
	hri_tcc_write_WAVE_WAVEGEN_bf(tccpwm, TCC_WAVE_WAVEGEN_NPWM);

	tccpwm->PERBUF.bit.PERBUF = pwmTimerPeriod - 1;
	tccpwm->PER.bit.PER = pwmTimerPeriod - 1;
	tccpwm->CCBUF[InductiveHeaterPwmTccOutputNumber].bit.CCBUF = 0;
	tccpwm->CC[InductiveHeaterPwmTccOutputNumber].bit.CC = 0;

	hri_tcc_set_CTRLA_ENABLE_bit(tccpwm);

	// Retrigger them both to synchronise them.
	{
		AtomicCriticalSectionLocker lock;
		tccpwm->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;					// if we don't do this then there is a delay before PWM starts
		tccosc->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;					// if we don't do this then there is a delay before PWM starts
	}

	// Set up the CCL to gate them together
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_CCL;								// enable the CCL APB clock
	CCL->CTRL.reg = 0;														// SAME5x errata: the LUT config registers are enable-protected by the global enable bit

	// Currently we don't need to provide a clock to the CCL because we don't use input events, a filter, edge detection or sequential logic.
	// If we do start using any of these then we need to provide the CCL with a clock, max 100MHz.

	// Set up CCL0 to just copy TCC0 WO[0] to the output so that we can use it as an input to CCL3
	constexpr uint32_t Lut0RegValue =  CCL_LUTCTRL_TRUTH(1u << 1)						// just copy INSEL0
									| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_TCC_Val)	// INSEL0 from TCC output 0
									| CCL_LUTCTRL_INSEL1(CCL_LUTCTRL_INSEL1_MASK_Val)	// INSEL1 not used
									| CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL2_MASK_Val);	// INSEL2 not used
	CCL->LUTCTRL[InductiveHeaterAuxCCLNumber].reg = Lut0RegValue;
	CCL->LUTCTRL[InductiveHeaterAuxCCLNumber].reg = Lut0RegValue | CCL_LUTCTRL_ENABLE;

	// Set up CCL3 to AND the TCC3 WO[1] and the output from CCL0 together
	constexpr uint32_t Lut3RegValue = CCL_LUTCTRL_TRUTH(1u << 3)						// AND of INSEL0 and INSEL11
									| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_TCC_Val)	// INSEL0 from TCC3.0 output
									| CCL_LUTCTRL_INSEL1(CCL_LUTCTRL_INSEL1_LINK_Val)	// INSEL1 from CCL1
									| CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL2_MASK_Val);	// INSEL2 not used
	CCL->LUTCTRL[InductiveHeaterCCLNumber].reg = Lut3RegValue;
	CCL->LUTCTRL[InductiveHeaterCCLNumber].reg = Lut3RegValue | CCL_LUTCTRL_ENABLE;
	CCL->CTRL.reg = CCL_CTRL_ENABLE;													// SAME5x errata: the LUT config registers are enable-protected by the global enable bit

	// Finally, set the FET drive pin to be the CCL3 output
	SetPinFunction(InductiveHeaterCCLOutPin, InductiveHeaterCCLOutPinPeriphMode);
}

// Set the PWM value in the range 0..1
void InductiveHeaterPort::SetPwm(float pwm) noexcept
{
	const uint32_t ccIdeal = (uint32_t)(pwm * (float)pwmTimerPeriod);
	const uint32_t cc = ccIdeal - (ccIdeal % oscTimerPeriod);
	volatile Tcc *const tccdev = Timers::TccDevices[InductiveHeaterPwmTccDeviceNumber];
	tccdev->CCBUF[InductiveHeaterPwmTccOutputNumber].bit.CCBUF = cc;
}

///////////////////////// Heater calibration constants and functions ////////////////////////////////

// Heater on-time tuning constants
constexpr uint32_t TuneOnFirstSeed = 180;			// safe starting floor
constexpr uint32_t TuneOnStep = 2;					// ramp granularity
constexpr uint32_t TuneOnMax = 1080;

// Provisional OFF used while ramping ON: long enough to contain a full positive
// resonance halfcycle even at the lowest resonant frequency.
constexpr uint32_t TuneOffProv = 840;

// OFF cycles fired after the last measured cycle so its resonance completes
// before the timer stops.
constexpr uint32_t TuneTrailingOff = 1;

// Quiet time between bursts, long enough for the tank to fully stop resonating.
constexpr uint32_t TunsSettleMicroseconds = 350;

// Whole-tune watchdog. A full sweep is typically well under ~2 s, so this is just a timeout to ensure that session doesn't hang in case of hardware faults etc.
constexpr uint32_t TuneTotalTimeoutMicroseconds = 10000000;		// 10 s

// Waveform sweep parameters for OFF time tuning. Must be long enough to encompass the full on+off cycle to ensure we can capture the full positive resonance halfcycle.
constexpr uint32_t TuneSampleStep = 8;
constexpr uint32_t TuneNumSamples = (TuneOnMax + TuneOffProv) / TuneSampleStep;

// Margin when finding the resonance cycle in the sampled data.
constexpr float TuneZeroMarginV = 1.0;

// Waveform-dump pacing interval, to reduce risk of lost messages.
constexpr uint32_t TuneDumpIntervalMicroseconds = 3000;

// The AC provides 64 trigger levels. Define the values that indicate over target and over maximum.
constexpr float TargetVoltage = 96.0;
constexpr float OverVoltage = 100.0;
constexpr uint32_t TargetVoltageACValue = (uint32_t)(64.0 * TargetVoltage/InductiveHeaterVoltageFeedbackRange);
constexpr uint32_t OverVoltageACValue =  (uint32_t)(64.0 * OverVoltage/InductiveHeaterVoltageFeedbackRange);

static_assert(OverVoltageACValue <= 63);
static_assert(TargetVoltageACValue < OverVoltageACValue);

// Start calibrating the heater
// Returns GCodeResult::notFinished if we started, else GCodeResult::error with an error message in 'reply' if we couldn't start calibrating
GCodeResult InductiveHeaterPort::StartCalibration(const StringRef& reply) noexcept
{
	SetPwm(0.0);
	if (calibrationTask == nullptr)
	{
		calibrationTask = new Task<CalibrationTaskStackWords>;
		calibrationTask->Create(CalibrationTaskEntry, "IndCalib", (void*)this, TaskPriority::InductiveHeaterCalibration);
	}
	calState = CalibrationState::start;
	calibrationTask->Give(NotifyIndices::InductiveHeaterCalibration);
	return GCodeResult::notFinished;
}

// Check whether heater calibration is complete
// Return GCodeResult::notFinished if calibration still in progress, GCodeResult::ok if finished,
// or GCodeResult::error with an error message in 'reply' if calibration failed.
GCodeResult InductiveHeaterPort::CheckCalibrationComplete(const StringRef& reply) noexcept
{
	switch (calState)
	{
	case CalibrationState::calibrating:
	case CalibrationState::start:
		return GCodeResult::notFinished;

	case CalibrationState::complete:
		calState = CalibrationState::idle;
		return GCodeResult::ok;		//TODO check for errors

	case CalibrationState::idle:
	default:
		reply.copy("unexpected heater calibration state");
		return GCodeResult::error;
	}
}

// Initial entry point of the calibration task
/*static*/ void InductiveHeaterPort::CalibrationTaskEntry(void *pv) noexcept
{
	((InductiveHeaterPort*)pv)->CalibrationTaskFunc();
}

// Member function that the calibration task executes
void InductiveHeaterPort::CalibrationTaskFunc() noexcept
{
	for (;;)
	{
		if (calState == CalibrationState::start)
		{
			calState = CalibrationState::calibrating;
			CalibrateHeater();
			calState = CalibrationState::complete;
		}
		TaskBase::TakeIndexed(NotifyIndices::InductiveHeaterCalibration);
	}
}

static std::atomic<uint32_t> acIntflag(0);

// Analog comparator interrupt handler
extern "C" void AC_Handler()
{
	const uint32_t intFlag = AC->INTFLAG.reg;
    acIntflag |= intFlag;
    AC->INTFLAG.reg = intFlag;
    //TODO do we need to take any other action on overvoltage?
}

static void SetupAnalogComparator() noexcept
{
	// Enable clocks for the AC
	hri_mclk_set_APBCMASK_AC_bit(MCLK);
	GCLK->PCHCTRL[AC_GCLK_ID].reg = GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN;

	// Set the AC input pin function
	SetPinFunction(InductiveHeaterVoltageFeedbackAcPin, GpioPinFunction::B);

    AC->CTRLA.bit.SWRST = 1;
    while (AC->SYNCBUSY.bit.SWRST) { }

    // COMP0: overvoltage detector, interrupt triggers on rising edge which indicates over voltage.
    AC->SCALER[0].reg = AC_SCALER_VALUE(OverVoltageACValue);
    AC->COMPCTRL[0].reg = AC_COMPCTRL_MUXPOS_PIN2 | AC_COMPCTRL_MUXNEG_VSCALE |
                          AC_COMPCTRL_SPEED_HIGH | AC_COMPCTRL_INTSEL_RISING |
                          AC_COMPCTRL_HYSTEN | AC_COMPCTRL_HYST_HYST50 |
                          AC_COMPCTRL_ENABLE;
    while (AC->SYNCBUSY.bit.COMPCTRL0) { }

    // COMP1: tuner peak detector. Latches rising edge, not used in normal driving. No hysteresis as that would bias the detect peak high.
    AC->SCALER[1].reg = AC_SCALER_VALUE(TargetVoltageACValue);
    AC->COMPCTRL[1].reg = AC_COMPCTRL_MUXPOS_PIN2 | AC_COMPCTRL_MUXNEG_VSCALE |
                          AC_COMPCTRL_SPEED_HIGH | AC_COMPCTRL_INTSEL_RISING |
                          AC_COMPCTRL_ENABLE;
    while (AC->SYNCBUSY.bit.COMPCTRL1) { }

//	AC->EVCTRL.reg = AC_EVCTRL_COMPEO0;								// COMP0 state sent to event output
    AC->INTENSET.reg = AC_INTENSET_COMP0;							// COMP0 rising edge interrupt is always enabled to detect overvoltage

    NVIC_SetPriority(AC_IRQn, NvicPriorityAC);
    NVIC_EnableIRQ(AC_IRQn);

    AC->CTRLA.reg = AC_CTRLA_ENABLE;
    while (AC->SYNCBUSY.bit.ENABLE) { }
}

// This function is called by the calibration task to calibrate the heater.
void InductiveHeaterPort::CalibrateHeater() noexcept
{
	// 1. Calibrate the first cycle length. This is the cycle length that just reaches the target peak voltage when sent as an isolated cycle.
	// Set up the analog comparator to interrupt when the target peak voltage is reached.
	SetupAnalogComparator();
	acIntflag.store(0);
    AC->INTENSET.reg = AC_INTENSET_COMP1;							// enable interrupt on COMP1 rising edge

	//TODO
	// 2. Calibrate the subsequent cycle length.
	//TODO
	// 3. Calibrate the off-time
	//TODO
}

#endif

// End
