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
#include <Math/DeviationAccumulator.h>
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

// The AC provides 64 trigger levels. Define the values that indicate over target and over maximum.
constexpr float TargetVoltage = 80.0;
constexpr float OverVoltage = 100.0;
constexpr uint32_t TargetVoltageACValue = (uint32_t)(64.0 * TargetVoltage/InductiveHeaterVoltageFeedbackRange) - 1;
constexpr uint32_t OverVoltageACValue =  (uint32_t)(64.0 * OverVoltage/InductiveHeaterVoltageFeedbackRange) - 1;

static_assert(OverVoltageACValue <= 63);
static_assert(TargetVoltageACValue < OverVoltageACValue);

Task<InductiveHeaterPort::CalibrationTaskStackWords> *_ecv_null InductiveHeaterPort::calibrationTask = nullptr;

InductiveHeaterPort::InductiveHeaterPort()
{
	// Nothing to do here
}

void InductiveHeaterPort::Init() noexcept
{
	SetPinMode(InductiveHeaterCCLOutPin, OUTPUT_LOW);							// turn the FET driver off to avoid glitches

	// Set up the CCL to gate the oscillator and the PWM timer together
	{
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_CCL;								// enable the CCL APB clock
		CCL->CTRL.reg = 0;														// SAME5x errata: the LUT config registers are enable-protected by the global enable bit

		// Currently we don't need to provide a clock to the CCL because we don't use input events, a filter, edge detection or sequential logic.
		// If we do start using any of these then we need to provide the CCL with a clock, max 100MHz.

		// Set up CCL0 to just copy TCC0 WO[0] to the output so that we can use it as an input to CCL3
		constexpr uint32_t Lut0RegValue =  CCL_LUTCTRL_TRUTH(1u << 1)						// just copy INSEL0 to output
										| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_TCC_Val)	// INSEL0 from TCC output 0 (PWM timer)
										| CCL_LUTCTRL_INSEL1(CCL_LUTCTRL_INSEL1_MASK_Val)	// INSEL1 not used
										| CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL2_MASK_Val);	// INSEL2 not used
		CCL->LUTCTRL[InductiveHeaterAuxCCLNumber].reg = Lut0RegValue;
		CCL->LUTCTRL[InductiveHeaterAuxCCLNumber].reg = Lut0RegValue | CCL_LUTCTRL_ENABLE;

		// Set up CCL3 to AND the TCC3 WO[1] and the output from CCL0 together
		constexpr uint32_t Lut3RegValue = CCL_LUTCTRL_TRUTH(1u << 3)						// AND of INSEL0 and INSEL11
										| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_TCC_Val)	// INSEL0 from TCC3.0 output (oscillator timer)
										| CCL_LUTCTRL_INSEL1(CCL_LUTCTRL_INSEL1_LINK_Val)	// INSEL1 from CCL1 (inverted input from PWM timer)
										| CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL2_MASK_Val);	// INSEL2 not used
		CCL->LUTCTRL[InductiveHeaterCCLNumber].reg = Lut3RegValue;
		CCL->LUTCTRL[InductiveHeaterCCLNumber].reg = Lut3RegValue | CCL_LUTCTRL_ENABLE;
		CCL->CTRL.reg = CCL_CTRL_ENABLE;													// SAME5x errata: the LUT config registers are enable-protected by the global enable bit
	}

	// Set up the analog comparator to detect the peak voltage on the FET drain
	{
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_AC;
		GCLK->PCHCTRL[AC_GCLK_ID].reg = GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN;

		AC->CTRLA.bit.SWRST = 1;
		while (AC->SYNCBUSY.bit.SWRST) { }

		// Set the AC input pin function
		SetPinFunction(InductiveHeaterVoltageFeedbackAcPin, GpioPinFunction::B);

		AC->CALIB.reg = (*reinterpret_cast<const uint32_t*>(AC_FUSES_BIAS0_ADDR) & AC_FUSES_BIAS0_Msk) >> AC_FUSES_BIAS0_Pos;	// set the calibration value

		// COMP0: overvoltage detector, interrupt triggers on rising edge which indicates over voltage
		AC->SCALER[0].reg = AC_SCALER_VALUE(OverVoltageACValue);
		AC->COMPCTRL[0].reg = AC_COMPCTRL_MUXPOS_PIN2 | AC_COMPCTRL_MUXNEG_VSCALE |
							  AC_COMPCTRL_SPEED_HIGH | AC_COMPCTRL_INTSEL_RISING | AC_COMPCTRL_FLEN(1) |
							  AC_COMPCTRL_HYSTEN | AC_COMPCTRL_HYST_HYST50 |
							  AC_COMPCTRL_ENABLE;
		while (AC->SYNCBUSY.bit.COMPCTRL0) { }

#if 0
		// COMP1: tuner peak detector. Latches rising edge, not used in normal driving
		AC->SCALER[1].reg = AC_SCALER_VALUE(TargetVoltageACValue);
		AC->COMPCTRL[1].reg = AC_COMPCTRL_MUXPOS_PIN2 | AC_COMPCTRL_MUXNEG_VSCALE |
							  AC_COMPCTRL_SPEED_HIGH | AC_COMPCTRL_INTSEL_RISING | AC_COMPCTRL_FLEN(1) |
//							  AC_COMPCTRL_HYSTEN | AC_COMPCTRL_HYST_HYST50 |
							  AC_COMPCTRL_ENABLE;
		while (AC->SYNCBUSY.bit.COMPCTRL1) { }
#endif

		AC->EVCTRL.reg = AC_EVCTRL_COMPEO0 | AC_EVCTRL_COMPEO1;					// both channels generate output events

		AC->CTRLA.reg = AC_CTRLA_ENABLE;
		while (AC->SYNCBUSY.bit.ENABLE) { }

	   // Wait for both comparators to become ready before enabling interrupts, see SAME5x errata 2.2.2
		while ((AC->STATUSB.reg & AC_STATUSB_READY0) != AC_STATUSB_READY0) { }

		// Set up the event system to generate events on the comparators triggering
		MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;								// enable the EVSYS APB clock

		// The user multiplexer must be configured before the channel (datasheet section 31.5.2.3)
		// The definition of EVSYS_USER in the DFP is a 32-bit type whereas the datasheet says it as an 8-bit type. This affects the indexing.
#if 0	// If we assume the datasheet is correct:
		volatile uint8_t *const evsysUser = (volatile uint8_t *)(&EVSYS->USER);
		evsysUser[InductiveHeaterOscTccCaptureEventUserNumber] = (AcEvent + 1) + 1;			// route channel (AcEvent + 1) events to oscillator TC capture 1
#else	// If we assume the DFP is correct:
		EVSYS->USER[InductiveHeaterOscTccCaptureEventUserNumber].reg = (AcEvent + 1) + 1;	// route channel (AcEvent + 1) events to oscillator TC capture 1
#endif
//		EVSYS->Channel[AcEvent].CHANNEL.reg = EVSYS_CHANNEL_EVGEN(0x6B) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT;		// route comparator 0 event to event channel AcEvent
		// Erratum 2.21.1: "TCC peripheral is not compatible with an EVSYS channel in SYNC or RESYNC mode.
		//                  Workaround: Use TCC with an EVSYS channel in ASYNC mode."
		EVSYS->Channel[AcEvent + 1].CHANNEL.reg = EVSYS_CHANNEL_EVGEN(0x6C) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT;	// route comparator 1 event to event channel AcEvent + 1
		AC->INTENSET.reg = AC_INTENSET_COMP0;									// COMP0 rising edge interrupt is always enabled to detect overvoltage

		NVIC_SetPriority(AC_IRQn, NvicPriorityAC);
		NVIC_EnableIRQ(AC_IRQn);
	}

	// Set the oscillator to use the default values, or (TODO)values fetched from NVRAM
	{
		EnableTccClock(InductiveHeaterOscTccDeviceNumber, GclkNum120MHz);		// use the 120MHz GCLK to get the best timing resolution
		EnableTccClock(InductiveHeaterPwmTccDeviceNumber, GclkNum120MHz);
		SetupOscillator(0x00FFFFFF);											// this finishes by setting the FET to be driven from the CCL output
	}
}

// Set up the timing parameters from firstCycleLength, laterCycleLength and offTime. This is called during initialisation and during tuning.
// pwmOnCout is the point at which we should start applying PWM. A value of 0x00FFFFFF (or any value other greater than pwmTimerPeriod) keeps the heater off.
void InductiveHeaterPort::SetupOscillator(uint32_t pwmOnCount) noexcept
{
	SetPinMode(InductiveHeaterCCLOutPin, OUTPUT_LOW);							// turn the FET driver off to avoid glitches

	// Set up the oscillator TCC to generate output with the required on (except first pulse) and off times
	const uint32_t oscPrescaler = 0;											// 16-or 24-bit TCC with a 120MHz clock and prescaler 1 gives us frequencies from 1.8kHz upwards
	const uint32_t oscTimerPeriod = mainOnTime + offTime;

	volatile Tcc *const tccosc = Timers::TccDevices[InductiveHeaterOscTccDeviceNumber];
	hri_tcc_clear_CTRLA_ENABLE_bit(tccosc);
	hri_tcc_set_CTRLA_SWRST_bit(tccosc);

	tccosc->CTRLA.reg = TCC_CTRLA_PRESCALER(oscPrescaler) | (TCC_CTRLA_CPTEN0 << InductiveHeaterOscTccCaptureNumber);	// enable capture
	tccosc->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM_Val;
	hri_tcc_wait_for_sync(tccosc, TCC_SYNCBUSY_MASK);

	tccosc->PERBUF.reg = oscTimerPeriod - 1;
	tccosc->PER.reg = oscTimerPeriod - 1;
	tccosc->CCBUF[InductiveHeaterOscTccOutputNumber].reg = mainOnTime - 1;
	tccosc->CC[InductiveHeaterOscTccOutputNumber].reg = mainOnTime - 1;
	tccosc->EVCTRL.reg = (TCC_EVCTRL_MCEI0 << InductiveHeaterOscTccCaptureNumber);	// input event causes capture

	// Set up the PWM TCC to generate the PWM.
	// We sync it to the oscillator TCC to avoid short pulses at the start or end of a PWM period.
	// To do this we use the same GCLK and prescaler as the oscillator TCC and just multiply the period by a suitable integer.
	// Then we always set the counter match value to a multiple of the oscillator period. Because of this, we need a 24-bit TCC.
	const uint32_t pwmPrescaler = 0;
	pwmTimerPeriod = oscTimerPeriod * PwmFrequencyDivisor;

	volatile Tcc *const tccpwm = Timers::TccDevices[InductiveHeaterPwmTccDeviceNumber];
	hri_tcc_clear_CTRLA_ENABLE_bit(tccpwm);
	hri_tcc_set_CTRLA_SWRST_bit(tccpwm);

	tccpwm->CTRLA.reg = TCC_CTRLA_PRESCALER(pwmPrescaler);
	tccpwm->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM | (TCC_WAVE_POL0 << InductiveHeaterPwmTccOutputNumber);
	hri_tcc_wait_for_sync(tccpwm, TCC_SYNCBUSY_MASK);

	tccpwm->PERBUF.reg = pwmTimerPeriod - 1;
	tccpwm->PER.reg = pwmTimerPeriod - 1;
	tccpwm->CC[InductiveHeaterPwmTccOutputNumber].reg = pwmOnCount;
	tccpwm->CCBUF[InductiveHeaterPwmTccOutputNumber].reg = pwmOnCount;

	hri_tcc_set_CTRLA_ENABLE_bit(tccpwm);
	hri_tcc_set_CTRLA_ENABLE_bit(tccosc);

	// Retrigger them both to synchronise them.
	{
		AtomicCriticalSectionLocker lock;
		tccpwm->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;					// if we don't do this then there is a delay before PWM starts
		tccosc->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;					// if we don't do this then there is a delay before PWM starts
	}

	// Make sure that the PWM timer output is zero before we enable the output, otherwise the heater will be on at full power for a whole PWM cycle at the start
	while ((tccpwm->STATUS.reg & (TCC_STATUS_CMP0 << InductiveHeaterPwmTccOutputNumber)) != 0) { }

	// Set the FET drive pin to be the CCL3 output
	SetPinFunction(InductiveHeaterCCLOutPin, InductiveHeaterCCLOutPinPeriphMode);

	// Enable the oscillator TCC capture interrupt
	NVIC_SetPriority(OSC_TCC_IRQn, NvicPriorityOscTcc);
	NVIC_EnableIRQ(OSC_TCC_IRQn);
}

// Set the PWM value in the range 0..1. Used in normal operation.
void InductiveHeaterPort::SetPwm(float pwm) noexcept
{
	const uint32_t idealOnClocks = (uint32_t)(pwm * (float)pwmTimerPeriod);						// range is 0..pwmTimerPeriod
	const uint32_t oscTimerPeriod = mainOnTime + offTime;
	const uint32_t actualOnClocks = idealOnClocks - (idealOnClocks % oscTimerPeriod);			// range is still 0..pwmTimerPeriod
	const uint32_t cc = (actualOnClocks == 0) ? 0x00FFFFFF										// heater is off
						: (actualOnClocks == pwmTimerPeriod) ? 0								// heater is fully on
							: pwmTimerPeriod - actualOnClocks + (mainOnTime - firstOnTime);		// delay comparison to make the first cycle shorter than the rest
	volatile Tcc *const tccdev = Timers::TccDevices[InductiveHeaterPwmTccDeviceNumber];
	tccdev->CC[InductiveHeaterPwmTccOutputNumber].bit.CC = cc;
	tccdev->CCBUF[InductiveHeaterPwmTccOutputNumber].bit.CCBUF = cc;
}

// Reset the heater cycle parameters to the latest stored values and set a specified burst length.
// Used during heater calibration. burstLength is a small integer, at least 1.
void InductiveHeaterPort::SetBurst(uint32_t burstLength) noexcept
{
	const uint32_t nextPwmTimerPeriod = (mainOnTime + offTime) * PwmFrequencyDivisor;
	const uint32_t cc = nextPwmTimerPeriod - (burstLength - 1) * (mainOnTime + offTime) - (firstOnTime + offTime);
	SetupOscillator(cc);
}

// Turn the heater off
void InductiveHeaterPort::TurnOff() noexcept
{
	SetPinMode(InductiveHeaterCCLOutPin, OUTPUT_LOW);
}

///////////////////////// Heater calibration constants and functions ////////////////////////////////

// Provisional OFF used while ramping ON: long enough to contain a full positive
// resonance halfcycle even at the lowest resonant frequency.
//constexpr uint32_t TuneOffProv = 840;

// OFF cycles fired after the last measured cycle so its resonance completes
// before the timer stops.
//constexpr uint32_t TuneTrailingOff = 1;

// Quiet time between bursts, long enough for the tank to fully stop resonating.
//constexpr uint32_t TunsSettleMicroseconds = 350;

// Whole-tune watchdog. A full sweep is typically well under ~2 s, so this is just a timeout to ensure that session doesn't hang in case of hardware faults etc.
//constexpr uint32_t TuneTotalTimeoutMicroseconds = 10000000;		// 10 s

// Waveform sweep parameters for OFF time tuning. Must be long enough to encompass the full on+off cycle to ensure we can capture the full positive resonance halfcycle.
//constexpr uint32_t TuneSampleStep = 8;
//constexpr uint32_t TuneNumSamples = (TuneOnMax + TuneOffProv) / TuneSampleStep;

// Margin when finding the resonance cycle in the sampled data.
//constexpr float TuneZeroMarginV = 1.0;

// Waveform-dump pacing interval, to reduce risk of lost messages.
//constexpr uint32_t TuneDumpIntervalMicroseconds = 3000;

static DeviationAccumulator accumulator;

// Start calibrating the heater, or check whether heater calibration is complete
// Returns GCodeResult::notFinished if we started
//         GCodeResult::error with an error message in 'reply' if we couldn't start calibration or calibration failed
//		   GCodeResult::ok if finished with the calibration parameters in 'reply'
GCodeResult InductiveHeaterPort::Calibrate(bool start, const StringRef& reply) noexcept
{
	if (start)
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

	switch (calState)
	{
	case CalibrationState::calibrating:
	case CalibrationState::start:
		return GCodeResult::notFinished;

	case CalibrationState::success:
		calState = CalibrationState::idle;
		reply.printf("Calibration succeeded, parameters: %lu, %lu, %lu", firstOnTime, mainOnTime, offTime);
		reply.lcatf("num %u mean %.1f dev %.1f", accumulator.GetNumSamples(), (double)accumulator.GetMean(), (double)accumulator.GetDeviation());
		return GCodeResult::error;	//TEMP
		//return GCodeResult::ok;

	case CalibrationState::failed:
		calState = CalibrationState::idle;
		reply.copy(calibrationFailedReason);
		reply.lcatf("num %u mean %.1f dev %.1f", accumulator.GetNumSamples(), (double)accumulator.GetMean(), (double)accumulator.GetDeviation());
		return GCodeResult::error;

	default:
		calState = CalibrationState::idle;
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
			CalibrateHeater();
		}
		TaskBase::TakeIndexed(NotifyIndices::InductiveHeaterCalibration);
	}
}

static std::atomic<uint32_t> acIntflag(0);
static std::atomic<uint32_t> count(0);

// Analog comparator interrupt handler
extern "C" void AC_Handler() noexcept
{
	const uint32_t intFlag = AC->INTFLAG.reg;
//	if ((intFlag & acIntflag) & AC_INTFLAG_COMP1)
//	{
//		// This is the first COMP1 interrupt so log its time
//		volatile Tcc *const tccosc = Timers::TccDevices[InductiveHeaterOscTccDeviceNumber];
//		tccosc->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
//		while (tccosc->SYNCBUSY.bit.COUNT) { }
//		count = tccosc->COUNT.reg;
//	}
	acIntflag |= intFlag;
    //TODO do we need to take any other action on overvoltage?
	AC->INTFLAG.reg = intFlag;
}

#ifndef OSC_TCC_Handler
# error OSC_TCC_Handler not defined
#endif

static uint32_t captureBuffer[8];
static unsigned int captureIndex = 0;

extern "C" void OSC_TCC_Handler() noexcept
{
	volatile Tcc *const tccosc = Timers::TccDevices[InductiveHeaterOscTccDeviceNumber];
	const uint32_t intflag = tccosc->INTFLAG.reg;
	// Erratum 2.21.11: "In capture operation, MC0/MC1 interrupt status flags (INTFLAG.MC0/INTFLAG.MC1) are not automatically cleared when the CC0/CC1 register is read.
	//					 Workaround: MC0/MC1 interrupt status flags must be cleared by the software (INTFLAG.MC0 = 1/INTFLAG.MC1 = 1)."
	if (intflag & (TCC_INTFLAG_MC0 << InductiveHeaterOscTccCaptureNumber))
	{
		const uint32_t capturedValue = tccosc->CC[InductiveHeaterOscTccCaptureNumber].bit.CC;
		if (captureIndex < ARRAY_SIZE(captureBuffer))
		{
			captureBuffer[captureIndex++] = capturedValue;
		}
	}
	tccosc->INTFLAG.reg = intflag;
}

// Set comparator 1 to interrupt when the target voltage is exceeded
static void SetComp1Target() noexcept
{
	AC->COMPCTRL[1].reg = 0;										// disable comparator 1
	while (AC->SYNCBUSY.bit.COMPCTRL1) { }

	AC->SCALER[1].reg = AC_SCALER_VALUE(TargetVoltageACValue);
	AC->COMPCTRL[1].reg = AC_COMPCTRL_MUXPOS_PIN2 | AC_COMPCTRL_MUXNEG_VSCALE |
						  AC_COMPCTRL_SPEED_HIGH | AC_COMPCTRL_INTSEL_RISING | AC_COMPCTRL_FLEN(1) |
						  AC_COMPCTRL_HYSTEN | AC_COMPCTRL_HYST_HYST50 |
						  AC_COMPCTRL_ENABLE;
	while (AC->SYNCBUSY.bit.COMPCTRL1) { }
	while ((AC->STATUSB.reg & AC_STATUSB_READY1) != AC_STATUSB_READY1) { }
}

// Set comparator 1 to interrupt when the voltage goes below zero
static void SetComp1Zero() noexcept
{
	AC->COMPCTRL[1].reg = 0;										// disable comparator 1
	while (AC->SYNCBUSY.bit.COMPCTRL1) { }

	AC->SCALER[1].reg = AC_SCALER_VALUE(0);							// detect when voltage falls to 1/64 of VDDANA which is 1.6V at the mosfet drain
	AC->COMPCTRL[1].reg = AC_COMPCTRL_MUXPOS_PIN2 | AC_COMPCTRL_MUXNEG_VSCALE |
						  AC_COMPCTRL_SPEED_HIGH | AC_COMPCTRL_INTSEL_FALLING | AC_COMPCTRL_FLEN(1) |
						  AC_COMPCTRL_HYSTEN | AC_COMPCTRL_HYST_HYST50 |
						  AC_COMPCTRL_ENABLE;
	while (AC->SYNCBUSY.bit.COMPCTRL1) { }
	while ((AC->STATUSB.reg & AC_STATUSB_READY1) != AC_STATUSB_READY1) { }
}

// Start capturing the oscillator TCC count triggered by analog comparator COMP1
static void StartCapturing() noexcept
{
	captureIndex = 0;
	volatile Tcc *const tccosc = Timers::TccDevices[InductiveHeaterOscTccDeviceNumber];
	while (tccosc->INTFLAG.reg & (TCC_INTFLAG_MC0 << InductiveHeaterOscTccCaptureNumber))
	{
		(void)tccosc->CC;
		tccosc->INTFLAG.reg = (TCC_INTFLAG_MC0 << InductiveHeaterOscTccCaptureNumber);	// Erratum: reading CC does not clear the flag
	}
	tccosc->INTENSET.reg = (TCC_INTENSET_MC0 << InductiveHeaterOscTccCaptureNumber);
}

// Stop capturing the oscillator TCC count triggered by analog comparator COMP1
static void StopCapturing() noexcept
{
	volatile Tcc *const tccosc = Timers::TccDevices[InductiveHeaterOscTccDeviceNumber];
	tccosc->INTENCLR.reg = (TCC_INTENCLR_MC0 << InductiveHeaterOscTccCaptureNumber);
}

// This function is called by the calibration task to calibrate the heater.
void InductiveHeaterPort::CalibrateHeater() noexcept
{
	calState = CalibrationState::calibrating;

	// 1. Calibrate the first cycle length. This is the cycle length that just reaches the target peak voltage when sent as an isolated cycle.
	// Set up the analog comparator to interrupt when the target peak voltage is reached.
	TurnOff();														// must turn the heater off before messing with the parameters
	firstOnTime = OscMinFirstOnTime;								// set the default parameters
	mainOnTime = OscMinFirstOnTime;
	offTime = OscDefaultOffTime;

	SetComp1Target();												// set comparator to trigger when the target threshold was reached
	delay(2);														// may need this to allow VSCALE to settle
	unsigned int successCount = 0;
	constexpr unsigned int requiredSuccess = 3;

	for (;;)    													// loop increasing firstOnTime until the target is reached
	{
		// Set up the analog comparator channel 1 to detect when the target mosfet drain voltage is reached
		acIntflag.store(0);
		AC->INTFLAG.reg = AC_INTENSET_COMP1;						// clear any pending COMP1 interrupt
		AC->INTENSET.reg = AC_INTENSET_COMP1;						// enable interrupt on COMP1 edge

		// Command the heater to perform single-cycle bursts using the current parameters
		SetBurst(1);

		// Delay long enough for a few bursts to happen. Each burst is about 5ms long
		delay(25);
		AC->INTENCLR.reg = AC_INTENSET_COMP1;						// disable interrupt on COMP1 edge
		TurnOff();

		// Check whether the target first pulse height has been reached
		if (acIntflag.load() && AC_INTFLAG_COMP1)
		{
			++successCount;
			if (successCount == requiredSuccess) { break; }
		}
		else
		{
			if (firstOnTime >= OscMaxOnTime || (acIntflag.load() & AC_INTFLAG_COMP0))
			{
				calibrationFailedReason = "exceeded maximum first pulse length";
				calState = CalibrationState::failed;
				return;
			}

			// Increase the pulse length
			firstOnTime += OscOnTimeStep;
			mainOnTime = firstOnTime;
			successCount = 0;
		}
		delay(1);													// allow time for the heater coil to stop resonating
	}

	// Here when we have detected the target peak voltage in the first cycle
	if (firstOnTime - OscMinFirstOnTime < 10)						// we expect to have to increase the first on time above the minimum
	{
		calibrationFailedReason = "first pulse length too short, probably hardware error";
		calState = CalibrationState::failed;
		return;
	}

	// 2. Calibrate the off-time
//	DeviationAccumulator accumulator;
	accumulator.Clear();

	acIntflag.store(0);
	SetComp1Zero();													// set comparator to trigger on zero crossing
	delay(2);														// may need this to allow VSCALE to settle
	AC->INTFLAG.reg = AC_INTENSET_COMP1;							// clear any pending COMP1 interrupt (I think this is needed to prime the event output)
	AC->INTENSET.reg = AC_INTENSET_COMP1;							// enable interrupt on COMP1 rising edge
	StartCapturing();
	SetBurst(1);
	delay(25);
	StopCapturing();
	AC->INTENCLR.reg = AC_INTENSET_COMP1;							// disable interrupt on COMP1 rising edge
	TurnOff();

	if (captureIndex == 0)
	{
		calibrationFailedReason = "failed to capture zero crossings";
		calState = CalibrationState::failed;
		return;
	}

	debugPrintf("%u: %lu %lu", captureIndex, captureBuffer[0], captureBuffer[1]);
	if (captureIndex < 10)
	{
		calibrationFailedReason = "done capture";
		calState = CalibrationState::failed;
		return;
	}

#if 0
	if (accumulator.GetNumSamples() < 10)
	{
		calibrationFailedReason = "failed to detect zero crossings";
		calState = CalibrationState::failed;
		return;
	}

	if (accumulator.GetDeviation() * 20 > accumulator.GetMean())
	{
		calibrationFailedReason = "zero crossing deviation too high";
		calState = CalibrationState::failed;
		return;
	}
#endif
#if 0
	const uint32_t currentCycleTime = mainOnTime + offTime;
	const uint32_t measuredCycleTime = (uint32_t)(accumulator.GetMean() + accumulator.GetDeviation()) + 2;
	if (measuredCycleTime < currentCycleTime && measuredCycleTime > mainOnTime)
	{
		offTime = measuredCycleTime - mainOnTime;
		mainOnTime = currentCycleTime - offTime;
	}
#endif

	// 3. Calibrate the subsequent on-time.
	// As we are using a long off-time, on the second and subsequent cycles the mosfet will turn on while the coil is still feeding power back to the supply.

	firstOnTime -= 6;												// make sure the first pulse doesn't trigger the comparator
	SetComp1Target();
	successCount = 0;

	for (;;)    													// loop increasing mainOnTime until the target is reached
	{
		// Set up the analog comparator channel 1 to detect when the target mosfet drain voltage is reached
		acIntflag.store(0);
		AC->INTFLAG.reg = AC_INTENSET_COMP1;						// clear any pending COMP1 interrupt
		AC->INTENSET.reg = AC_INTENSET_COMP1;						// enable interrupt on COMP1 rising edge

		// As the second pulse can have a different amplitude from the third and later, use 3-cycle bursts
		SetBurst(3);

		// Delay long enough for a few bursts to happen. Each burst is about 5ms long
		delay(25);
		AC->INTENCLR.reg = AC_INTENSET_COMP1;						// disable interrupt on COMP1 rising edge
		TurnOff();

		// Check whether the target subsequent pulse height has been reached
		if (acIntflag.load() && AC_INTFLAG_COMP1)
		{
			++successCount;
			if (successCount == requiredSuccess) { break; }
		}
		else
		{
			if (mainOnTime >= OscMaxOnTime || (acIntflag.load() & AC_INTFLAG_COMP0))
			{
				calibrationFailedReason = "exceeded maximum subsequent pulse length";
				calState = CalibrationState::failed;
				return;
			}

			// Increase the pulse length
			mainOnTime += OscOnTimeStep;
			successCount = 0;
		}
		delay(1);													// allow time for the heater coil to stop resonating
	}

	firstOnTime += 6;									// restore the first pulse length to the value we fond earlier

	//TODO
	// 3. Calibrate the off-time
	//TODO

	calState = CalibrationState::success;
}

#endif

// End
