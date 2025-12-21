/*
 * InductiveHeater.cpp
 *
 *  Created on: 21 Dec 2025
 *      Author: David
 *
 *  Support for the inductive heater in the Bondtech INDX system
 */

#include "InductiveHeater.h"

#if SUPPORT_INDUCTIVE_HEATER

#include <Timers.h>

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

InductiveHeater::InductiveHeater() noexcept
{
	// Nothing needed here yet, we do the work in the Init function
}

void InductiveHeater::Init() noexcept
{
	// Set up the oscillator TC to generate a square wave at the coil resonant frequency
	EnableTcClock(InductiveHeaterOscTcDeviceNumber, Timers::TcGclkNum);
	const uint32_t oscPrescaler = Timers::ChoosePrescaler(ResonantFrequency, 16, oscTimerTop);

	volatile Tc *const tcdev = Timers::TcDevices[InductiveHeaterOscTcDeviceNumber];
	hri_tc_clear_CTRLA_ENABLE_bit(tcdev);
	hri_tc_set_CTRLA_SWRST_bit(tcdev);
	tcdev->COUNT16.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT16_Val;
	tcdev->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MPWM_Val;
	tcdev->COUNT16.CTRLA.bit.PRESCALER = oscPrescaler;
	tcdev->COUNT16.CC[0].bit.CC = oscTimerTop;
	tcdev->COUNT16.CCBUF[0].bit.CCBUF = oscTimerTop;
	tcdev->COUNT16.CC[InductiveHeaterOscTcOutputNumber].bit.CC = oscTimerTop/2;				// 50% square wave
	tcdev->COUNT16.CCBUF[InductiveHeaterOscTcOutputNumber].bit.CCBUF = oscTimerTop/2;		// 50% square wave
	hri_tc_set_CTRLA_ENABLE_bit(tcdev);
	tcdev->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;

	// Set up the PWM TCC to generate the PWM modulation, preferably synchronised to the oscillator TC
	EnableTccClock(InductiveHeaterPwmTccDeviceNumber, Timers::TcGclkNum);
	volatile Tcc *const tccdev = Timers::TccDevices[InductiveHeaterPwmTccDeviceNumber];
	hri_tcc_clear_CTRLA_ENABLE_bit(tccdev);
	hri_tcc_set_CTRLA_SWRST_bit(tccdev);

	// Temporarily set up the PWM timer not synced to the osc timer
	// TODO sync it by counting osc timer events
	const uint32_t pwmPrescaler = Timers::ChoosePrescaler(ResonantFrequency/PwmFrequencyDivisor, Timers::TccCounterBits[InductiveHeaterPwmTccDeviceNumber], pwmTimerTop);
	tccdev->CTRLA.bit.PRESCALER = pwmPrescaler;
	tccdev->CTRLA.bit.RESOLUTION = 0;
	hri_tcc_write_WAVE_WAVEGEN_bf(tccdev, TCC_WAVE_WAVEGEN_NPWM_Val);

	tccdev->PERBUF.bit.PERBUF = pwmTimerTop;
	tccdev->PER.bit.PER = pwmTimerTop;

	tccdev->CCBUF[InductiveHeaterPwmTccOutputNumber].bit.CCBUF = 0;			// turn output off for now (does this work?)
	tccdev->CC[InductiveHeaterPwmTccOutputNumber].bit.CC = 0;				// turn output off for now (does this work?)

	// Set up the CCL to gate them together
	//TODO
	SetPinFunction(InductiveHeaterCCLOutPin, InductiveHeaterCCLOutPinPeriphMode);

#if 1
	// TEST: set fixed PWM value
	const float val = 0.2;
	SetPwm(val);
#endif
}

// Set the PWM value in the range 0..1
void InductiveHeater::SetPwm(float pwm) noexcept
{
	const uint32_t cc = Timers::ConvertRange(pwm, pwmTimerTop);
	volatile Tcc *const tccdev = Timers::TccDevices[InductiveHeaterPwmTccDeviceNumber];
	tccdev->CCBUF[InductiveHeaterPwmTccOutputNumber].bit.CCBUF = cc;
}

#endif

// End
