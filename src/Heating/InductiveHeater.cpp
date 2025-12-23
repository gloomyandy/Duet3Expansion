/*
 * InductiveHeater.cpp
 *
 *  Created on: 21 Dec 2025
 *      Author: David
 *
 *  Support for inductive heaters with resonant frequency in the 100 to 400kHz range
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

#define USE_TCC_OSC		(1)

InductiveHeater::InductiveHeater() noexcept
{
	// Nothing needed here yet, we do the work in the Init function
}

void InductiveHeater::Init() noexcept
{
#if USE_TCC_OSC
	{
		// Set up the oscillator TCC to generate a square wave at the coil resonant frequency
		EnableTccClock(InductiveHeaterOscTccDeviceNumber, GclkNum120MHz);
		const uint32_t oscPrescaler = 0;									// 16-or 24-bit TCC with a 120MHz clock and prescaler 1 gives us frequencies from 1.8kHz upwards
		oscTimerPeriod = 120'000'000/OscResonantFrequency;
		const uint32_t oscMarkCount = (uint32_t)((float)oscTimerPeriod * OscMarkSpaceRatio);

		volatile Tcc *const tccdev = Timers::TccDevices[InductiveHeaterOscTccDeviceNumber];
		hri_tcc_clear_CTRLA_ENABLE_bit(tccdev);
		hri_tcc_set_CTRLA_SWRST_bit(tccdev);
		// Warning: the following code is FRAGILE. The TCC is prone to hanging up if the following code is changed, for example adding wait_for_sync calls.
		tccdev->CTRLA.bit.PRESCALER = oscPrescaler;
		tccdev->CTRLA.bit.RESOLUTION = 0;
		hri_tcc_write_WAVE_WAVEGEN_bf(tccdev, TCC_WAVE_WAVEGEN_NPWM_Val);
		tccdev->PERBUF.bit.PERBUF = oscTimerPeriod - 1;
		tccdev->PER.bit.PER = oscTimerPeriod - 1;
		tccdev->CCBUF[InductiveHeaterOscTccOutputNumber].bit.CCBUF = oscMarkCount - 1;
		tccdev->CC[InductiveHeaterOscTccOutputNumber].bit.CC = oscMarkCount - 1;
		hri_tcc_set_CTRLA_ENABLE_bit(tccdev);
		tccdev->CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;					// if we don't do this then there is a delay before PWM starts
	}
#else
	{
		// Set up the oscillator TC to generate a square wave at the coil resonant frequency
		// We use an 8-bit counter because only WO[0] can be fed into the CCL, and only 8-bit mode has a period register separate from CC0
		ConfigureGclk(GclkNum25MHz, (GclkSource)((uint8_t)GclkSource::xosc0 + AppGetXoscNumber()), 1, true);
		EnableTcClock(InductiveHeaterOscTcDeviceNumber, GclkNum25MHz);
		const uint32_t oscPrescaler = 0;									// 8-bit TC with a 25MHz clock and prescaler 1 gives us frequencies from 98kHz upwards
		oscTimerPeriod = 25'000'000/OscResonantFrequency;
		const uint8_t oscMarkCount = (uint8_t)(oscTimerPeriod * OscMarkSpaceRatio);

		volatile Tc *const tcdev = Timers::TcDevices[InductiveHeaterOscTcDeviceNumber];
		hri_tc_clear_CTRLA_ENABLE_bit(tcdev);
		hri_tc_set_CTRLA_SWRST_bit(tcdev);
		tcdev->COUNT8.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT8_Val;
		tcdev->COUNT8.CTRLA.bit.PRESCALER = oscPrescaler;
		tcdev->COUNT8.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_NPWM;
		tcdev->COUNT8.PER.bit.PER = oscTimerPeriod - 1;
		tcdev->COUNT8.PERBUF.bit.PERBUF = oscTimerPeriod - 1;
		tcdev->COUNT8.CC[0].bit.CC = oscMarkCount - 1;
		tcdev->COUNT8.CCBUF[0].bit.CCBUF = oscMarkCount - 1;
		hri_tc_set_CTRLA_ENABLE_bit(tcdev);
		tcdev->COUNT8.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;				// if we don't do this then there is a delay before PWM starts
	}
#endif

	// Set up the PWM TCC to generate the PWM, preferably synchronised to the oscillator TC
	{
		EnableTccClock(InductiveHeaterPwmTccDeviceNumber, Timers::TcGclkNum);
		volatile Tcc *const tccdev = Timers::TccDevices[InductiveHeaterPwmTccDeviceNumber];
		hri_tcc_clear_CTRLA_ENABLE_bit(tccdev);
		hri_tcc_set_CTRLA_SWRST_bit(tccdev);

		// Temporarily set up the PWM timer not synced to the osc timer
		// TODO sync it by counting osc timer events
		const uint32_t pwmPrescaler = Timers::ChoosePrescaler(OscResonantFrequency/PwmFrequencyDivisor, Timers::TccCounterBits[InductiveHeaterPwmTccDeviceNumber], pwmTimerTop);
		tccdev->CTRLA.bit.PRESCALER = pwmPrescaler;
		tccdev->CTRLA.bit.RESOLUTION = 0;
		hri_tcc_write_WAVE_WAVEGEN_bf(tccdev, TCC_WAVE_WAVEGEN_NPWM);

		tccdev->PERBUF.bit.PERBUF = pwmTimerTop;
		tccdev->PER.bit.PER = pwmTimerTop;

		tccdev->CCBUF[InductiveHeaterPwmTccOutputNumber].bit.CCBUF = 0;		// turn output off for now (does this work?)
		tccdev->CC[InductiveHeaterPwmTccOutputNumber].bit.CC = 0;			// turn output off for now (does this work?)
		delayMicroseconds(10);												// without a delay here the programming doesn't work when using a high GCLK frequency
		hri_tcc_set_CTRLA_ENABLE_bit(tccdev);
		tccdev->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;					// if we don't do this then there is a delay before PWM starts
	}

	// Set up the CCL to gate them together
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_CCL;								// enable the CCL APB clock
//	MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;								// enable the event system APB clock
#if 0	// not needed unless we use edge detection in the CCL
	GCLK->PCHCTRL[CCL_GCLK_ID].reg = GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN;
#endif
	CCL->CTRL.reg = 0;														// SAME5x errata: the LUT config registers are enable-protected by the global enable bit

#if USE_TCC_OSC
	constexpr uint32_t Lut0RegValue =  CCL_LUTCTRL_TRUTH(1u << 1)						// just copy INSEL0
									| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_TCC_Val)	// INSEL0 from TCC output 0
									| CCL_LUTCTRL_INSEL1(CCL_LUTCTRL_INSEL1_MASK_Val)	// INSEL1 not used
									| CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL2_MASK_Val);	// INSEL2 not used
	CCL->LUTCTRL[InductiveHeaterAuxCCLNumber].reg = Lut0RegValue;
	CCL->LUTCTRL[InductiveHeaterAuxCCLNumber].reg = Lut0RegValue | CCL_LUTCTRL_ENABLE;
	constexpr uint32_t Lut3RegValue = CCL_LUTCTRL_TRUTH(1u << 3)						// AND of INSEL0 and INSEL11
									| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_LINK_Val)	// INSEL0 from CCL0 output
									| CCL_LUTCTRL_INSEL1(CCL_LUTCTRL_INSEL1_TCC_Val)	// INSEL1 from TCC3.1
									| CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL2_MASK_Val);	// INSEL2 not used
#else
	constexpr uint32_t Lut3RegValue = CCL_LUTCTRL_TRUTH(1u << 3)						// AND of INSEL0 and INSEL11
									| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_TC_Val)		// INSEL0 from TC3.0
									| CCL_LUTCTRL_INSEL1(CCL_LUTCTRL_INSEL1_TCC_Val)	// INSEL1 from TCC3.1
									| CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL2_MASK_Val);	// INSEL2 not used
#endif
	CCL->LUTCTRL[InductiveHeaterCCLNumber].reg = Lut3RegValue;
	CCL->LUTCTRL[InductiveHeaterCCLNumber].reg = Lut3RegValue | CCL_LUTCTRL_ENABLE;
	CCL->CTRL.reg = CCL_CTRL_ENABLE;										// SAME5x errata: the LUT config registers are enable-protected by the global enable bit

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
