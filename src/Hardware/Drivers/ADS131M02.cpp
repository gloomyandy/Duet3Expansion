/*
 * ADS131M02.cpp
 *
 *  Created on: 3 Jan 2026
 *      Author: David
 */

#include "ADS131M02.h"

#if SUPPORT_ADS131M02

ADS131M02::ADS131M02() noexcept : SpiDevice(ADS131M02_SercomNumber, ADS131M02_DataInPad, ADS131M02_DataOutPad)
{
	initOk = false;

	SetPinMode(ADS131M02_CsPin, OUTPUT_HIGH);
	SetPinFunction(ADS131M02_MosiPin, ADS131M02_SpiPinFunction);
	SetPinFunction(ADS131M02_MisoPin, ADS131M02_SpiPinFunction);
	SetPinFunction(ADS131M02_SclkPin, ADS131M02_SpiPinFunction);
	SetPinMode(ADS131M02_DRDYPin, INPUT);
	ConfigureGclk(ADA131M02_GclkNumber, GclkSource::dpll0, 120/8, true);			// set up 8MHz clock
	SetPinFunction(ADS131M0_GclkPin, ADS131M0_GclkPinFunction);						// enable 8MHz clock on output pin

	// Reset the device
	if (!SendSimpleCommand(Ads131M02Command::reset)) return;
	delayMicroseconds(10);															// need to delay at least 5us after a reset
	if (!ReadRegister(Ads131M02Register::id)) return;								// get the response to reset and read the ID register
	if (rslt != 0xFF22) return;
	if (!SendSimpleCommand(Ads131M02Command::nullCmd)) return;
	if ((rslt >> 8) != 0x22) return;

	// Device has been reset and identified OK so initialise it
	if (!WriteRegister(Ads131M02Register::mode,  0b0000'0001'0000'0000)) return;	// 24 bit word length, no CRCs
	if (!WriteRegister(Ads131M02Register::clock, 0b0000'0011'0000'1110)) return;	// enable both channels, high resolution, oversampling 1024
	if (!WriteRegister(Ads131M02Register::gain,  0b0000'0000'0111'0111)) return;	// both channels gain 128
	if (!WriteRegister(Ads131M02Register::cfg,   0b0000'0110'0000'0000)) return;	// global chop delay disabled

	// We don't need to write the channel registers, the defaults are good
	initOk = true;
}

// Send a simple command, not a read or write register
bool ADS131M02::SendSimpleCommand(Ads131M02Command cmd) noexcept
{
	regWriteBuffer[0] = (uint32_t)cmd >> 24;
	regWriteBuffer[1] =  (uint32_t)cmd >> 16;
	regWriteBuffer[2] =  (uint32_t)cmd >> 8;
	memset(regWriteBuffer + 3, 0, sizeof(regWriteBuffer) - 3);
	return TransceivePacket(regWriteBuffer, regReadBuffer, sizeof(regReadBuffer));
}

// Send the command to read a 16-bit register, need to do another transfer to actually read it
bool ADS131M02::ReadRegister(Ads131M02Register regNum) noexcept
{
	const uint32_t cmd = (uint32_t)Ads131M02Command::rreg | ((uint32_t)regNum << 7);
	regWriteBuffer[0] = (uint32_t)cmd >> 24;
	regWriteBuffer[1] =  (uint32_t)cmd >> 16;
	regWriteBuffer[2] =  (uint32_t)cmd >> 8;
	memset(regWriteBuffer + 3, 0, sizeof(regWriteBuffer) - 3);
	return TransceivePacket(regWriteBuffer, regReadBuffer, sizeof(regReadBuffer));
}

// Write a 16-bit register
bool ADS131M02::WriteRegister(Ads131M02Register regNum, uint16_t val) noexcept
{
	const uint32_t cmd = (uint32_t)Ads131M02Command::wreg | ((uint32_t)regNum << 7);
	regWriteBuffer[0] = (uint32_t)cmd >> 24;
	regWriteBuffer[1] =  (uint32_t)cmd >> 16;
	regWriteBuffer[2] =  (uint32_t)cmd >> 8;
	regWriteBuffer[3] = val >> 8;
	regWriteBuffer[4] = val & 0xFF;
	memset(regWriteBuffer + 5, 0, sizeof(regWriteBuffer) - 5);
	return TransceivePacket(regWriteBuffer, regReadBuffer, sizeof(regReadBuffer));
}

#endif
