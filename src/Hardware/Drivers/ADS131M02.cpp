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
	SetPinMode(ADS131M02_CsPin, OUTPUT_HIGH);
	SetPinFunction(ADS131M02_MosiPin, ADS131M02_SpiPinFunction);
	SetPinFunction(ADS131M02_MisoPin, ADS131M02_SpiPinFunction);
	SetPinFunction(ADS131M02_SclkPin, ADS131M02_SpiPinFunction);
}

#endif
