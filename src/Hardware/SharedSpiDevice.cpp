/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 3 Jan 2026
 *      Author: David
 */

#include "SharedSpiDevice.h"

#if NUM_SHARED_SPI != 0

#if SAME5x || SAMC21
SharedSpiDevice::SharedSpiDevice(uint8_t sercomNum, uint32_t dataInPad, uint32_t dataOutPad) noexcept : SpiDevice(sercomNum, dataInPad, dataOutPad)
#else
SharedSpiDevice::SharedSpiDevice(uint8_t spiInstanceNum) noexcept : SpiDevice(spiInstanceNum)
#endif
{
	mutex.Create("SPI");
}

#endif
