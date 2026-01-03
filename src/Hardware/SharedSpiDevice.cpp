/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 3 Jan 2026
 *      Author: David
 */

#include "SharedSpiDevice.h"

#if SUPPORT_SPI_SENSORS || SUPPORT_CLOSED_LOOP || defined(ATEIO)

#if SAME5x || SAMC21
SharedSpiDevice::SharedSpiDevice(uint8_t sercomNum, uint32_t dataInPad, uint32_t dataOutPad) noexcept : SpiDevice(sercomNum, dataInPad, dataOutPad)
#elif RP2040
SharedSpiDevice::SharedSpiDevice(uint8_t spiInstanceNum) noexcept : SpiDevice(spiInstanceNum)
#endif
{
	mutex.Create("SPI");
}

#endif
