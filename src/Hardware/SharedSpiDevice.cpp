/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 3 Jan 2026
 *      Author: David
 */

#include "SharedSpiDevice.h"

#if NUM_SPI_CHANNELS > 0

#if SAME5x || SAMC21
SharedSpiDevice::SharedSpiDevice(uint8_t sercomNum, uint32_t dataInPad, uint32_t dataOutPad) noexcept : SpiDevice(sercomNum, dataInPad, dataOutPad)
{
	mutex.Create("SPI");
}
#elif RPXXXX
static const char *names[] = { "SPI0", "SPI1", "SPI2"};
SharedSpiDevice::SharedSpiDevice(SPIChannel dev) noexcept : SpiDevice(dev)
{
	mutex.Create(names[dev]);
}
#endif

#endif
