/*
 * SpiDevice.cpp
 *
 *  Created on: 28 Dec 2022
 *      Author: David
 *  Modified 23 Dec 2025 to support HW and PIO based SPI devices
 *      Author: Andy
 */

#include <Hardware/SpiDevice.h>
#if RPXXXX
#if NUM_SPI_CHANNELS > 0

SpiDevice::SpiDevice(SPIChannel dev) noexcept
	: hardware(SPI::getSPIDevice(dev))
{
}

void SpiDevice::Disable() const noexcept
{
}

void SpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	hardware->configureDevice(8, (uint32_t)mode, freq);
}

bool SpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const noexcept
{
	return hardware->transceivePacket(tx_data, rx_data, len) == SPI_OK;
}


#endif
#endif

// End
