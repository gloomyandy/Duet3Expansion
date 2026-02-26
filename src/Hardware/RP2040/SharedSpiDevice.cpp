/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 28 Dec 2022
 *      Author: David
 *  Modified 23 Dec 2025 to support HW and PIO based SPI devices
 *      Author: Andy
 */

#include <Hardware/SharedSpiDevice.h>
#if RPXXXX
#if NUM_SPI_CHANNELS > 0
static const char *names[] = { "SPI0", "SPI1", "SPI2"};

SharedSpiDevice::SharedSpiDevice(SPIChannel dev) noexcept
	: hardware(SPI::getSPIDevice(dev))
{
	mutex.Create(names[dev]);
}

void SharedSpiDevice::Disable() const noexcept
{
}

void SharedSpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	hardware->configureDevice(8, (uint32_t)mode, freq);
}

bool SharedSpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const noexcept
{
	return hardware->transceivePacket(tx_data, rx_data, len) == SPI_OK;
}


#endif
#endif

// End
