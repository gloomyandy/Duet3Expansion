/*
 * SharedSpiDevice.cpp
 *
 *  Created on: 28 Dec 2022
 *      Author: David
 *  Modified 23 Dec 2025 to support HW and PIO based SPI devices
 *      Author: Andy
 */

#include <Hardware/SharedSpiDevice.h>
#include <PioSpi.h>
#if RPXXXX
#if NUM_HW_SPI_CHANNELS > 0
# if NUM_HW_SPI_CHANNELS > 1
#  error "Only a single shared HW SPI channel is currently supported"
# endif

SharedHWSpiDevice::SharedHWSpiDevice(uint8_t spiInstanceNum) noexcept
	: hardware((spiInstanceNum == 0) ? spi0 : spi1)
{
	mutex.Create("HSPI");
}

void SharedHWSpiDevice::Disable() const noexcept
{
	spi_deinit(hardware);
}

void SharedHWSpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	spi_init(hardware, freq);
	spi_set_format(hardware, 8, ((uint8_t)mode & 2) ? SPI_CPOL_1 : SPI_CPOL_0, ((uint8_t)mode & 1) ? SPI_CPHA_1 : SPI_CPHA_0, SPI_MSB_FIRST);
}

bool SharedHWSpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const noexcept
{
	const int bytesTransferred = (rx_data == nullptr) ? spi_write_blocking(hardware, tx_data, len)
								: (tx_data == nullptr) ? spi_read_blocking(hardware, 0xFF, rx_data, len)
									: spi_write_read_blocking(hardware, tx_data, rx_data, len);
	return bytesTransferred == (int)len;
}
#endif

#if NUM_PIO_SPI_CHANNELS > 0
# if NUM_PIO_SPI_CHANNELS > 1
#  error "Only a single shared PIO SPI channel is currently supported"
# endif
SharedPIOSpiDevice::SharedPIOSpiDevice() noexcept
{
	mutex.Create("PSPI");
	pio_spi_init(&piospi0, SPIOSPI0SclkPin, SPIOSPI0MisoPin, SPIOSPI0MosiPin);
}

void SharedPIOSpiDevice::Disable() const noexcept
{
	pio_spi_disable(&piospi0);
}

void SharedPIOSpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	pio_spi_set_format(&piospi0, 8, ((uint8_t)mode & 2) ? SPI_CPOL_1 : SPI_CPOL_0, ((uint8_t)mode & 1) ? SPI_CPHA_1 : SPI_CPHA_0, SPI_MSB_FIRST, freq);
}

bool SharedPIOSpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const noexcept
{
	const int bytesTransferred = (rx_data == nullptr) ? pio_spi_write_blocking(&piospi0, (const uint8_t *)tx_data, len)
								: (tx_data == nullptr) ? pio_spi_read_blocking(&piospi0, 0xFF, rx_data, len)
									: pio_spi_write_read_blocking(&piospi0, (const uint8_t *)tx_data, rx_data, len);
	return bytesTransferred == (int)len;
}
#endif
#endif

// End
