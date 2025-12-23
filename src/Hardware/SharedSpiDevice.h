/*
 * SharedSpiDevice.h
 *
 *  Created on: 1 Jul 2019
 *      Author: David
 *
 *  This currently supports only a single SPI channel. To support multiple SPI channels we would need to make the underlying SERCOM device
 *  configured in SPI mode a separate object, and have a pointer or reference to it in SharedSpiDevice.
 */

#ifndef SRC_HARDWARE_SHAREDSPIDEVICE_H_
#define SRC_HARDWARE_SHAREDSPIDEVICE_H_

#include "RepRapFirmware.h"

#if NUM_SPI_CHANNELS > 0

#include <RTOSIface/RTOSIface.h>

#if RPXXXX
# include "hardware/spi.h"
#endif

enum class SpiMode : uint8_t
{
	mode0 = 0, mode1, mode2, mode3
};


class SharedSpiDevice
{
public:
#if SAME5x || SAMC21
	SharedSpiDevice(uint8_t sercomNum, uint32_t dataInPad) noexcept;
#elif RPXXXX
	SharedSpiDevice() noexcept {}
#endif

#if RPXXXX
	virtual void Disable() const noexcept = 0;
	virtual void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept = 0;
	virtual bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept = 0;
#else
	void Disable() const noexcept;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept;
#endif
	bool Take(uint32_t timeout) noexcept { return mutex.Take(timeout); }					// get ownership of this SPI, return true if successful
	void Release() noexcept { mutex.Release(); }

	static constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;

private:

#if SAME5x || SAMC21
	bool waitForTxReady() const noexcept;
	bool waitForTxEmpty() const noexcept;
	bool waitForRxReady() const noexcept;

	Sercom * const hardware;
#endif
#if RPXXXX
protected:
#endif
	Mutex mutex;
};

#if RPXXXX
class SharedHWSpiDevice : public SharedSpiDevice
{
public:
	SharedHWSpiDevice(uint8_t spiInstanceNum) noexcept;

	void Disable() const noexcept override;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept override;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept override;
private:
	spi_inst_t *hardware;
};

class SharedPIOSpiDevice : public SharedSpiDevice
{
public:
	SharedPIOSpiDevice() noexcept;

	void Disable() const noexcept override;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept override;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept override;
};
#endif

#endif

#endif /* SRC_HARDWARE_SHAREDSPIDEVICE_H_ */
