/*
 * SpiDevice.h
 *
 *  Created on: 3 Jan 2026
 *      Author: David
 */

#ifndef SRC_HARDWARE_SPIDEVICE_H_
#define SRC_HARDWARE_SPIDEVICE_H_

#include "RepRapFirmware.h"

#if NUM_SHARED_SPI != 0

#include <RTOSIface/RTOSIface.h>

#if RP2040
# include "hardware/spi.h"
#endif

enum class SpiMode : uint8_t
{
	mode0 = 0, mode1, mode2, mode3
};


// This class represents a master SPI interface, but not the associated CS pin(s).
// It is used as the base class for SharedSpiDevice. It can also be used by itself to control a non-shared SPI master.
class SpiDevice
{
public:
#if SAME5x || SAMC21
	SpiDevice(uint8_t sercomNum, uint32_t dataInPad, uint32_t dataOutPad) noexcept;
#else
	explicit SpiDevice(uint8_t spiInstanceNum) noexcept;
#endif

	void Disable() const noexcept;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept;
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept;

protected:
	static constexpr uint32_t DefaultSpiClockFrequency = 2000000;

#if SAME5x || SAMC21
	bool waitForTxReady() const noexcept;
	bool waitForTxEmpty() const noexcept;
	bool waitForRxReady() const noexcept;

	Sercom * const hardware;
#elif RP2040
	spi_inst_t *hardware;
#endif
};

#endif

#endif /* SRC_HARDWARE_SPIDEVICE_H_ */
