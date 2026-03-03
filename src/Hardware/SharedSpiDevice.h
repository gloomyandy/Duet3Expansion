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

#include "SpiDevice.h"
#include <RTOSIface/RTOSIface.h>

class SharedSpiDevice : public SpiDevice
{
public:
#if SAME5x || SAMC21
	SharedSpiDevice(uint8_t sercomNum, uint32_t dataInPad, uint32_t dataOutPad) noexcept;
#elif RPXXXX
	SharedSpiDevice(SPIChannel dev) noexcept;
#endif

	bool Take(uint32_t timeout) noexcept { return mutex.Take(timeout); }					// get ownership of this SPI, return true if successful
	void Release() noexcept { mutex.Release(); }

private:
	Mutex mutex;
};



#endif

#endif /* SRC_HARDWARE_SHAREDSPIDEVICE_H_ */
