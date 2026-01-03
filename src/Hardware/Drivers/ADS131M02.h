/*
 * ADS131M02.h
 *
 *  Created on: 3 Jan 2026
 *      Author: David
 */

#ifndef SRC_HARDWARE_DRIVERS_ADS131M02_H_
#define SRC_HARDWARE_DRIVERS_ADS131M02_H_

#include <RepRapFirmware.h>

#if SUPPORT_ADS131M02

#include <Hardware/SpiDevice.h>

class ADS131M02 : public SpiDevice
{
public:
	ADS131M02() noexcept;
};

#endif

#endif /* SRC_HARDWARE_DRIVERS_ADS131M02_H_ */
