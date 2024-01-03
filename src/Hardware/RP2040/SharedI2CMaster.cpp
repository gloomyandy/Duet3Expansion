/*
 * SharedI2CMaster.cpp
 *
 *  Created on: 13 Mar 2021
 *      Author: Andy
 */

#include <Hardware/SharedI2CMaster.h>

#if SUPPORT_I2C_SENSORS
#define USE_I2C_DMA		(0)

constexpr uint32_t DefaultSharedI2CClockFrequency = 400000;
constexpr uint32_t I2CTimeoutTicks = 100;

SharedI2CMaster::SharedI2CMaster(uint8_t spiInstanceNum) noexcept
	: hardware((spiInstanceNum == 0) ? i2c0 : i2c1), taskWaiting(nullptr), busErrors(0), naks(0), contentions(0), otherErrors(0), state(I2cState::idle)
{
	mutex.Create("I2C");
	currentClockRate = DefaultSharedI2CClockFrequency;
	Enable();
}

// Set the I2C clock frequency. Caller must own the mutex first.
void SharedI2CMaster::SetClockFrequency(uint32_t freq) noexcept
{
	if (freq != currentClockRate)
	{
		// We have to disable I2C device in order to change the baud rate
		Disable();
		currentClockRate = freq;
		Enable();
	}
}

void SharedI2CMaster::Enable() const noexcept
{
	int ret = i2c_init(hardware, currentClockRate);
	debugPrintf("Enable i3c freq %d ret %d\n", currentClockRate, ret);

}

void SharedI2CMaster::Disable() const noexcept
{
	i2c_deinit(hardware);
}

// Write then read data. Caller must own the mutex first.
bool SharedI2CMaster::Transfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept
{
	// If an empty transfer, nothing to do
	if (numToRead + numToWrite == 0)
	{
		return true;
	}

	for (unsigned int triesDone = 0; triesDone < 3; ++triesDone)
	{
		if (InternalTransfer(address, txBuffer, rxBuffer, numToWrite, numToRead))
		{
			return true;
		}

		// Had an I2C error, so re-initialise
		Disable();
		Enable();
	}
	return false;
}

// Get ownership of this I2C interface, return true if successful
bool SharedI2CMaster::Take(uint32_t timeout) noexcept
{
	const bool success = mutex.Take(timeout);
	if (!success)
	{
		++contentions;
	}
	return success;
}

void SharedI2CMaster::Release() noexcept
{
	mutex.Release();
}

void SharedI2CMaster::Diagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("I2C bus errors %u, naks %u, contentions %u, other errors %u", busErrors, naks, contentions, otherErrors);
	busErrors = naks = contentions = otherErrors = 0;
}

bool SharedI2CMaster::InternalTransfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept
{
	if (numToWrite > 0)
	{
		//debugPrintf("i2c write len %d addr %x\n", numToWrite, address);
		int ret = i2c_write_timeout_us(hardware, address, txBuffer, numToWrite, numToRead > 0, I2CTimeoutTicks*1000);
		if (ret <= 0)
		{
			debugPrintf("i2c write error %d\n", ret);
			if (ret == PICO_ERROR_TIMEOUT) 
				busErrors++;
			else if (ret == PICO_ERROR_GENERIC)
				naks++;
			else
				otherErrors++;
			
			return false;
		}
	}
	if (numToRead > 0)
	{
		//debugPrintf("i2c read len %d addr %x\n", numToWrite, address);
		int ret = i2c_read_timeout_us(hardware, address, rxBuffer, numToRead, false, I2CTimeoutTicks*1000);
		if (ret <= 0)
		{
			debugPrintf("i2c read error %d\n", ret);
			if (ret == PICO_ERROR_TIMEOUT) 
				busErrors++;
			else if (ret == PICO_ERROR_GENERIC)
				naks++;
			else
				otherErrors++;
			return false;
		}
	}
	return true;
}

void SharedI2CMaster::ProtocolError() noexcept
{

}

void SharedI2CMaster::Interrupt() noexcept
{

}

#endif

// End
