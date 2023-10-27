/*
 * LDC1612.h
 *
 *  Created on: 14 Jun 2023
 *      Author: David
 */

#ifndef SRC_HARDWARE_LDC1612_H_
#define SRC_HARDWARE_LDC1612_H_

#include <RepRapFirmware.h>

#if SUPPORT_LDC1612

#include "SharedI2CClient.h"

// LDC1612 class
class LDC1612 : public SharedI2CClient
{
public:
	LDC1612(SharedI2CMaster& dev, uint16_t i2cAddress = LDC1612_I2cAddress) noexcept;
	~LDC1612() noexcept {}

	// Do a quick test to check whether the accelerometer is present, returning true if it is
	bool CheckPresent() noexcept;
	bool Reset() noexcept;
	bool SetDefaultConfiguration(uint8_t channel, bool calibrationMode) noexcept;
	bool GetChannelResult(uint8_t channel, uint32_t& result) noexcept;
	bool CalibrateDriveCurrent(uint8_t channel) noexcept;
	uint8_t GetDriveCurrent(uint8_t channel) noexcept { return currentSetting[channel]; }
	bool SetDriveCurrent(uint8_t channel, uint16_t value) noexcept;
	bool IsChannelReady(uint8_t channel) noexcept;

	// Conversion result error bits (shifted left by 28 bits)
	static constexpr uint32_t ERR_UR0 = 1u << 3;
	static constexpr uint32_t ERR_OR0 = 1u << 2;
	static constexpr uint32_t ERR_WD0 = 1u << 1;
	static constexpr uint32_t ERR_AE0 = 1u << 0;

#if defined(SAMMYC21) || defined(TOOL1LC)
	static constexpr float ClockFrequency =	40.0;					// External or internal clock frequency. Assume we are using a LDC1612 breakout module with 40MHz crystal oscillator
	static constexpr uint16_t ClockDivisor = 2;						// The divisor we use for the clock. In continuous conversion mode the max divided clock speed allowed is 35MHz.
#else
	static constexpr float ClockFrequency =	25.0;					// External or internal clock frequency in MHz. Assume a tool board or SZP with 25MHz or 32MHz clock generated by the SAMC21
	static constexpr uint16_t ClockDivisor = 1;						// The divisor we use for the clock. Values greater than 1 are needed only for very low resonant frequencies.
#endif
	static constexpr float FRef = ClockFrequency/ClockDivisor;

private:
	static constexpr unsigned int NumChannels = 2;
	static constexpr float DefaultLCStabilizeTime = 100;			// in microseconds
	static constexpr float DefaultConversionTime = 2000.0;			// in microseconds. Scanning at 500mm/sec with point spacing 10mm gives 20ms interval between required readings.
	static constexpr uint16_t DefaultDriveCurrentVal = 13;			// current setting for Seeed LDC1612 board on toolchanger with flex bed plate at Z=1mm

	enum class LDCRegister : uint8_t
	{
		CONVERSION_RESULT_REG_START = 0x00,
		SET_CONVERSION_TIME_REG_START = 0x08,
		SET_CONVERSION_OFFSET_REG_START = 0x0C,
		SET_LC_STABILIZE_REG_START = 0x10,
		SET_FREQ_REG_START = 0x14,
		SENSOR_STATUS_REG = 0x18,
		ERROR_CONFIG_REG = 0x19,
		SENSOR_CONFIG_REG = 0x1A,
		MUX_CONFIG_REG = 0x1B,
		SENSOR_RESET_REG = 0x1C,
		SET_DRIVER_CURRENT_REG = 0x1E,
		READ_MANUFACTURER_ID = 0x7E,
		READ_DEVICE_ID = 0x7F
	};

	uint16_t GetStatus() noexcept;
	bool SetConversionTime(uint8_t channel, float microseconds) noexcept;
	bool SetLCStabilizeTime(uint8_t channel, float microseconds) noexcept;
	bool SetConversionOffset(uint8_t channel, uint16_t value) noexcept;
	bool SetErrorConfiguration(uint16_t value) noexcept;
	bool UpdateConfiguration(uint16_t value) noexcept;
	bool SetMuxConfiguration(uint16_t value) noexcept;
	bool ReadInitCurrent(uint8_t channel, uint16_t& value) noexcept;
	bool SetDivisors(uint8_t channel) noexcept;

	bool Read16bits(LDCRegister reg, uint16_t& val) noexcept;
	bool Write16bits(LDCRegister reg, uint16_t val) noexcept;

	uint16_t currentSetting[NumChannels];
};

#endif	// SUPPORT_LDC1612

#endif /* SRC_HARDWARE_LDC1612_H_ */
