/*
 * LP5817.h
 *
 *  Created on: 26 Dec 2025
 *      Author: David
 */

#ifndef SRC_HARDWARE_LP5817_H_
#define SRC_HARDWARE_LP5817_H_

#include <RepRapFirmware.h>

#if SUPPORT_LP5817

#include <Hardware/SharedI2CClient.h>

// Temperature derating curve for LTST-E143EGBW LEDs
// Due to C++ rules on calling constexpr functions these have to be declared before the class that calls them
static inline constexpr float MaxCurrentRed(float temperature) noexcept		{ return (temperature <= 25) ? 30 : (temperature > 100) ? 0 : 5 + (25/75) * (100 - temperature); }
static inline constexpr float MaxCurrentGreenBlue(float temperature) noexcept	{ return (temperature <= 25) ? 20 : (temperature > 100) ? 0 : 5 + (15/75) * (100 - temperature); }

// LP5817 LED driver class
class LP5817 : public SharedI2CClient
{
public:
	LP5817(SharedI2CMaster& dev) noexcept;

	// Currently we only support colours for which each LED is either on or off
	enum LedColour : uint8_t
	{
		black = 0,
		red,
		green,
		yellow,
		blue,
		magenta,
		cyan,
		white
	};

	bool Init(uint8_t inputPins, uint8_t initialOutputs) noexcept;		// initialise the device returning true if it was found
	void SetColour(LedColour colour) noexcept;							// set new LED colours

private:
	enum class LP5817_Register : uint8_t								// these are all 8-bit registers
	{
		chip_en = 0, dev_config0, dev_config1, dev_config2, dev_config3,
		shutdown_cmd = 0x0D, reset_cmd, update_cmd,
		flag_clr = 0x13, out0_dc, out1_dc, out2_dc,
		out0_manual_pwm = 0x18, out1_manual_pwm, out2_manual_pwm,
		flag = 0x40
	};

	static constexpr uint32_t LP5817_I2CTimeout = 25;					// timeout in milliseconds when waiting to acquire the I2C bus

	// Define the analog current setting to use on each LED
	static constexpr float FullScaleCurrent = 25.5;						// we use the 25.5mA max current setting on the LP5817 (the alternative is 51mA)
	static constexpr unsigned int NumParalelLEDs = 2;
	static constexpr float PlannedMaxTemperature = 85;

	static constexpr float MaxCurrentSettingRed = MaxCurrentRed(PlannedMaxTemperature)/FullScaleCurrent * 255 * NumParalelLEDs;
	static constexpr float MaxCurrentSettingGreenBlue = MaxCurrentGreenBlue(PlannedMaxTemperature)/FullScaleCurrent * 255 * NumParalelLEDs;
	static_assert(MaxCurrentSettingRed >= 1.0 && MaxCurrentSettingRed <= 255.0);
	static_assert(MaxCurrentSettingGreenBlue >= 1.0 && MaxCurrentSettingRed <= 255.0);

	static constexpr uint8_t CurrentSetting[3] = { (uint8_t)MaxCurrentSettingRed, (uint8_t)MaxCurrentSettingGreenBlue, (uint8_t)MaxCurrentSettingGreenBlue };

	bool Read8(LP5817_Register reg, uint8_t& val) noexcept;
	bool Write8(LP5817_Register reg, uint8_t val) noexcept;

	LedColour currentColour;
};

#endif

#endif /* SRC_HARDWARE_LP5817_H_ */
