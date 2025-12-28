/*
 * TPiS_1T_1086_L5_5.h
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_TPIS_1T_1086_L5_5_H_
#define SRC_HEATING_SENSORS_TPIS_1T_1086_L5_5_H_

#include "RepRapFirmware.h"

#if SUPPORT_TPiS_1T_1086_L5_5

#include <Hardware/SharedI2CClient.h>

class CanMessageGenericParser;

class TPiS_1T_1086_L5_5
{
public:
	TPiS_1T_1086_L5_5(SharedI2CMaster& master) noexcept;
	void Init() noexcept;
	bool TakeReading() noexcept;
	float GetAmbientTemperature() const noexcept { return ambientTemperatureDegK - 273.15; }
	float GetObjectTemperature() const noexcept { return objectTemperatureDegK - 273.15; }

private:
	static constexpr uint32_t TPis_I2C_timeout = 25;					// timeout in milliseconds when waiting to acquire the I2C bus

	static constexpr float FovAndEmissivityCorrection = 0.45;			// Calibration factor for reduced emissivity and/or reduced FOV coverage

	SharedI2CClient device;

	// Calibration constants read from EPROM
	uint16_t TempCalibPtat25;
	uint16_t TempCalibM;
	uint32_t TempCalibUo;
	uint32_t TempCalibUout1;
	uint8_t TempCalibTobj1;

	// Calibration constants calculated from the above
	float recipTempCalibM;
	float recipCorrectedK;

	// Latest readings
	float ambientTemperatureDegK;
	float objectTemperatureDegK;

	bool initialised = false;
};

#endif

#endif /* SRC_HEATING_SENSORS_TPIS_1T_1086_L5_5_H_ */
