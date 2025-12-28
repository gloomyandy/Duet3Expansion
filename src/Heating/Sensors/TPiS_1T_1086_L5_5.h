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

#include "I2CTemperatureSensor.h"
#include "AdditionalOutputSensor.h"

class CanMessageGenericParser;

class TPiS_1T_1086_L5_5 : public I2CTemperatureSensor
{
public:
	TPiS_1T_1086_L5_5(unsigned int sensorNum) noexcept;

	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;

	static constexpr const char *TypeName = "thermopile_tpis.object";

	const uint8_t GetNumAdditionalOutputs() const noexcept override { return 1; }
	TemperatureError GetAdditionalOutput(float& t, uint8_t outputNumber) noexcept override;

	void Poll() override;

private:
	static SensorTypeDescriptor typeDescriptor;

	static constexpr uint32_t TPis_I2C_timeout = 25;					// timeout in milliseconds when waiting to acquire the I2C bus

	static constexpr float FovAndEmissivityCorrection = 0.45;			// Calibration factor for reduced emissivity and/or reduced FOV coverage

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
};

// This class represents the TPiS_1T_1086_L5_5 ambient temperature sensor
class TPiS_AmbientTemperatureSensor : public AdditionalOutputSensor
{
public:
	TPiS_AmbientTemperatureSensor(unsigned int sensorNum) noexcept;
	~TPiS_AmbientTemperatureSensor() noexcept;

	static constexpr const char *TypeName = "thermopile_tpis.ambient";

private:
	static SensorTypeDescriptor typeDescriptor;
};

#endif

#endif /* SRC_HEATING_SENSORS_TPIS_1T_1086_L5_5_H_ */
