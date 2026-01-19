/*
 * TPiS_1T_1086_L5_5.cpp
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#include "TPiS_1T_1086_L5_5.h"

#if SUPPORT_TPiS_1T_1086_L5_5

#include <CanMessageGenericParser.h>

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor TPiS_1T_1086_L5_5::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TPiS_1T_1086_L5_5(sensorNum); } );
TemperatureSensor::SensorTypeDescriptor TPiS_AmbientTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TPiS_AmbientTemperatureSensor(sensorNum); } );

TPiS_1T_1086_L5_5::TPiS_1T_1086_L5_5(unsigned int sensorNum) noexcept
	: I2CTemperatureSensor(sensorNum, TypeName, TPiS_I2CChannel, TPiS_I2CAddress)
{
}

// Configure this temperature sensor
GCodeResult TPiS_1T_1086_L5_5::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	if (parser.HasParameter('P'))
	{
		// Initialise the sensor
		bool ok;
		// Set the slave address of the sensor. After power up it only responds to the General Call address.
		{
			device.SetAddress(0);									// general call address
			const uint8_t txBuffer[] = { 0x04 };					// command to load address from EPROM
			ok = device.Transfer(txBuffer, nullptr, sizeof(txBuffer), 0, TPis_I2C_timeout, true);
		}

		// Set up to read the EPROM
		if (ok)
		{
			delay(10);
			device.SetAddress(TPiS_I2CAddress);						// default address read from EPROM
			const uint8_t txBuffer[] = { 0x1F, 0x80 };				// command to enable read from EPROM
			ok = device.Transfer(txBuffer, nullptr, sizeof(txBuffer), 0, TPis_I2C_timeout, false);
		}

		// Read the whole EPROM
		uint8_t epromBuffer[32];
		if (ok)
		{
			const uint8_t txBuffer[] = { 0x20 };
			ok = device.Transfer(txBuffer, epromBuffer, sizeof(txBuffer), sizeof(epromBuffer), TPis_I2C_timeout, false);
			const uint8_t txBuffer2[] = { 0x1F, 0x00 };				// command to disable read from EPROM
			(void)device.Transfer(txBuffer2, nullptr, sizeof(txBuffer2), 0, TPis_I2C_timeout, true);
		}

		if (ok)
		{
			// Check the EPROM
			uint16_t calculatedChecksum = epromBuffer[0];
			for (size_t i = 3; i < 32; ++i)
			{
				calculatedChecksum += epromBuffer[i];
			}
			const uint16_t storedChecksum = ((uint16_t)epromBuffer[1] << 8) | (uint16_t)epromBuffer[2];
			if (epromBuffer[0] == 3 && epromBuffer[31] == TPiS_I2CAddress && calculatedChecksum == storedChecksum)
			{
				// Get needed data from EPROM contents
				TempCalibPtat25 = ((uint16_t)epromBuffer[10] << 8) | epromBuffer[11];
				TempCalibM = ((uint16_t)epromBuffer[12] << 8) | epromBuffer[13];
				TempCalibUo = (((uint32_t)epromBuffer[14] << 8) | epromBuffer[15]) + 32768L;
				TempCalibUout1 = (((uint32_t)epromBuffer[16] << 8) | epromBuffer[17]) * 2;
				TempCalibTobj1 = epromBuffer[18];

				// Pre-compute some derived calibration constants
				recipTempCalibM = 1.0/(float)TempCalibM;
				recipCorrectedK = ((powf((float)TempCalibTobj1, 4.2) - powf(25.0 - ABS_ZERO, 4.2)) * FovAndEmissivityCorrection) / (float)(TempCalibUout1 - TempCalibUo);

				ambientTemperatureDegK = objectTemperatureDegK = 0.0;
				return GCodeResult::ok;
			}
			reply.copy("Error in thermopile sensor EPROM");
			return GCodeResult::error;
		}
		reply.copy("Failed to initialise thermopile sensor");
		return GCodeResult::error;
	}
	else
	{
		CopyBasicDetails(reply);
		return GCodeResult::ok;
	}
}

void TPiS_1T_1086_L5_5::Poll() noexcept
{
	// Read the object and ambient temperatures from the sensor
	const uint8_t txBuffer[] = { 0x01 };
	uint8_t rxBuffer[4];
	if (device.Transfer(txBuffer, rxBuffer, sizeof(txBuffer), sizeof(rxBuffer), true))
	{
		const uint32_t rawTpObject = ((uint32_t)rxBuffer[0] << 9) | ((uint32_t)rxBuffer[1] << 1) | ((uint32_t)rxBuffer[2] >> 7);
		const uint32_t rawTAmb = ((uint32_t)(rxBuffer[2] & 0x7F)) << 8 | (uint32_t)rxBuffer[3];

		// Calculate the ambient temperature reported by the sensor in degK. See datasheet section 8.4.
		ambientTemperatureDegK = (25.0 - ABS_ZERO) + (float)(rawTAmb - TempCalibPtat25) * recipTempCalibM;

		// Calculate the object temperature. See datasheet section 8.5. We assume that LOOKUP# = 2.
		objectTemperatureDegK = powf((rawTpObject - TempCalibUo) * recipCorrectedK + powf(ambientTemperatureDegK, 4.2), 1.0/4.2);
		SetResult(objectTemperatureDegK - ABS_ZERO, TemperatureError::ok);
	}
	else
	{
		SetResult(TemperatureError::ioError);
	}
}

TemperatureError TPiS_1T_1086_L5_5::GetAdditionalOutput(float &t, uint8_t outputNumber) noexcept
{
	const auto result = TemperatureSensor::GetLatestTemperature(t);
	switch (outputNumber)
	{
	case 1:
		t = ambientTemperatureDegK - ABS_ZERO;
		break;

	default:
		t = BadErrorTemperature;
		return TemperatureError::invalidOutputNumber;
	}
	return result;
}

// TPiS_AmbientTemperatureSensor members

TPiS_AmbientTemperatureSensor::TPiS_AmbientTemperatureSensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, TypeName, false)
{
}

TPiS_AmbientTemperatureSensor::~TPiS_AmbientTemperatureSensor() noexcept
{
}

#endif

// End

