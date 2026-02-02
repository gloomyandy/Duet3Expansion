/*
 * TPiS_1T_1086_L5_5.cpp
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#include "TPiS_1T_1086_L5_5.h"

#if SUPPORT_TPiS_1T_1086_L5_5

#include <CanMessageGenericParser.h>
#include <Platform/Platform.h>

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor TPiS_1T_1086_L5_5::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TPiS_1T_1086_L5_5(sensorNum); } );
TemperatureSensor::SensorTypeDescriptor TPiS_AmbientTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TPiS_AmbientTemperatureSensor(sensorNum); } );
TemperatureSensor::SensorTypeDescriptor TPiS_EnvironmentTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TPiS_EnvironmentTemperatureSensor(sensorNum); } );

TPiS_1T_1086_L5_5::TPiS_1T_1086_L5_5(unsigned int sensorNum) noexcept
	: I2CTemperatureSensor(sensorNum, TypeName, TPiS_I2CChannel, TPiS_I2CAddress),
	  r25(ThermistorR25[envThermistorAdcFilterChannel]),
	  beta(ThermistorBeta[envThermistorAdcFilterChannel]),
	  shC(ThermistorShC[envThermistorAdcFilterChannel]),
	  seriesR(ThermistorSeriesR[envThermistorAdcFilterChannel])
{
}

// Configure this temperature sensor
GCodeResult TPiS_1T_1086_L5_5::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool changed = false;
	if (parser.HasParameter('P'))
	{
		changed = true;

		// Configure the environment thermistor
		SetPinMode(TempSensePins[envThermistorAdcFilterChannel], PinMode::AIN);
		Platform::InitThermistorFilter(TempSensePins[envThermistorAdcFilterChannel], envThermistorAdcFilterChannel, false);

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
			if (epromBuffer[0] == 3 && epromBuffer[31] == (TPiS_I2CAddress | 0x80) && calculatedChecksum == storedChecksum)
			{
				// Get needed data from EPROM contents
				TempCalibPtat25 = ((uint16_t)epromBuffer[10] << 8) | epromBuffer[11];
				TempCalibM = ((uint16_t)epromBuffer[12] << 8) | epromBuffer[13];
				TempCalibUo = (((uint32_t)epromBuffer[14] << 8) | epromBuffer[15]) + 32768UL;
				TempCalibUout1 = (((uint32_t)epromBuffer[16] << 8) | epromBuffer[17]) * 2;
				TempCalibTobj1 = epromBuffer[18];

				// Pre-compute some derived calibration constants
				recipTempCalibM = 100.0/(float)TempCalibM;
				recipCorrectedK = (powf((float)TempCalibTobj1 - ABS_ZERO, 4.2) - powf(25.0 - ABS_ZERO, 4.2)) / ((float)(int32_t)(TempCalibUout1 - TempCalibUo) * FovAndEmissivityCorrection);

				ambientTemperatureDegK = objectTemperatureDegK = 0.0;
			}
			else
			{
				reply.printf("Error in thermopile sensor EPROM: %02x %02x %04x %04x", epromBuffer[0], epromBuffer[31], calculatedChecksum, storedChecksum);
				return GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Failed to initialise thermopile sensor");
			return GCodeResult::error;
		}
	}

	// Get the environment thermistor parameters. We don't currently allow the series resistance to be configured because we don't expect it to change.
	if (parser.GetFloatParam('B', beta))
	{
		shC = 0.0;						// if user changes B and doesn't define C, assume C=0
		changed = true;
	}
	changed = parser.GetFloatParam('C', shC) || changed;
	changed = parser.GetFloatParam('T', r25) || changed;

	if (changed)
	{
		shB = 1.0/beta;
		const float lnR25 = logf(r25);
		shA = 1.0/(25.0 - ABS_ZERO) - shB * lnR25 - shC * lnR25 * lnR25 * lnR25;
	}
	else
	{
		CopyBasicDetails(reply);
		reply.catf(", environment thermistor: T:%.1f B:%.1f C:%.2e R:%.1f", (double)r25, (double)beta, (double)shC, (double)seriesR);
	}
	return GCodeResult::ok;
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
		ambientTemperatureDegK = (25.0 - ABS_ZERO) + (float)((int32_t)(rawTAmb - TempCalibPtat25)) * recipTempCalibM;

		// Calculate the object temperature. See datasheet section 8.5. We assume that LOOKUP# = 2.
		objectTemperatureDegK = powf((float)(int32_t)(rawTpObject - TempCalibUo) * recipCorrectedK + powf(ambientTemperatureDegK, 4.2), 1.0/4.2);

#if 0	//DEBUG
		{
			static uint8_t count=0;
			++count;
			if (count == 0)
			{
				debugPrintf("PTAT25=%u M=%u U0=%lu Uout1=%lu Tobj1=%u 1/M=%.4f 1/K=%.3f", TempCalibPtat25, TempCalibM, TempCalibUo, TempCalibUout1, TempCalibTobj1, (double)recipTempCalibM, (double)recipCorrectedK);
				debugPrintf(" %lu %lu %.1f %.1f\n", rawTAmb, rawTpObject, (double)ambientTemperatureDegK, (double)objectTemperatureDegK);
			}
		}
#endif
		SetResult(objectTemperatureDegK + ABS_ZERO, TemperatureError::ok);
	}
	else
	{
		SetResult(TemperatureError::ioError);
	}

	// Read the environment thermistor
	const volatile ThermistorAveragingFilter * const tempFilter = Platform::GetAdcFilter(envThermistorAdcFilterChannel);
	if (tempFilter->IsValid())
	{
		const int32_t averagedTempReading = tempFilter->GetSum()/(tempFilter->NumAveraged() >> AdcOversampleBits);
		if (OversampledAdcRange <= averagedTempReading)
		{
			SetResult(ABS_ZERO, TemperatureError::openCircuit);
		}
		else
		{
			const float denom = (float)(OversampledAdcRange - averagedTempReading) - 0.5;
			float resistance = seriesR * ((float)averagedTempReading + 0.5)/denom;
			const float logResistance = logf(resistance);
			const float recipT = shA + shB * logResistance + shC * logResistance * logResistance * logResistance;
			const float temp = (recipT > 0.0) ? (1.0/recipT) + ABS_ZERO : BadErrorTemperature;

			// It's hard to distinguish between an open circuit and a cold high-resistance thermistor.
			// So we treat a temperature below -5C as an open circuit, unless we are using a low-resistance thermistor.
			if (temp < MinimumConnectedTemperature && resistance > seriesR * 100)
			{
				// Assume thermistor is disconnected
				environmentTemperature = ABS_ZERO;
				thermistorResult = TemperatureError::openCircuit;
			}
			else
			{
				environmentTemperature = temp;
				thermistorResult = TemperatureError::ok;
			}
		}
	}
	else
	{
		environmentTemperature = BadErrorTemperature;
		thermistorResult = TemperatureError::notReady;
	}
}

TemperatureError TPiS_1T_1086_L5_5::GetAdditionalOutput(float &t, uint8_t outputNumber) noexcept
{
	const auto result = TemperatureSensor::GetLatestTemperature(t);
	switch (outputNumber)
	{
	case 1:
		t = ambientTemperatureDegK + ABS_ZERO;
		break;

	case 2:
		t = environmentTemperature;
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

// TPiS_EnvironmentTemperatureSensor members
TPiS_EnvironmentTemperatureSensor::TPiS_EnvironmentTemperatureSensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, TypeName, false)
{
}

TPiS_EnvironmentTemperatureSensor::~TPiS_EnvironmentTemperatureSensor() noexcept
{
}

#endif

// End

