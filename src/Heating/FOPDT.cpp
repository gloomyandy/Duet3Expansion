/*
 * FOPDT.cpp
 *
 *  Created on: 16 Aug 2016
 *      Author: David
 */

#include "FOPDT.h"
#include "CanMessageFormats.h"

#define SQRT_FAN_SCALING		1		// 1 = fan cooling rate assumed to scale with square root of fan PWM, 0 = assumed to scale linearly

// Set up sensible defaults here in case the user enables the heater without specifying values for all the parameters.
FopDt::FopDt() noexcept
{
	Reset();
}

// Check the model parameters are sensible, if they are then save them and return true. If not then write an error message to reply and return false.
bool FopDt::SetParameters(const CanMessageHeaterModelV3& msg, const StringRef& reply) noexcept
{
	// DC 2017-06-20: allow S down to 0.01 for one of our OEMs (use > 0.0099 because >= 0.01 doesn't work due to rounding error)
	const char *const err =
		  (msg.basicModel.heatingRate/msg.basicModel.basicCoolingRate < 0.1) ? "estimated temperature rise too small"					// minimum 10C temperature rise (same as with earlier heater model)
		: (msg.basicModel.fanCoolingRate < 0.0) ? "fan reduces cooling rate"
		: (msg.basicModel.coolingRateExponent < 1.0 || msg.basicModel.coolingRateExponent > 1.6) ? "cooling rate exponent out of range"
		: (msg.basicModel.deadTime <= 0.0099) ? "dead time too small"
		: (msg.basicModel.deadTime * msg.basicModel.basicCoolingRate > 50.0) ? "dead time too close to cooling time constant"			// dead time less than half the cooling time constant
		: (msg.maxPwm <= 0.0099 || msg.maxPwm > 1.0) ? "PWM out of range"
		: nullptr;
	if (err == nullptr)
	{
		basicModel = msg.basicModel;
		maxPwm = msg.maxPwm;
		inverted = msg.inverted;
		pidParametersOverridden = msg.pidParametersOverridden;

		if (msg.pidParametersOverridden)
		{
			SetRawPidParameters(msg.kP, msg.recipTi, msg.tD);
		}
		else
		{
			CalcPidConstants(100.0);
		}
		enabled = true;
		return true;
	}
	reply.printf("bad model parameters: %s", err);
	return false;
}

void FopDt::Reset() noexcept
{
	SetDefaultModel(DefaultToolHeaterModel);	// set some values so that we don't report rubbish in the OM
	enabled = false;																// heater is disabled until the parameters are set
}

void FopDt::SetDefaultModel(const HeaterModel& model) noexcept
{
	basicModel = model;
	basicModel.zero = 0;

	maxPwm = 1.0;
	inverted = pidParametersOverridden = false;
	CalcPidConstants(basicModel.typicalTemperature);
	enabled = true;
}

// Get the PID parameters as reported by M301
M301PidParameters FopDt::GetM301PidParameters(bool forLoadChange) const
{
	M301PidParameters rslt;
	const PidParameters& pp = GetPidParameters(forLoadChange);
	const float reportedKp = pp.kP * 255.0;
	rslt.kP = reportedKp;
	rslt.kI = pp.recipTi * reportedKp;
	rslt.kD = pp.tD * reportedKp;
	return rslt;
}

// Override the PID parameters. We set both sets to the same parameters.
void FopDt::SetM301PidParameters(const M301PidParameters& pp) noexcept
{
	SetRawPidParameters(pp.kP * (1.0/255.0), pp.kI/pp.kP, pp.kD/pp.kP);
}

void FopDt::SetRawPidParameters(float p_kP, float p_recipTi, float p_tD) noexcept
{
	loadChangeParams.kP = setpointChangeParams.kP = p_kP;
	loadChangeParams.recipTi = setpointChangeParams.recipTi = p_recipTi;
	loadChangeParams.tD = setpointChangeParams.tD = p_tD;
	pidParametersOverridden = true;
}

/* Re-calculate the PID parameters.
 * For some possible formulas, see "Comparison of some well-known PID tuning formulas", Computers and Chemical Engineering 30 (2006) 1416�1423,
 * available at http://www.ece.ualberta.ca/~marquez/journal_publications_files/papers/tan_cce_06.pdf
 * Here are some examples, where r = td/tc:
 *    Cohen-Coon (modified to use half the original Kc value):
 *     Kc = (0.67/G) * (r + 0.185)
 *     Ti = 2.5 * td * (tc + 0.185 * td)/(tc + 0.611 * td)
 *     Td = 0.37 * td * tc/(tc + 0.185 * td)
 *    Ho et al, best response to setpoint changes:
 *     Kc = (1.086/G) * (r^-0.869
 *     Ti = tc/(0.74 - 0.13 * r)
 *     Td = 0.348 * tc * r^0.914
 *    IAE-setpoint:
 *     Kc = (0.65/G) * r^-1.04432
 *     Ti = tc/(0.9895 + 0.09539 * r)
 *     Td = 0.50814 * tc * r^1.08433
 *    Ho et al, best response to load changes:
 *     Kc = (1.435/G) * r^-0.921
 *     Ti = 1.14 * tc * r^0.749
 *     Td = 0.482 * tc * r^1.137
 *    ITAE-load:
 *     Kc = (0.77902/G) * r^-1.06401
 *     Ti = (tc/1.14311) * r^0.70949
 *     Td = 0.57137 * tc * r^1.03826
 * However, none of these works well in this application. The setpoint-based methods have integral times comparable to the process time
 * constant. This makes them very slow to reach that target temperature. Typically, the power is reduced too soon, so the temperature
 * flattens out too soon it and then it takes a very long time for the integral term to accumulate to the required value. The load-based
 * ones tend to have massive overshoot when the setpoint is changed, and even in the steady state some of them have marginal stability.
 */

void FopDt::CalcPidConstants(float targetTemperature) noexcept
{
	if (!pidParametersOverridden)
	{
		// Calculate the cooling rate per degC at this temperature. We assume the fan is at 20% speed.
		const float temperatureRise = max<float>(targetTemperature - NormalAmbientTemperature, 1.0);		// avoid division by zero!
		const float averageCoolingRatePerDegC = GetCoolingRate(temperatureRise, 0.2)/temperatureRise;
		loadChangeParams.kP = 0.7/(basicModel.heatingRate * basicModel.deadTime);
		loadChangeParams.recipTi = powf(averageCoolingRatePerDegC, 0.25)/(1.14 * powf(basicModel.deadTime, 0.75));	// Ti = 1.14 * timeConstant^0.25 * deadTime^0.75 (Ho et al)
		loadChangeParams.tD = basicModel.deadTime * 0.7;

		setpointChangeParams.kP = 0.7/(basicModel.heatingRate * basicModel.deadTime);
		setpointChangeParams.recipTi = powf(averageCoolingRatePerDegC, 0.5)/powf(basicModel.deadTime, 0.5);			// Ti = timeConstant^0.5 * deadTime^0.5
		setpointChangeParams.tD = basicModel.deadTime * 0.7;
	}
}

// Adjust the actual heater PWM for supply voltage
float FopDt::CorrectPwmForVoltage(float requiredPwm, float actualVoltage) const noexcept
{
	if (requiredPwm < maxPwm && basicModel.standardVoltage >= 10.0 && actualVoltage >= 10.0)
	{
		requiredPwm *= fsquare(basicModel.standardVoltage/actualVoltage);
	}
	return min<float>(requiredPwm, maxPwm);
}

// Calculate the fan cooling rate. We assume that the cooling rate exponent does not apply to fan cooling.
float FopDt::GetFanCoolingRate(float temperatureRise, float fanPwm) const noexcept
{
#if SQRT_FAN_SCALING
	return temperatureRise * 0.01 * basicModel.fanCoolingRate * fastSqrtf(fanPwm);
#else
	return temperatureRise * 0.01 * basicModel.fanCoolingRate * fanPwm;
#endif
}

// Calculate the change in required heater PWM due to a change in fan PWM
float FopDt::GetPwmCorrectionForFan(float temperatureRise, float oldFanPwm, float newFanPwm) const noexcept
{
#if SQRT_FAN_SCALING
	return temperatureRise * 0.01 * basicModel.fanCoolingRate * (fastSqrtf(newFanPwm) - fastSqrtf(oldFanPwm)) / basicModel.heatingRate;
#else
	return temperatureRise * 0.01 * basicModel.fanCoolingRate * (newFanPwm - oldFanPwm) / basicModel.heatingRate;
#endif
}

// Calculate the expected cooling rate for a given temperature rise above ambient
float FopDt::GetCoolingRate(float temperatureRise, float fanPwm) const noexcept
{
	temperatureRise *= 0.01;
	// If the temperature rise is negative then we must not try to raise it to a non-integral power!
	const float adjustedTemperatureRise = (temperatureRise < 0.0) ? -powf(-temperatureRise, basicModel.coolingRateExponent) : powf(temperatureRise, basicModel.coolingRateExponent);
	return basicModel.basicCoolingRate * adjustedTemperatureRise + GetFanCoolingRate(temperatureRise, fanPwm);
}

// Get an estimate of the expected heating rate at the specified temperature rise and PWM. The result may be negative.
float FopDt::GetNetHeatingRate(float temperatureRise, float fanPwm, float heaterPwm) const noexcept
{
	return basicModel.heatingRate * heaterPwm - GetCoolingRate(temperatureRise, fanPwm);
}

// Get an estimate of the heater PWM required to maintain a specified temperature
float FopDt::EstimateRequiredPwm(float temperatureRise, float fanPwm) const noexcept
{
	return GetCoolingRate(temperatureRise, fanPwm)/basicModel.heatingRate;
}

/*static*/ float FopDt::EstimateMaxTemperatureRise(float hr, float cr, float cre) noexcept
{
	return 100.0 * powf(hr/cr, 1.0/cre);
}

// End
