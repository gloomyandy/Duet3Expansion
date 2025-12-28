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

class TPiS_1T_1086_L5_5 : public I2CTemperatureSensor
{
public:
	TPiS_1T_1086_L5_5(unsigned int sensorNum);
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) override;

	static constexpr const char *TypeName = "tpis1t1086";

	void Poll() override;

private:
	static SensorTypeDescriptor typeDescriptor;
};

#endif

#endif /* SRC_HEATING_SENSORS_TPIS_1T_1086_L5_5_H_ */
