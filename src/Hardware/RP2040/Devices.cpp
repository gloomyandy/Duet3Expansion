/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if RPXXXX

#include <AnalogIn.h>
#include <AnalogOut.h>
#include <Platform/TaskPriorities.h>
#include <RTOSIface/RTOSIface.h>
#include <TinyUsbInterface.h>
#include <SerialCDC_tusb.h>
#if SUPPORT_CAN && USE_SPICAN
# include <Platform.h>
# include <SharedSpiClient.h>
# include <CanSpi.h>
# include "StepTimer.h"
#endif

// Analog input support
constexpr size_t AnalogInTaskStackWords = 300;
static Task<AnalogInTaskStackWords> analogInTask;

constexpr size_t UsbDeviceTaskStackWords = 200;
static Task<UsbDeviceTaskStackWords> usbDeviceTask;

SerialCDC serialUSB;

void DeviceInit() noexcept
{
	AnalogIn::Init(DmacChanAdcRx, DmacPrioAdcRx);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);

	CoreUsbInit(NvicPriorityUSB);
	usbDeviceTask.Create(CoreUsbDeviceTask, "USBD", nullptr, TaskPriority::UsbPriority);
}

#if SUPPORT_CAN && USE_SPICAN
// SPICAN SPI interface
SharedSpiClient *spiCanHardware;
extern "C" bool DRV_SPI_Initialize()
{
	debugPrintf("SPI init start\n");
	spiCanHardware = new SharedSpiClient(Platform::GetSharedSpi(spiCan_SpiChannel), 15000000, SpiMode::mode0, NoPin, false);
	IoPort::SetPinMode(SPICanCsPin, OUTPUT_HIGH);
	spiCanHardware->Select(1000);
	debugPrintf("SPI init complete\n");
    return true;
}

extern "C" void DRV_SPI_Select()
{
}

extern "C" void DRV_SPI_Deselect()
{
}

extern "C" int8_t DRV_SPI_TransferData(uint32_t index, uint8_t *SpiTxData, uint8_t *SpiRxData, size_t spiTransferSize)
{
	IoPort::WriteDigital(SPICanCsPin, 0);
	const bool ret = spiCanHardware->TransceivePacket(SpiTxData, SpiRxData, spiTransferSize);
	IoPort::WriteDigital(SPICanCsPin, 1);
	return !ret;
}

extern "C" uint32_t DRV_SPI_GetStepTimerTicks()
{
    return StepTimer::GetTimerTicks();
}

#endif
#endif

// End
