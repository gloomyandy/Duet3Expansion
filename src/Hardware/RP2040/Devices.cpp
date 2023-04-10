/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if RP2040

#include <AnalogIn.h>
#include <AnalogOut.h>
#include <Platform/TaskPriorities.h>
#include <RTOSIface/RTOSIface.h>
#include <TinyUsbInterface.h>
#include <SerialCDC_tusb.h>
#if SUPPORT_CAN && USE_SPICAN
# include <CanSpi.h>
# include <GPIO/GpioPorts.h>
# include "hardware/spi.h"
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
spi_inst_t *spiCanHardware = (spiCanSpiInstanceNumber == 0) ? spi0 : spi1;
extern "C" bool DRV_SPI_Initialize()
{
	debugPrintf("SPI init start\n");
	SetPinFunction(SPICanMosiPin, SPICanMosiPinPeriphMode);
	SetPinFunction(SPICanSclkPin, SPICanSclkPinPeriphMode);
	SetPinFunction(SPICanMisoPin, SPICanMisoPinPeriphMode);
	IoPort::SetPinMode(SPICanCsPin, OUTPUT_HIGH);
	uint32_t ret = spi_init(spiCanHardware, 15000000);
	debugPrintf("SPI init freq %d\n", (int)ret);
	spi_set_format(spiCanHardware, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
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
	const int bytesTransferred = (SpiRxData == nullptr) ? spi_write_blocking(spiCanHardware, SpiTxData, spiTransferSize)
								: (SpiTxData == nullptr) ? spi_read_blocking(spiCanHardware, 0xFF, SpiRxData, spiTransferSize)
									: spi_write_read_blocking(spiCanHardware, SpiTxData, SpiRxData, spiTransferSize);
	IoPort::WriteDigital(SPICanCsPin, 1);
	return !(bytesTransferred == (int)spiTransferSize);
}
#endif
#endif

// End
