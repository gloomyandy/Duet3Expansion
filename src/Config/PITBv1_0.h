/*
 * Fly36_RRF.h
 *
 *  Created on: 19 June 2023
 *      Author: jay_s_uk
 */

#ifndef SRC_CONFIG_PITB_V1_0_H_
#define SRC_CONFIG_PITB_V1_0_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"PITBV1_0"
#define BOOTLOADER_NAME		"PITBV1_0"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0
#define HAS_USB_SERIAL			1
#define USE_SERIAL_DEBUG		1

// Drivers configuration
#define SUPPORT_DRIVERS			1

#if SUPPORT_DRIVERS

#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			0
#define TMC51xx_USES_SEPARATE_CS		1
#define TMC51xx_USES_SEPARATE_ENABLE	1
#define TMC51xx_USES_SHARED_SPI	1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0

#define SUPPORT_TMC51xx			1
#define SUPPORT_TMC2160			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2240			0

constexpr size_t NumDrivers = 2;
constexpr size_t MaxSmartDrivers = 2;
constexpr float MaxTmc5160Current = 6300.0;
constexpr uint32_t DefaultStandstillCurrentPercent = 71;
constexpr float Tmc5160SenseResistor = 0.075;

#if TMC51xx_USES_SEPARATE_ENABLE
constexpr Pin Tmc51xxEnablePins[] = {GpioPin(5), GpioPin(9)};
#else
constexpr Pin GlobalTmc51xxEnablePin = GpioPin(5);
#endif
#if TMC51xx_USES_SEPARATE_CS
constexpr Pin Tmc51xxCSPins[] = {GpioPin(6), GpioPin(10)};
#else
constexpr Pin GlobalTmc51xxCSPin = GpioPin(6);
#endif

constexpr Pin StepPins[NumDrivers] = { GpioPin(8), GpioPin(12) };
constexpr Pin DirectionPins[NumDrivers] = { GpioPin(7), GpioPin(11) };


#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#endif

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_LIS3DH			0
#define SUPPORT_DHT_SENSOR		0

#define USE_MPU					0
#define USE_CACHE				0

#define PIN_TODO	GpioPin(NoPin)	//TEMPORARY! Used when we haven't assigned a pin yet.

constexpr bool UseAlternateCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 1;
constexpr float DefaultThermistorSeriesR = 4700.0;		// TEMP0 has 1K or 4K7 pullup, chamber thermistor has 4K7

constexpr Pin TempSensePins[NumThermistorInputs] = { GpioPin(26)};

constexpr Pin CanTxPin = GpioPin(13);
constexpr Pin CanRxPin = GpioPin(14);

constexpr Pin ButtonPins[] = { PIN_TODO };

#if HAS_VOLTAGE_MONITOR

// VIN voltage monitor
constexpr Pin VinMonitorPin = GpioPin(28);
constexpr float VinDividerRatio = (47.0 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;				// the Pico uses the 3.3V supply as the voltage reference

#endif

// Diagnostic LEDs
constexpr Pin LedPins[] = { GpioPin(15) };
constexpr bool LedActiveHigh = false;

#if SUPPORT_SPI_SENSORS || TMC51xx_USES_SHARED_SPI

// Shared SPI pin connections
constexpr uint8_t SspiSpiInstanceNumber = 0;
constexpr Pin SSPIMosiPin = GpioPin(3);
constexpr GpioPinFunction SSPIMosiPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SSPISclkPin = GpioPin(2);
constexpr GpioPinFunction SSPISclkPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SSPIMisoPin = GpioPin(0);
constexpr GpioPinFunction SSPIMisoPinPeriphMode = GpioPinFunction::Spi;

#endif

#if SUPPORT_LIS3DH

#define ACCELEROMETER_USES_SPI			(1)					// 0 if the accelerometer is connected via I2C, 1 if via SPI
constexpr Pin Lis3dhCsPin = GpioPin(9);
constexpr Pin Lis3dhInt1Pin = GpioPin(29);

#endif

// Table of pin functions that we are allowed to use
//TODO restrict each of pwm0 to pwm7 to just one output, to prevent users trying to use the same PWM unit for more than one pin
constexpr PinDescription PinTable[] =
{
	//	PWM					ADC				PinName
	// Port A
	{ PwmOutput::pwm0a,	AdcInput::none,		nullptr		},	// GPIO0 DRIVER SPI MISO
	{ PwmOutput::pwm0b,	AdcInput::none,		nullptr		},	// GPIO1
	{ PwmOutput::pwm1a,	AdcInput::none,		nullptr		},	// GPIO2 DRIVER SPI SCK
	{ PwmOutput::pwm1b,	AdcInput::none,		nullptr		},	// GPIO3 DRIVER SPI MOSI
	{ PwmOutput::pwm2a,	AdcInput::none,		nullptr		},	// GPIO4
	{ PwmOutput::pwm2b,	AdcInput::none,		nullptr		},	// GPIO5 MOT1 EN
	{ PwmOutput::pwm3a,	AdcInput::none,		nullptr		},	// GPIO6 MOT1 CS
	{ PwmOutput::pwm3b,	AdcInput::none,		nullptr		},	// GPIO7 MOT1 DIR
	{ PwmOutput::pwm4a,	AdcInput::none,		nullptr		},	// GPIO8 MOT1 STEP
	{ PwmOutput::pwm4b,	AdcInput::none,		nullptr		},	// GPIO9 MOT2 EN
	{ PwmOutput::pwm5a,	AdcInput::none,		nullptr		},	// GPIO10 MOT2 CS
	{ PwmOutput::pwm5b,	AdcInput::none,		nullptr		},	// GPIO11 MOT2 DIR
	{ PwmOutput::pwm6a,	AdcInput::none,		nullptr 	},	// GPIO12 MOT2 STEP
	{ PwmOutput::pwm6b,	AdcInput::none,		nullptr 	},	// GPIO13 CAN TX
	{ PwmOutput::pwm7a,	AdcInput::none,		nullptr		},	// GPIO14 CAN RX
	{ PwmOutput::pwm7b,	AdcInput::none,		nullptr		},	// GPIO15 STATUS LED
	{ PwmOutput::pwm0a,	AdcInput::none,		nullptr		},	// GPIO16 SDA
	{ PwmOutput::pwm0b,	AdcInput::none,		nullptr		},	// GPIO17 SCL
	{ PwmOutput::pwm1a,	AdcInput::none,		nullptr 	},	// GPIO18
	{ PwmOutput::pwm1b,	AdcInput::none,		nullptr		},	// GPIO19
	{ PwmOutput::pwm2a,	AdcInput::none,		"out0"		},	// GPIO20 FAN0
	{ PwmOutput::pwm2b,	AdcInput::none,		"out1"		},	// GPIO21 FAN1
	{ PwmOutput::pwm3a,	AdcInput::none,		nullptr		},	// GPIO22
	{ PwmOutput::none,	AdcInput::none,		"io0.in"	},	// GPIO23 XSTOP
	{ PwmOutput::none,	AdcInput::none,		"io1.in"	},	// GPIO24 YSTOP
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO25
	{ PwmOutput::pwm5a,	AdcInput::adc0_0,	"temp0"		},	// GPIO26 T0_TEMP
	{ PwmOutput::pwm5b,	AdcInput::adc0_1,	nullptr		},	// GPIO27
	{ PwmOutput::pwm6a,	AdcInput::adc0_2,	nullptr		},	// GPIO28
	{ PwmOutput::none,	AdcInput::adc0_3,	"rgbled"	},	// GPIO29 ACC_INT1
};

static constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static constexpr size_t NumRealPins = 30;				// 30 GPIO pins on RP2040
static_assert(NumPins == NumRealPins);					// no virtual pins

// Timer/counter used to generate step pulses and other sub-millisecond timings
constexpr unsigned int StepTimerAlarmNumber = 0;
constexpr unsigned int StepTcIRQn = TIMER_IRQ_0;

// Available UART ports
#define NUM_SERIAL_PORTS		1
//constexpr IRQn Serial0_IRQn = SERCOM5_IRQn;

// DMA channel assignments
constexpr DmaChannel DmacChanCAN = 0;					// this must match the value used in the RP2040 CAN driver in CoreN2G!
constexpr DmaChannel DmacChanAdcRx = 1;
constexpr DmaChannel DmacChanTmcTx = 2;
constexpr DmaChannel DmacChanTmcRx = 3;					// this must be one higher than DmacChanTmcTx for RP2040 build configurations
constexpr DmaChannel DmacChanCRC = 4;
constexpr DmaChannel DmaChanWS2812 = 5;

constexpr unsigned int NumDmaChannelsUsed = 6;			// must be at least the number of channels used, may be larger. Max 12 on the RP2040.

// DMA priorities, higher is better. RP2040 has only 0 and 1.
constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 1;
constexpr DmaPriority DmacPrioAdcRx = 1;

// Interrupt priorities, lower means higher priority. Only 0 to 3 are available.
const NvicPriority NvicPriorityStep = 1;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 2;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 2;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityI2C = 2;
const NvicPriority NvicPriorityCan = 3;
const NvicPriority NvicPriorityDmac = 3;				// priority for DMA complete interrupts
const NvicPriority NvicPriorityAdc = 3;
const NvicPriority NvicPriorityUSB = 3;

#endif /* SRC_CONFIG_FLY36_RRF_H_ */