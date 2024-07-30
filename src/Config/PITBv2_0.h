/*
 * PITBv1_0.h
 *
 *  Created on: 19 June 2023
 *      Author: jay_s_uk
 */

#ifndef SRC_CONFIG_PITB_V2_0_H_
#define SRC_CONFIG_PITB_V2_0_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"PITBV2_0"
#define BOOTLOADER_NAME		"PITBV2_0"
#define BOARD_USES_UF2_BINARY	1

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0
#define HAS_USB_SERIAL			1
#define USE_SERIAL_DEBUG		0

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
#define SUPPORT_INPUT_SHAPING	1

constexpr size_t NumDrivers = 2;
constexpr size_t MaxSmartDrivers = 2;
constexpr float MaxTmc5160Current = 6300.0;
constexpr uint32_t DefaultStandstillCurrentPercent = 71;
constexpr float Tmc5160SenseResistor = 0.075;

#if TMC51xx_USES_SEPARATE_ENABLE
constexpr Pin Tmc51xxEnablePins[] = {GpioPin(20), GpioPin(22)};
#else
constexpr Pin GlobalTmc51xxEnablePin = GpioPin(5);
#endif
#if TMC51xx_USES_SEPARATE_CS
constexpr Pin Tmc51xxCSPins[] = {GpioPin(1), GpioPin(21)};
#else
constexpr Pin GlobalTmc51xxCSPin = GpioPin(6);
#endif

constexpr Pin DirectionPins[NumDrivers] = { GpioPin(5), GpioPin(23) };
constexpr Pin StepPins[NumDrivers] = { GpioPin(6), GpioPin(13) };


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

constexpr Pin CanTxPin = GpioPin(NoPin);
constexpr Pin CanRxPin = GpioPin(NoPin);

constexpr Pin ButtonPins[] = { PIN_TODO };

#if HAS_VOLTAGE_MONITOR

// VIN voltage monitor
constexpr Pin VinMonitorPin = GpioPin(29);
constexpr float VinDividerRatio = 100/5.1;
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
constexpr Pin SSPIMisoPin = GpioPin(4);
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
	{ PwmOutput::pwm0a,	AdcInput::none,		"out0"		},	// GPIO0 FAN0
	{ PwmOutput::pwm0b,	AdcInput::none,		nullptr		},	// GPIO1 MOT0 CS
	{ PwmOutput::pwm1a,	AdcInput::none,		nullptr		},	// GPIO2 DRIVER SPI SCK
	{ PwmOutput::pwm1b,	AdcInput::none,		nullptr		},	// GPIO3 DRIVER SPI MOSI
	{ PwmOutput::pwm2a,	AdcInput::none,		nullptr		},	// GPIO4 DRIVER SPI MISO
	{ PwmOutput::pwm2b,	AdcInput::none,		nullptr		},	// GPIO5 MOT0 DIR
	{ PwmOutput::pwm3a,	AdcInput::none,		nullptr		},	// GPIO6 MOT0 STEP
	{ PwmOutput::pwm3b,	AdcInput::none,		nullptr		},	// GPIO7 MOT0 DIAG
	{ PwmOutput::pwm4a,	AdcInput::none,		nullptr		},	// GPIO8 SPI1 MISO
	{ PwmOutput::pwm4b,	AdcInput::none,		nullptr		},	// GPIO9 SPI1 CS
	{ PwmOutput::pwm5a,	AdcInput::none,		nullptr		},	// GPIO10 SPI1 SCK
	{ PwmOutput::pwm5b,	AdcInput::none,		nullptr		},	// GPIO11 SPI1 MOSI
	{ PwmOutput::pwm6a,	AdcInput::none,		nullptr 	},	// GPIO12 MCP INT
	{ PwmOutput::pwm6b,	AdcInput::none,		nullptr 	},	// GPIO13 MOT1 STEP
	{ PwmOutput::pwm7a,	AdcInput::none,		nullptr		},	// GPIO14 MOT1 DIAG
	{ PwmOutput::pwm7b,	AdcInput::none,		nullptr		},	// GPIO15 STATUS LED
	{ PwmOutput::pwm0a,	AdcInput::none,		"io0.in"	},	// GPIO16 XSTOP
	{ PwmOutput::pwm0b,	AdcInput::none,		"io1.in"	},	// GPIO17 YSTOP
	{ PwmOutput::pwm1a,	AdcInput::none,		"out1"	 	},	// GPIO18 FAN1
	{ PwmOutput::pwm1b,	AdcInput::none,		"rgbled"	},	// GPIO19 Neopixel
	{ PwmOutput::pwm2a,	AdcInput::none,		nullptr		},	// GPIO20 MOT0 EN
	{ PwmOutput::pwm2b,	AdcInput::none,		nullptr		},	// GPIO21 MOT1 CS
	{ PwmOutput::pwm3a,	AdcInput::none,		nullptr		},	// GPIO22 MOT1 EN
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO23 MOT1 DIR
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO24 SDA
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO25 SCL
	{ PwmOutput::pwm5a,	AdcInput::adc0_0,	"temp0"		},	// GPIO26 T0_TEMP
	{ PwmOutput::pwm5b,	AdcInput::adc0_1,	"temp1"		},	// GPIO27 T1_TEMP
	{ PwmOutput::pwm6a,	AdcInput::adc0_2,	"temp2"		},	// GPIO28 T2_TEMP
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

#if SUPPORT_CAN && USE_SPICAN
constexpr uint8_t spiCanSpiInstanceNumber = 1;
constexpr Pin SPICanMosiPin = GpioPin(11);
constexpr GpioPinFunction SPICanMosiPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SPICanSclkPin = GpioPin(10);
constexpr GpioPinFunction SPICanSclkPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SPICanMisoPin = GpioPin(8);
constexpr GpioPinFunction SPICanMisoPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SPICanCsPin = GpioPin(9);
#endif


#endif /* SRC_CONFIG_PITB_V2_0_H_ */
