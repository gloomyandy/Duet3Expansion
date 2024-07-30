/*
 * FlySHT36.h
 *
 *  Created on: 18 Nov 2022
 *      Author: GA
 */
/* Supported board revisions
	REV 0-2 : Klipper version (not supported)
	REV 300 : New spican board with scanning sensor prototype
	REV 301 : New spican board with scanning sensor production
*/

#ifndef SRC_CONFIG_SHT36_H_
#define SRC_CONFIG_SHT36_H_

#if BOARD_REV < 300
#error "Unsupported board version"
#endif

#include <Hardware/PinDescription.h>
#if BOARD_REV == 300
#define BOARD_TYPE_NAME		"SHT36V3"
#define BOOTLOADER_NAME		"SHT36V3"
#elif BOARD_REV == 301
#define BOARD_TYPE_NAME		"SHT36MAX3"
#define BOOTLOADER_NAME		"SHT36MAX3"
#else
#error "Unsupported board version"
#endif

#define BOARD_USES_UF2_BINARY	1

// General features
#define HAS_VREF_MONITOR		0
#if BOARD_REV == 300
#define HAS_VOLTAGE_MONITOR		1
#elif BOARD_REV == 301
#define HAS_VOLTAGE_MONITOR		0
#endif
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0
#define HAS_USB_SERIAL			1
#define USE_SERIAL_DEBUG		0
#define SUPPORT_LED_STRIPS		1
#define SUPPORT_PIO_NEOPIXEL	1
#define SUPPORT_INPUT_SHAPING	1

// Drivers configuration
#define SUPPORT_DRIVERS			1

#if SUPPORT_DRIVERS

#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		0
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2160			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			1
#define SUPPORT_TMC2240			0

constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;

#define TMC22xx_HAS_MUX					0
#define TMC22xx_SINGLE_DRIVER			1
#define TMC22xx_HAS_ENABLE_PINS			0
#define TMC22xx_VARIABLE_NUM_DRIVERS	0
#define TMC22xx_USE_SLAVEADDR			0

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// In testing I found that 500kbaud was not reliable on the Duet Maestro, so now using 200kbaud.
constexpr uint32_t DriversBaudRate = 200000;
constexpr uint32_t TransferTimeout = 10;									// any transfer should complete within 10 ticks @ 1ms/tick

constexpr float DriverSenseResistor = 0.11 + 0.02;							// in ohms
constexpr float DriverVRef = 180.0;											// in mV
constexpr float DriverFullScaleCurrent = DriverVRef/DriverSenseResistor;	// in mA
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;
constexpr float MaximumMotorCurrent = 1600.0;
constexpr float MaximumStandstillCurrent = 1200.0;
constexpr uint32_t DefaultStandstillCurrentPercent = 75;

constexpr Pin GlobalTmc22xxEnablePin = GpioPin(14);
constexpr Pin Tmc22xxUartPin = GpioPin(15);

constexpr Pin StepPins[NumDrivers] = { GpioPin(7) };
constexpr Pin DirectionPins[NumDrivers] = { GpioPin(6) };

#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#endif

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		1
#define SUPPORT_I2C_SENSORS		1
#define SUPPORT_LIS3DH			1
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_LDC1612			1

#define USE_MPU					0
#define USE_CACHE				0

#define PIN_TODO	GpioPin(NoPin)	//TEMPORARY! Used when we haven't assigned a pin yet.

constexpr bool UseAlternateCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 2;
constexpr float DefaultThermistorSeriesR = 4700.0;		// TEMP0 has 1K or 4K7 pullup, chamber thermistor has 4K7

constexpr Pin TempSensePins[NumThermistorInputs] = { GpioPin(27), GpioPin(28) };

constexpr Pin CanTxPin = GpioPin(0);
constexpr Pin CanRxPin = GpioPin(1);

constexpr Pin ButtonPins[] = { PIN_TODO };

// VIN voltage monitor
#if HAS_VOLTAGE_MONITOR
constexpr Pin VinMonitorPin = GpioPin(29);
constexpr float VinDividerRatio = (47.0 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;				// the Pico uses the 3.3V supply as the voltage reference
#endif

// Diagnostic LEDs
constexpr Pin LedPins[] = { GpioPin(5) };
constexpr bool LedActiveHigh = false;

#if SUPPORT_SPI_SENSORS

// Shared SPI pin connections
constexpr uint8_t SspiSpiInstanceNumber = 0;
constexpr Pin SSPIMosiPin = GpioPin(3);
constexpr GpioPinFunction SSPIMosiPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SSPISclkPin = GpioPin(2);
constexpr GpioPinFunction SSPISclkPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SSPIMisoPin = GpioPin(4);
constexpr GpioPinFunction SSPIMisoPinPeriphMode = GpioPinFunction::Spi;
#endif

#if SUPPORT_I2C_SENSORS

// I2C using pins 18,19
constexpr uint8_t I2CInstanceNumber = 1;
constexpr Pin I2CSDAPin = GpioPin(18);
constexpr GpioPinFunction I2CSDAPinPeriphMode = GpioPinFunction::I2c;
constexpr Pin I2CSCLPin = GpioPin(19);
constexpr GpioPinFunction I2CSCLPinPeriphMode = GpioPinFunction::I2c;
#endif

#if SUPPORT_LIS3DH

#define ACCELEROMETER_USES_SPI			(1)					// 0 if the accelerometer is connected via I2C, 1 if via SPI
constexpr Pin Lis3dhCsPin = GpioPin(12);
constexpr Pin Lis3dhInt1Pin = GpioPin(25);

#endif

#if SUPPORT_LDC1612
constexpr uint16_t LDC1612_I2CAddress = 0x2B;				// pin 4 is tied high on the Grove board
constexpr Pin LDC1612InterruptPin = GpioPin(29);			// this is brought out to a test pad
#endif

// Table of pin functions that we are allowed to use

//TODO restrict each of pwm0 to pwm7 to just one output, to prevent users trying to use the same PWM unit for more than one pin
constexpr PinDescription PinTable[] =
{
	//	PWM					ADC				PinName
	// Port A
	{ PwmOutput::pwm0a,	AdcInput::none,		nullptr		},	// GPIO0 CAN_TX - picocan?
	{ PwmOutput::pwm0b,	AdcInput::none,		nullptr		},	// GPIO1 CAN_RX - picocan?
	{ PwmOutput::pwm1a,	AdcInput::none,		nullptr		},	// GPIO2 SPI0_SCK
	{ PwmOutput::pwm1b,	AdcInput::none,		nullptr		},	// GPIO3 SPIO_MOSI
	{ PwmOutput::pwm2a,	AdcInput::none,		nullptr		},	// GPIO4 SPIO_MISO
	{ PwmOutput::pwm2b,	AdcInput::none,		nullptr		},	// GPIO5 status LED
	{ PwmOutput::pwm3a,	AdcInput::none,		nullptr		},	// GPIO6 DIR
	{ PwmOutput::pwm3b,	AdcInput::none,		nullptr		},	// GPIO7 STEP
	{ PwmOutput::pwm4a,	AdcInput::none,		nullptr		},	// GPIO8 SPI1_MISO
	{ PwmOutput::pwm4b,	AdcInput::none,		nullptr		},	// GPIO9 SPI1_CS
	{ PwmOutput::pwm5a,	AdcInput::none,		nullptr		},	// GPIO10 SPI1_SCLK
	{ PwmOutput::pwm5b,	AdcInput::none,		nullptr		},	// GPIO11 SPI1_MOSI
	{ PwmOutput::pwm6a,	AdcInput::none,		nullptr 	},	// GPIO12 accelerometer CS
	{ PwmOutput::pwm6b,	AdcInput::none,		"out1"		},	// GPIO13 FAN0
	{ PwmOutput::pwm7a,	AdcInput::none,		nullptr		},	// GPIO14 EN
	{ PwmOutput::pwm7b,	AdcInput::none,		nullptr		},	// GPIO15 TMC UART
	{ PwmOutput::pwm0a,	AdcInput::none,		"io2.in"	},	// GPIO16 ENDSTOP2
	{ PwmOutput::pwm0b,	AdcInput::none,		"max31865cs,rtdcs" },	// GPIO17 MAX31865_CS
	{ PwmOutput::pwm1a,	AdcInput::none,		nullptr 	},	// GPIO18 I2C_SDA
	{ PwmOutput::pwm1b,	AdcInput::none,		nullptr		},	// GPIO19 I2C_SDL
	{ PwmOutput::pwm2a,	AdcInput::none,		"io1.in"	},	// GPIO20 ENDSTOP1
	{ PwmOutput::pwm2b,	AdcInput::none,		"out2"		},	// GPIO21 FAN1
	{ PwmOutput::pwm3a,	AdcInput::none,		"io0.in"	},	// GPIO22 PROBE
	{ PwmOutput::pwm3b,	AdcInput::none,		"out0"		},	// GPIO23 HEAT0
	{ PwmOutput::pwm4a,	AdcInput::none,		"io0.out"	},	// GPIO24 SERVO
	{ PwmOutput::none,	AdcInput::none,		nullptr		},	// GPIO25 ACC_INT1
	{ PwmOutput::pwm5a,	AdcInput::adc0_0,	"rgbled"	},	// GPIO26 RGB
	{ PwmOutput::pwm5b,	AdcInput::adc0_1,	"temp0"		},	// GPIO27 TEMP0
	{ PwmOutput::pwm6a,	AdcInput::adc0_2,	"temp1"		},	// GPIO28 CHAMBER_TEMP
	{ PwmOutput::none,	AdcInput::adc0_3,	nullptr		},	// GPIO29 VIN ADC/ldc1612 int pin
	// Virtual pins
#if SUPPORT_LDC1612
	{ PwmOutput::none,	AdcInput::ldc1612,	"i2c.ldc1612"},	// LDC1612 sensor connected via I2C
#endif
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static constexpr size_t NumRealPins = 30;				// 30 GPIO pins on RP2040
constexpr size_t NumVirtualPins = SUPPORT_LDC1612;

static_assert(NumPins == NumRealPins + NumVirtualPins);

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
#endif /* SRC_CONFIG_FLY36_RRF_H_ */
