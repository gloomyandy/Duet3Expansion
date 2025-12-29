/*
 * TOOLINDX.h
 *
 *  Created on: 20 Dec 2025
 *      Author: David
 */

#ifndef SRC_CONFIG_TOOLINDX_H_
#define SRC_CONFIG_TOOLINDX_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"TOOLINDX"
#define BOOTLOADER_NAME		"SAME5x"	// temporary until we have the cmposite bootloader

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0

// Drivers configuration
#define SUPPORT_DRIVERS			1
#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	0
#define DEDICATED_STEP_TIMER	1
#define SUPPORT_INPUT_SHAPING	1

#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			1
#define SUPPORT_TMC2208			0
#define SUPPORT_TMC2209			0
#define SUPPORT_TMC2240			1

constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;

#define TMC22xx_USES_SERCOM				1
#define TMC22xx_HAS_MUX					0
#define TMC22xx_SINGLE_DRIVER			1
#define TMC22xx_HAS_ENABLE_PINS			0
#define TMC22xx_VARIABLE_NUM_DRIVERS	0
#define TMC22xx_USE_SLAVEADDR			0

constexpr Pin GlobalTmc22xxEnablePin = PortAPin(0);

//TODO change the following for the TMC2240 SPI driver
constexpr uint8_t TMC22xxSercomNumber = 0;
Sercom * const SERCOM_TMC22xx = SERCOM0;

constexpr Pin TMC22xxSercomTxPin = PortAPin(8);
constexpr GpioPinFunction TMC22xxSercomTxPinPeriphMode = GpioPinFunction::C;
constexpr Pin TMC22xxSercomRxPin = PortAPin(9);
constexpr GpioPinFunction TMC22xxSercomRxPinPeriphMode = GpioPinFunction::C;
constexpr uint8_t TMC22xxSercomRxPad = 1;

// Define the baud rate used to send/receive data to/from the drivers.
// If we assume a worst case clock frequency of 8MHz then the maximum baud rate is 8MHz/16 = 500kbaud.
// We send data via a 1K series resistor. Even if we assume a 200pF load on the shared UART line, this gives a 200ns time constant, which is much less than the 2us bit time @ 500kbaud.
// To write a register we need to send 8 bytes. To read a register we send 4 bytes and receive 8 bytes after a programmable delay.
// So at 500kbaud it takes about 128us to write a register, and 192us+ to read a register.
// In testing I found that 500kbaud was not reliable on the Duet Maestro, so now using 200kbaud.
constexpr uint32_t DriversBaudRate = 200000;
constexpr uint32_t TransferTimeout = 10;									// any transfer should complete within 10 ticks @ 1ms/tick

//TODO adjust the following according the Rref we use
constexpr uint32_t Tmc2240CurrentRange = 0x01;								// which current range we set the TMC2240 to (2A)
constexpr uint32_t Tmc2240SlopeControl = 0x01;								// which slope control we set the TMC2240 to (200V/us)
constexpr float Tmc2240Rref = 15.0;											// TMC2240 reference resistor in Kohms
constexpr float DriverFullScaleCurrent = 24000/Tmc2240Rref;					// in mA, assuming we set the range bits in the DRV_CONF register to 0x01
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;			// with RRef = 15K this works out as 1.6A so this is the maximum current we can ask for

constexpr float MaximumMotorCurrent = 1000.0;
constexpr float MaximumStandstillCurrent = 1000.0;

constexpr uint32_t DefaultStandstillCurrentPercent = 75;

PortGroup * const StepPio = &(PORT->Group[1]);								// the PIO that all the step pins are on
constexpr Pin StepPins[NumDrivers] = { PortBPin(23) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(10) };
constexpr Pin DriverDiagPins[NumDrivers] = { PortAPin(21) };

#define SUPPORT_THERMISTORS			1
#define SUPPORT_SPI_SENSORS			0										// SPI temperature sensors not supported
#define SUPPORT_LDC1612				1
#define SUPPORT_TPiS_1T_1086_L5_5	1										// IR temperature sensor
#define SUPPORT_AS5601				0										// support direct-connected magnetic filament monitor encoder chip
#define SUPPORT_DMA_NEOPIXEL		1										// using QSPI for Neopixels
#define SUPPORT_INDUCTIVE_HEATER	1										// Inductive heater support
#define SUPPORT_LP5817				1										// LP5817 LED driver support

#ifdef DEBUG
# define NUM_I2C_CHANNELS		0											// in debug mode the SERCOM is used for debugging
# define SUPPORT_LIS3DH			0
#else
# define NUM_I2C_CHANNELS		2
# define SUPPORT_LIS3DH			1
#endif

#define SUPPORT_DHT_SENSOR		0
#define NUM_SERIAL_PORTS		0

#define USE_MPU					0
#define USE_CACHE				1

constexpr int CANInstanceNumber = 1;
constexpr bool UseLaterCanPins = true;

constexpr size_t MaxPortsPerHeater = 1;										// we support a single heater

constexpr size_t NumThermistorInputs = 3;
constexpr float DefaultThermistorSeriesR = 2200.0;							// TODO (we will use a higher value)
constexpr float DefaultThermistorR25_TOOL1RR_temp2 = 10000;					// TODO
constexpr float DefaultThermistorBeta_TOOL1RR_temp2 = 3425.0;				// TODO
constexpr float DefaultThermistorC_TOOL1RR_temp2 = 1.68e-7;					// TODO

// We don't use a Vref/Vssa calibration chain on this board

constexpr Pin BoardTypePin = PortAPin(3);

// Diagnostic LEDs
constexpr Pin LedPins[] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh = false;

constexpr Pin VinMonitorPin = PortAPin(2);
constexpr float VinDividerRatio = (60.4 + 4.7)/4.7;							// to be confirmed
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;

constexpr Pin TempSensePins[NumThermistorInputs] = { PortBPin(8), PortBPin(9), PortAPin(11) };

#if NUM_I2C_CHANNELS >= 1

// I2C0 using pins PA22,23
constexpr uint8_t I2C0SercomNumber = 3;
constexpr Pin I2C0SDAPin = PortAPin(23);
constexpr GpioPinFunction I2C0SDAPinPeriphMode = GpioPinFunction::C;
constexpr Pin I2C0SCLPin = PortAPin(22);
constexpr GpioPinFunction I2C0SCLPinPeriphMode = GpioPinFunction::C;
# define I2C0_HANDLER0		SERCOM3_0_Handler
# define I2C0_HANDLER1		SERCOM3_1_Handler
# define I2C0_HANDLER2		SERCOM3_2_Handler
# define I2C0_HANDLER3		SERCOM3_3_Handler

#endif

#if NUM_I2C_CHANNELS >= 2

// I2C1 using pins PA12,13
constexpr uint8_t I2C1SercomNumber = 4;
constexpr Pin I2C1SDAPin = PortAPin(13);
constexpr GpioPinFunction I2C1SDAPinPeriphMode = GpioPinFunction::D;
constexpr Pin I2C1SCLPin = PortAPin(13);
constexpr GpioPinFunction I2C1SCLPinPeriphMode = GpioPinFunction::D;
# define I2C1_HANDLER0		SERCOM4_0_Handler
# define I2C1_HANDLER1		SERCOM4_1_Handler
# define I2C1_HANDLER2		SERCOM4_2_Handler
# define I2C1_HANDLER3		SERCOM4_3_Handler

#endif

#if SUPPORT_LIS3DH												// we actually use a LIS2DW
# define ACCELEROMETER_USES_SPI			(0)						// accelerometer is connected via I2C
constexpr unsigned int Lis_I2CChannel = 0;
constexpr Pin Lis3dhInt1Pin = PortAPin(27);
#endif

#if SUPPORT_LDC1612
constexpr unsigned int LDC1612_I2CChannel = 0;
constexpr uint16_t LDC1612_I2CAddress = 0x2A;					// pin 4 is tied low
constexpr unsigned int Ldc1612GClkNumber = 5;
constexpr Pin LDC1612ClockGenPin = PortBPin(11);
constexpr Pin LDC1612InterruptPin = PortBPin(10);
#endif

#if SUPPORT_TPiS_1T_1086_L5_5
constexpr unsigned int TPiS_I2CChannel = 1;
constexpr uint16_t TPiS_I2CAddress = 0x0C;
#endif

#if SUPPORT_LP5817
constexpr unsigned int LP5817_I2CChannel = 1;
constexpr uint16_t LP5817_I2CAddress = 0x2D * 2;
#endif

#if SUPPORT_AS5601
constexpr unsigned int AS5601_I2CChannel = 0;					//TODO which I2C?
constexpr uint16_t AS5601_I2CAddress = 0x36;					// I2C address of the AS5601
#endif

#if SUPPORT_INDUCTIVE_HEATER

// Definitions for inductive heater support

#define HEATER_POLL_RATE_MULTIPLIER		(10)					// how many times faster we run the temperature control loop than the standard 4Hz. Preferably a divisor of HeatSampleIntervalMillis.

constexpr unsigned int InductiveHeaterOscTccDeviceNumber = 3;	// number of the TC we use to generate the ~120kHz signal to excite the resonant circuit
constexpr unsigned int InductiveHeaterOscTccOutputNumber = 0;	// which output from the TCC we are using
constexpr unsigned int InductiveHeaterPwmTccDeviceNumber = 0;	// number of the TCC we use to generate the PWM signal that is gated with the osc signal
constexpr unsigned int InductiveHeaterPwmTccOutputNumber = 0;	// which output from the TCC we are using
constexpr unsigned int InductiveHeaterCCLNumber = 3;			// number of the CCL that we use to gate the TC and TCC output together
constexpr unsigned int InductiveHeaterCCLOutPin = PortBPin(17);	// the CCL output pin that drive the inductive heater mosfet
constexpr unsigned int InductiveHeaterAuxCCLNumber = 0;			// number of the second CCL that we need to use to gate two TCCs together

constexpr GpioPinFunction InductiveHeaterCCLOutPinPeriphMode = GpioPinFunction::N;

#endif

// Misc definitions
constexpr Pin UsbNotCanSelectPin = PortBPin(31);

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,	"out0.tach"		},	// PA00 fan 0 tacho
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out0"			},	// PA01 fan 0 out
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	SercomIo::none,		SercomIo::none,		Nx,	"ate.vin"		},	// PA02 VIN monitor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA03 board type
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA04 SPI0 MOSI (AS5047D, sercom0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA05 SPI0 SCK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA06 SPI0 CS0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA07 SPI0 MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA08 NP out (QSPI D0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"pa9"			},	// PA09 unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	nullptr			},	// PA10 ADC DRDY
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_11,	SercomIo::none,		SercomIo::none,		Nx,	"temp2,chambertemp" },	// PA11 on-board thermistor
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr 		},	// PA12 I2C1 SCL (sercom4)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA13 I2C1 SDA
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA14 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA15 crystal
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA16 SPI1 MOSI (ADC, sercom1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA17 SPI1 SCL
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA18 SPI1 CS
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PA19 SPI1 MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,	"out1.tach"		},	// PA20 fan1 tacho
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"out0"			},	// PA21 fan1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA22 I2C0 SCL (sercom3)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA23 I2C0 SDA
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA24 USB DN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA25 USB DP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,	nullptr			},	// PA27 accelerometer interrupt
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA30 swclk and LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PA31 swdio and LED1

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB00 SPI2_CS0 (sercom5)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB01 SPI2 MISO
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PB02 SPI2 MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, nullptr			},	// PB03 SPI2 SCK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB04 Driver ENN
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_7,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB05 Heater voltage feedback
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_8,	SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB06 Heater current
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,	nullptr			},	// PB07 Driver diag
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	SercomIo::none,		SercomIo::none,		Nx,	"temp0"			},	// PB08 case temperature thermistor
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	SercomIo::none,		SercomIo::none,		Nx,	"temp1"			},	// PB09 LDC coil temperature
	{ TcOutput::none,	TccOutput::tcc0_4F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,	nullptr			},	// PB10 LDC interrupt
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB11 LDC1612 clock (GCLK5)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB12 Driver step
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB13 Driver clock (GCLK7)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB14 CAN1 Tx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB15 CAN1 Rx
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB16 ADC clock (GCLK2)
	{ TcOutput::none,	TccOutput::tcc3_1F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB17 TCC for heater PWM direct, or CCL_OUT3
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB18 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB19 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB20 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB21 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,	"io0.in"		},	// PB22 endstop
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB23 driver DIR and BOOTLOADER_RESET jumper
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB24 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB25 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB27 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB29 not on chip
	{ TcOutput::none,	TccOutput::tcc4_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB30 Heater PWM if using external gating, else unused
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	nullptr			},	// PB31 USB/!CAN select

	// Virtual pins
#if SUPPORT_LIS3DH
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"i2c.lis3dh,i2c.lis2dw"	},	// LIS3DH sensor connected via I2C
#endif
#if SUPPORT_LDC1612
	{ TcOutput::none,	TccOutput::none,	AdcInput::ldc1612,	SercomIo::none,		SercomIo::none,		Nx,	"i2c.ldc1612"	},	// LDC1612 sensor connected via I2C
#endif
#if SUPPORT_AS5601
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,	"i2c.mfm"		},	// AS5601+TCA6408A filament monitor connected via I2C
#endif
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32 + 32;			// 32 pins on port A (some missing), 32 on port B (some missing)
constexpr size_t NumVirtualPins = SUPPORT_LIS3DH + SUPPORT_LDC1612 + SUPPORT_AS5601;

static_assert(NumPins == NumRealPins + NumVirtualPins);

#if SUPPORT_AS5601
constexpr Pin MfmPin = NumRealPins + SUPPORT_LIS3DH + SUPPORT_LDC1612;				// pin number when the user selects magnetic filament monitor on I2C bus
#endif

// Timer/counter used to generate step pulses and other sub-millisecond timings
constexpr unsigned int StepTcNumber = 0;
TcCount32 * const StepTc = &(TC0->COUNT32);
constexpr IRQn StepTcIRQn = TC0_IRQn;
#define STEP_TC_HANDLER			TC0_Handler

// Available UART ports
#define NUM_SERIAL_PORTS		0

// DMA channel assignments
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel DmacChanAdc0Rx = 2;
constexpr DmaChannel DmacChanAdc1Rx = 3;
constexpr DmaChannel DmacChanLedTx = 4;
//TODO add DMA channels for SPI and I2C as needed

constexpr unsigned int NumDmaChannelsUsed = 5;			// must be at least the number of channels used, may be larger. Max 12 on the SAME5x.

constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioAdcRx = 2;
constexpr DmaPriority DmacPrioLed = 1;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const NvicPriority NvicPriorityStep = 3;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityDmac = 3;				// priority for DMA complete interrupts
const NvicPriority NvicPriorityUart = 3;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityI2C = 3;
const NvicPriority NvicPriorityPins = 3;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityCan = 4;
const NvicPriority NvicPriorityAdc = 5;

#endif /* SRC_CONFIG_TOOLINDX_H_ */
