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

#ifndef SRC_CONFIG_RP2350TEST_H_
#define SRC_CONFIG_RP2350TEST_H_
# include <hardware/timer.h>

#include <Hardware/PinDescription.h>
#include <SPI/SpiParameters.h>
#include <I2C/I2cParameters.h>
#define BOARD_TYPE_NAME		"RP2350TEST"
#define BOOTLOADER_NAME		"RP2350TEST"

#define BOARD_USES_UF2_BINARY	1

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0
#define HAS_USB_SERIAL			1
#define USE_SERIAL_DEBUG		1
#define SUPPORT_LED_STRIPS		1
#define SUPPORT_PIO_NEOPIXEL	1
#define SUPPORT_INPUT_SHAPING	1
#define NUM_SHARED_SPI			3

// Drivers configuration
#define SUPPORT_DRIVERS			1

#if SUPPORT_DRIVERS

#if 1 // TMC2240 using SPI driver
#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define TMCSPI_USES_SEPARATE_CS		0
#define TMCSPI_USES_SEPARATE_ENABLE	0
#define TMC_USES_SHARED_SPI	1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2240			0
#define SUPPORT_TMC2240_SPI		1
#define SUPPORT_INPUT_SHAPING	1


constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;

// TMC2240 current sense resistor and scaling
constexpr uint32_t Tmc2240CurrentRange = 0x03;								// which current range we set the TMC2240 to (3A)
constexpr uint32_t Tmc2240SlopeControl = 0x01;								// which slope control we set the TMC2240 to (200V/us)
constexpr float Tmc2240Rref = 12.0;											// TMC2240 reference resistor in Kohms
constexpr float DriverFullScaleCurrent = 24000/Tmc2240Rref;					// in mA, assuming we set the range bits in the DRV_CONF register to 0x01
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;			// with RRef = 15K this works out as 1.6A so this is the maximum current we can ask for

constexpr float MaxMotorCurrent = DriverFullScaleCurrent;

constexpr uint32_t DefaultStandstillCurrentPercent = 75;

#if TMCSPI_USES_SEPARATE_ENABLE
constexpr Pin Tmc51xxEnablePins[] = {GpioPin(14)};
#else
constexpr Pin GlobalTmcEnablePin = GpioPin(14);
#endif
#if TMCSPI_USES_SEPARATE_CS
constexpr Pin Tmc51xxCSPins[] = {GpioPin(15)};
#else
constexpr Pin GlobalTmcCSPin = GpioPin(15);
#endif
#if TMC_USES_SHARED_SPI
constexpr unsigned int Tmc5160_SpiChannel = 2;
#endif

constexpr Pin DirectionPins[NumDrivers] = { GpioPin(6) };
constexpr Pin StepPins[NumDrivers] = { GpioPin(7) };


#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low
#elif 0 // Standard TMC5160 driver
#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define TMCSPI_USES_SEPARATE_CS		0
#define TMCSPI_USES_SEPARATE_ENABLE	0
#define TMC_USES_SHARED_SPI	1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0

#define SUPPORT_TMC51xx			1
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2240			0
#define SUPPORT_TMC2240_SPI		0
#define SUPPORT_INPUT_SHAPING	1


constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;
constexpr float MaxMotorCurrent = 2500.0;
constexpr uint32_t DefaultStandstillCurrentPercent = 71;
constexpr float Tmc5160SenseResistor = 0.075;

#if TMCSPI_USES_SEPARATE_ENABLE
constexpr Pin Tmc51xxEnablePins[] = {GpioPin(14)};
#else
constexpr Pin GlobalTmcEnablePin = GpioPin(14);
#endif
#if TMCSPI_USES_SEPARATE_CS
constexpr Pin Tmc51xxCSPins[] = {GpioPin(15)};
#else
constexpr Pin GlobalTmcCSPin = GpioPin(15);
#endif
#if TMC_USES_SHARED_SPI
constexpr unsigned int Tmc5160_SpiChannel = 0;
#endif

constexpr Pin DirectionPins[NumDrivers] = { GpioPin(6) };
constexpr Pin StepPins[NumDrivers] = { GpioPin(7) };


#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#else // TMC2209 driver
#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			1
#define SUPPORT_TMC2240			0
#define SUPPORT_TMC2240_SPI		0

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
constexpr float MaxMotorCurrent = 1600.0;
constexpr float MaximumStandstillCurrent = 1200.0;
constexpr uint32_t DefaultStandstillCurrentPercent = 75;

constexpr Pin GlobalTmcEnablePin = GpioPin(14);
constexpr Pin Tmc22xxUartPin = GpioPin(15);

constexpr Pin StepPins[NumDrivers] = { GpioPin(7) };
constexpr Pin DirectionPins[NumDrivers] = { GpioPin(6) };
#if HAS_STALL_DETECT
constexpr Pin DriverDiagPins[NumDrivers] = { GpioPin(27) };
#endif

#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low
#endif
#endif

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		0
#define NUM_I2C_CHANNELS		0
#define SUPPORT_LIS3DH			0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_LDC1612			0

#define USE_MPU					0
#define USE_CACHE				0

#define PIN_TODO	GpioPin(NoPin)	//TEMPORARY! Used when we haven't assigned a pin yet.

constexpr unsigned int CANInstanceNumber = 0;
constexpr bool UseLaterCanPins = false;


constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 2;
constexpr float DefaultThermistorSeriesR = 4700.0;		// TEMP0 has 1K or 4K7 pullup, chamber thermistor has 4K7
constexpr Pin TempSensePins[NumThermistorInputs] = { GpioPin(40), GpioPin(41) };
constexpr Pin CanTxPin = GpioPin(0);
constexpr Pin CanRxPin = GpioPin(1);

constexpr Pin ButtonPins[] = { PIN_TODO };

// VIN voltage monitor
#if HAS_VOLTAGE_MONITOR
constexpr float VinDividerRatio = (47.0 + 4.7)/4.7;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;				// the Pico uses the 3.3V supply as the voltage reference
#endif

// Diagnostic LEDs
constexpr Pin LedPins[] = { GpioPin(5) };
constexpr bool LedActiveHigh = false;

#if NUM_SHARED_SPI > 0
// Shared SPI pin connections
#if 0
constexpr SpiParameters SharedSpiParams[NUM_SHARED_SPI] = {
{
	.instanceNumber = 0,
	.mosiPin = 3,
	.misoPin = 4,
	.sclkPin = 2,
},
{
	.instanceNumber = 1,
	.mosiPin = 11,
	.misoPin = 8,
	.sclkPin = 10,
}
};
#endif
constexpr SpiParameters SharedSpiParams[NUM_SHARED_SPI] = {
{
	.instanceNumber = 0,
	.mosiPin = NoPin,
	.misoPin = NoPin,
	.sclkPin = NoPin,
},
{
	.instanceNumber = 1,
	.mosiPin = 11,
	.misoPin = 8,
	.sclkPin = 10,
},
{
	.instanceNumber = 0,
	.mosiPin = 3,
	.misoPin = 4,
	.sclkPin = 2,
}
};

#if SUPPORT_CAN && USE_SPICAN

constexpr uint8_t spiCan_SpiChannel = 1;
constexpr Pin SPICanCsPin = GpioPin(9);

#endif

#endif

#if NUM_I2C_CHANNELS != 0

// I2C using pins 18,19
constexpr I2cParameters I2C0Params =
{
	.instanceNumber = 1,
	.sclPin = 19,
	.sdaPin = 18,
};
#endif

#if SUPPORT_LIS3DH

#define ACCELEROMETER_USES_SPI			(1)					// 0 if the accelerometer is connected via I2C, 1 if via SPI
constexpr unsigned int Lis_SpiChannel = 0;
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
	{ PwmOutput::pwm0b,	AdcInput::none,		"spi.cs.rtd,max31865cs,rtdcs" },	// GPIO17 MAX31865_CS
	{ PwmOutput::pwm1a,	AdcInput::none,		nullptr 	},	// GPIO18 I2C_SDA
	{ PwmOutput::pwm1b,	AdcInput::none,		nullptr		},	// GPIO19 I2C_SDL
	{ PwmOutput::pwm2a,	AdcInput::none,		"io1.in"	},	// GPIO20 ENDSTOP1
	{ PwmOutput::pwm2b,	AdcInput::none,		"out2"		},	// GPIO21 FAN1
	{ PwmOutput::pwm3a,	AdcInput::none,		"io0.in"	},	// GPIO22 PROBE
	{ PwmOutput::pwm3b,	AdcInput::none,		"out0"		},	// GPIO23 HEAT0
	{ PwmOutput::pwm4a,	AdcInput::none,		"io0.out"	},	// GPIO24 SERVO
	{ PwmOutput::pwm4b,	AdcInput::none,		nullptr		},	// GPIO25 ACC_INT1
	{ PwmOutput::pwm5a,	AdcInput::none,		"rgbled"	},	// GPIO26 RGB
	{ PwmOutput::pwm5b,	AdcInput::none,		"diag0"		},	// GPIO27 DIAG0
	{ PwmOutput::pwm6a,	AdcInput::none,		nullptr		},	// GPIO28 NC
	{ PwmOutput::pwm6b,	AdcInput::none,		"intb"		},	// GPIO29 ldc1612 int pin
	{ PwmOutput::pwm7a,	AdcInput::none,		"out2"		},	// GPIO30 FAN PWM0
	{ PwmOutput::pwm7b,	AdcInput::none,		nullptr		},	// GPIO31 NC
	{ PwmOutput::pwm8a,	AdcInput::none,		nullptr		},	// GPIO32 NC
	{ PwmOutput::pwm8b,	AdcInput::none,		"out3"		},	// GPIO33 FAN PWM1
	{ PwmOutput::pwm9a,	AdcInput::none,		nullptr		},	// GPIO34 NC
	{ PwmOutput::pwm9b,	AdcInput::none,		nullptr		},	// GPIO35 NC
	{ PwmOutput::pwm10a,AdcInput::none,		nullptr		},	// GPIO36 NC
	{ PwmOutput::pwm10b,AdcInput::none,		nullptr		},	// GPIO37 NC
	{ PwmOutput::pwm11a,AdcInput::none,		nullptr		},	// GPIO38 NC
	{ PwmOutput::pwm11b,AdcInput::none,		nullptr		},	// GPIO39 NC
	{ PwmOutput::pwm8a,	AdcInput::adc0_0,	"temp1"		},	// GPIO40 CHAMBER TEMP
	{ PwmOutput::pwm8b,	AdcInput::adc0_1,	"temp0"		},	// GPIO41 TEMP0
	{ PwmOutput::pwm9a,	AdcInput::adc0_2,	nullptr		},	// GPIO42 Power ADC
	{ PwmOutput::pwm9b,	AdcInput::adc0_3,	nullptr		},	// GPIO43 NC
	{ PwmOutput::pwm10a,AdcInput::adc0_4,	nullptr		},	// GPIO44 NC
	{ PwmOutput::pwm10b,AdcInput::adc0_5,	nullptr		},	// GPIO45 NC
	{ PwmOutput::pwm11a,AdcInput::adc0_6,	nullptr		},	// GPIO46 NC
	{ PwmOutput::pwm11b,AdcInput::adc0_7,	nullptr		},	// GPIO47 NC
	// Virtual pins
#if SUPPORT_LDC1612
	{ PwmOutput::none,	AdcInput::ldc1612,	"i2c.ldc1612"},	// LDC1612 sensor connected via I2C
#endif
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
#if RP2040
constexpr size_t NumRealPins = 30;				// 30 GPIO pins on RP2040
#elif RP2350
constexpr size_t NumRealPins = 48;				// 48 GPIO pins on RP2350
#endif
constexpr size_t NumVirtualPins = SUPPORT_LDC1612;

static_assert(NumPins == NumRealPins + NumVirtualPins);

// Timer/counter used to generate step pulses and other sub-millisecond timings
constexpr unsigned int StepTimerAlarmNumber = 0;
#if defined(__RP2040__)
constexpr unsigned int StepTcIRQn = TIMER_IRQ_0;
#else
// we should be able to use TIMER_ALARM_IRQ_NUM here but it generates an error when using it with constexpr
constexpr unsigned int StepTcIRQn = TIMER0_IRQ_0;
#endif

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

#endif /* SRC_CONFIG_RP2350TEST_H_ */
