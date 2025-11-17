/*
 * MT6835.cpp
 *
 *  Created on: Aug 24, 2025
 *      Author: Matt
 */

#include "MT6835.h"

#if SUPPORT_CLOSED_LOOP && SUPPORT_MT6835

#include <Hardware/IoPorts.h>
#include <ClosedLoop/ClosedLoop.h>

constexpr unsigned int MT6835ResolutionBits = 14;

// Operation Command Bits
constexpr uint8_t MT6835_OP_None = 0b00000000;
constexpr uint8_t MT6835_OP_Read = 0b00110000;			//Read one byte register
constexpr uint8_t MT6835_OP_Write = 0b01100000;		//Write one byte register
constexpr uint8_t MT6835_OP_Program = 0b11000000;		//Program EEPROM
constexpr uint8_t MT6835_OP_Auto_Zero = 0b01010000;	//Auto Set Zero
constexpr uint8_t MT6835_OP_Burst_Angle = 0b10100000;	//Burst read angle registers, address 0x003

constexpr uint16_t MT6835_CRC = 0b100000111;

constexpr uint16_t MT6835Write_ACK = 0x55;

constexpr uint16_t MT6835Reg_UserID = 0x01; 	//USER_ID[7:0], free byte for user

// Angle Data Registers
constexpr uint8_t MT6835Reg_Angle1 = 0x03;	//ANGLE[20:13]
constexpr uint8_t MT6835Reg_Angle2 = 0x04;	//ANGLE[12:5]
constexpr uint8_t MT6835Reg_Angle3 = 0x05;	//Bits 7-3: ANGLE[4:0], Bits 2-0: STATUS[2:0]
constexpr uint8_t MT6835Reg_Angle4 = 0x06;	//CRC[7:0], Redundant check of ANGLE and STATUS. Polynomial X^8 + X^2 + X + 1, MSB (ANGLE{20]) First
constexpr uint8_t MT6835Reg_Status = 0x05;

// Status bits Logic 1 for warning:
// Bit 0: Rotation Over Speed Warning
// Bit 1: Weak Magnetic Field Warning
// Bit 2: Under Voltage Warning

//Bits not in comments are MagnTek ONLY, DO NOT CHANGE
constexpr uint8_t MT6835Reg_ABZ_Res1 = 0x07;	//ABZ_RES[13:6]
constexpr uint8_t MT6835Reg_ABZ_Res2 = 0x08;	//Bits 7-2: ABZ_RES[5:0], Bit 1: ABZ_OFF, Bit 0: AB_SWAP

constexpr uint8_t MT6835Reg_Zero1 = 0x09;		//ZERO_POS[11:4]
constexpr uint8_t MT6835Reg_Zero2 = 0x0A;		//Bits 7-4: ZERO_POS[3:0], Bit 3: Z_EDGE, Bits 2-0: Z_PUL_WID[2:0]

constexpr uint8_t MT6835Reg_Opt_S0 = 0x0A;	//Bits 7-4: ZERO_POS[3:0], Bit 3: Z_EDGE, Bits 2-0: Z_PUL_WID[2:0]
constexpr uint8_t MT6835Reg_Opt_S1 = 0x0B;	//Bits 7-6: Z_PHASE[1:0], Bit 5: UVW_MUX, Bit 4: UVW_OFF, Bits 3-0: UVW_RES[3:0]
constexpr uint8_t MT6835Reg_Opt_S2 = 0x0C;	//Bit 5: NLC_EN, Bit 4: PWM_FQ, Bit 3: PWM_POL, Bits 2-0: PWM_SEL[2:0]
constexpr uint8_t MT6835Reg_Opt_S3 = 0x0D;	//Bit 3: ROT_DIR, Bits 2-0: HYST[2:0]
constexpr uint8_t MT6835Reg_Opt_S4 = 0x0E;	//Bit 7: GPIO_DS, Bits 6-4: AUTOCAL_FREQ[2:0]
constexpr uint8_t MT6835Reg_Opt_S5 = 0x11;	//Bits 2-0: BW[2:0]

// NLC Table, 192 bytes
constexpr uint16_t MT6835_NLC_Base = 0x013;

constexpr uint32_t MT6835ClockFrequency = 5000000;


// Convert nanoseconds to clock cycles
static inline constexpr uint32_t NanoSecondsToClocks(uint32_t ns) noexcept
{
	return ((SystemCoreClockFreq/1000000) * ns)/1000;
}

constexpr uint32_t Clocks300ns = NanoSecondsToClocks(300);
constexpr uint32_t ClocksHalfSclk = SystemCoreClockFreq/(2 * MT6835ClockFrequency);

MT6835::MT6835(uint32_t p_stepsPerRev, SharedSpiDevice& spiDev, Pin p_csPin) noexcept
	: SpiEncoder(spiDev, MT6835ClockFrequency, SpiMode::mode3, false, p_csPin),
	  AbsoluteRotaryEncoder(p_stepsPerRev, MT6835ResolutionBits)
{
}

// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
GCodeResult MT6835::Init(const StringRef& reply) noexcept
{
	// See if we can read sensible data from the encoder
	StatusRegister regs;
	if (GetDiagnosticRegisters(regs))
	{

		if ((regs.status & 0x07) == 0)
		{
			Enable();
			return GCodeResult::ok;
		}

		reply.copy("Encoder warning");
		if ((regs.status & 0x01) != 0)
		{
			reply.cat(": rotation over speed");
		}
		if ((regs.status & 0x02) != 0)
		{
			reply.cat(": magnet too weak");
			}
		if ((regs.status & 0x04) != 0)
		{
			reply.cat(": under voltage");
		}
		Enable();
		return GCodeResult::warning;

	}

	reply.copy("Failed to read encoder status register");
	return GCodeResult::error;
}

// Enable the encoder
void MT6835::Enable() noexcept
{
	IoPort::SetPinMode(csPin, OUTPUT_HIGH);
	ClosedLoop::EnableEncodersSpi();
}

// Disable the encoder
void MT6835::Disable() noexcept
{
	IoPort::SetPinMode(csPin, OUTPUT_HIGH);
	ClosedLoop::DisableEncodersSpi();
}

//---------------------------------------------------------------------------------------------------------------------------
// Return the current position as reported by the encoder.  Return true if error, false if success.
bool MT6835::GetRawReading() noexcept
{
	if (spi.Select(0))			// get the mutex and set the clock rate
	{
//		StatusRegister regs;
		uint8_t crc;
		uint8_t response[3];
		const bool ok = DoSpiTransaction(MT6835_OP_Read, MT6835Reg_Angle1, response[0])

				&& (DelayCycles(GetCurrentCycles(), Clocks300ns),
				DoSpiTransaction(MT6835_OP_Read, MT6835Reg_Angle2, response[1]))

				&& (DelayCycles(GetCurrentCycles(), Clocks300ns),
				DoSpiTransaction(MT6835_OP_Read, MT6835Reg_Angle3, response[2]))

				&& (DelayCycles(GetCurrentCycles(), Clocks300ns),
				DoSpiTransaction(MT6835_OP_Read, MT6835Reg_Angle4, crc));

		spi.Deselect();			// release the mutex8
		if (ok)
		{
//			regs.status = (response[2] & 0x07);

			uint32_t temp_rawReading = (((uint32_t)response[0] << 13 |(uint32_t)response[1] << 5 | ((uint32_t)response[2] >> 3)));

			rawReading = temp_rawReading >> 7;

			return false;
		}
	}

	return true;
}

// Get the encoder status register returning true if success, false if error
bool MT6835::GetDiagnosticRegisters(StatusRegister& regs) noexcept
{
	if (spi.Select(0))			// get the mutex and set the clock rate
	{
		uint8_t response[3];
		const bool ok = DoSpiTransaction(MT6835_OP_Read, MT6835Reg_Angle3, response[2]);

		spi.Deselect();			// release the mutex8
		if (ok)
		{
			regs.status = (response[2] & 0x07);
			return true;
		}
	}

	return false;
}

// Get diagnostic information and append it to a string
void MT6835::AppendDiagnostics(const StringRef &reply) noexcept
{
	reply.catf("Encoder reverse polarity: %s", (IsReversed()) ? "yes" : "no");
	reply.catf(", full rotations %" PRIi32, fullRotations);
	reply.catf(", last angle %" PRIu32, currentAngle);
	reply.catf(", minCorrection=%.1f, maxCorrection=%.1f", (double)minLUTCorrection, (double)maxLUTCorrection);
	StatusRegister regs;
	if (GetDiagnosticRegisters(regs))
	{
		if ((regs.status & 0x01) != 0)
		{
			reply.cat(", rotation over speed warning");
		}
		if ((regs.status & 0x02) != 0)
		{
			reply.cat(", magnet too weak");
		}
		if ((regs.status & 0x04) != 0)
		{
			reply.cat(", under voltage warning");
		}
	}
	else
	{
		reply.cat(", failed to read encoder status");
	}
}

// Append short form status to a string. If there is an error then the user can use M122 to get more details.
void MT6835::AppendStatus(const StringRef &reply) noexcept
{
	reply.lcatf("Magnetic encoder type MT6835, motor steps/rev %" PRIu32, stepsPerRev);
	StatusRegister regs;
	if (GetDiagnosticRegisters(regs))
	{
		reply.catf(", agc %u", regs.status & 0x07);
		if ((regs.status & 0x01) != 0)
		{
			reply.cat(", rotation over speed warning");
		}
		if ((regs.status & 0x02) != 0)
		{
			reply.cat(", magnet too weak");
		}
		if ((regs.status & 0x04) != 0)
		{
			reply.cat(", under voltage warning");
		}
	}
	else
	{
		reply.cat(", failed to read encoder status");
	}
}

// Perform an SPI transaction. Caller must get ownership of the SPI device first and release it afterwards, possibly after doing multiple transactions.
// Leave at least 300ns between multiple calls to this function.
bool MT6835::DoSpiTransaction(uint8_t command, uint8_t address, uint8_t &response) noexcept
{
	IoPort::WriteDigital(csPin, false);

	const uint8_t txBuffer[3] =
		{	(uint8_t)(command),
			(uint8_t)(address),
			0
		};

	uint8_t rxBuffer[3];

	const bool ok = spi.TransceivePacket(txBuffer, rxBuffer, 3);

	IoPort::WriteDigital(csPin, true);

	response = (rxBuffer[2]);

	return ok;
}

#endif

// End
