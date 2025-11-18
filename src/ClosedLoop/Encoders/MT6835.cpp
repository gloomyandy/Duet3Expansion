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

// CRC lookup table for polynomial x^8 + x^2 + x + 1
// This is the same as the CRC table used by the TMC22xx driver
constexpr uint8_t Crc8Table[] =
{
	0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75,   0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
	0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69,   0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
	0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D,   0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
	0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51,   0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
	0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05,   0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
	0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19,   0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
	0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D,   0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
	0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21,   0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
	0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95,   0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
	0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,   0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
	0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD,   0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
	0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1,   0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
	0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5,   0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
	0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9,   0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
	0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD,   0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
	0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1,   0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF
};

// Convert nanoseconds to clock cycles
static inline constexpr uint32_t NanoSecondsToClocks(uint32_t ns) noexcept
{
	return ((SystemCoreClockFreq/1000000) * ns)/1000;
}

constexpr uint32_t Clocks300ns = NanoSecondsToClocks(300);

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

		reply.copy("Encoder warning(s)");
		if ((regs.status & 0x01) != 0)
		{
			reply.cat(": rotation overspeed");
		}
		if ((regs.status & 0x02) != 0)
		{
			reply.cat(": magnet too weak");
		}
		if ((regs.status & 0x04) != 0)
		{
			reply.cat(": undervoltage");
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
// This must set rawReading to a value between 0 and GetMaxValue()-1. Return true if successful, false if error.
bool MT6835::GetRawReading() noexcept
{
	if (spi.Select(0))			// get the mutex and set the clock rate
	{
		const uint8_t txBuffer[6] = { MT6835_OP_Read, MT6835Reg_Angle1, 0, 0, 0, 0 };		// read the 3 angle registers and the CRC in a single transaction
		uint8_t rxBuffer[6];
		IoPort::WriteDigital(csPin, false);
		const bool ok = spi.TransceivePacket(txBuffer, rxBuffer, 6);
		IoPort::WriteDigital(csPin, true);
		spi.Deselect();			// release the mutex

		if (ok)
		{
			// Check the CRC
			uint8_t crc = Crc8Table[rxBuffer[2]];					// I am assuming the CRC is initialised to 0, the datasheet doesn't say
			crc = Crc8Table[crc ^ rxBuffer[3]];
			crc = Crc8Table[crc ^ rxBuffer[4]];
			if (crc != rxBuffer[5])									// I am assuming the CRC byte is not reflected, the datasheet doesn't say
			{
				// Enable the following return when we have verified that we are using the correct CRC calculation; alternatively, add code to retry the transaction. For now just report in debug every 16384 times.
				// return false;
				static uint16_t counter = 0;
				if ((counter & (16384 - 1)) == 0)
				{
					debugPrintf("Encoder CRC error, data %02x %02x %02x, calc %02x, rec'd %02x\n", rxBuffer[2], rxBuffer[3], rxBuffer[4], crc, rxBuffer[5]);
				}
				++counter;
			}

			const uint32_t temp_rawReading = ((uint32_t)rxBuffer[2] << 13) | ((uint32_t)rxBuffer[3] << 5) | ((uint32_t)rxBuffer[4] >> 3);
			rawReading = temp_rawReading >> 7;
			// regs.status = (response[5] & 0x07);					// if we want to save the status, this is how to extract it
			return true;
		}
	}

	return false;
}

// Get the encoder status register returning true if success, false if error
bool MT6835::GetDiagnosticRegisters(StatusRegister& regs) noexcept
{
	if (spi.Select(0))			// get the mutex and set the clock rate
	{
		uint8_t response;
		const bool ok = DoSpiTransaction(MT6835_OP_Read, MT6835Reg_Angle3, response);
		spi.Deselect();			// release the mutex8

		if (ok)
		{
			regs.status = (response & 0x07);
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
			reply.cat(", rotation overspeed warning");
		}
		if ((regs.status & 0x02) != 0)
		{
			reply.cat(", magnet too weak");
		}
		if ((regs.status & 0x04) != 0)
		{
			reply.cat(", undervoltage warning");
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
		if ((regs.status & 0x01) != 0)
		{
			reply.cat(", rotation overspeed warning");
		}
		if ((regs.status & 0x02) != 0)
		{
			reply.cat(", magnet too weak");
		}
		if ((regs.status & 0x04) != 0)
		{
			reply.cat(", undervoltage warning");
		}
	}
	else
	{
		reply.cat(", failed to read encoder status");
	}
}

// Perform an SPI transaction to read a single 8-bit register. Caller must get ownership of the SPI device first and release it afterwards.
// If making multiple calls to this function within a single select operation, toggle CS high for 300ns and then low again between calls.
bool MT6835::DoSpiTransaction(uint8_t command, uint8_t address, uint8_t &response) noexcept
{
	const uint8_t txBuffer[3] = { command, address, 0 };
	uint8_t rxBuffer[3];

	const bool ok = spi.TransceivePacket(txBuffer, rxBuffer, 3);
	response = (rxBuffer[2]);
	return ok;
}

#endif

// End
