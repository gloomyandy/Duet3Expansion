/*
 * MT6835.h
 *
 *  Created on: Aug 24, 2025
 *      Author: Matt
 */

#ifndef SRC_CLOSEDLOOP_MT6835_H_
#define SRC_CLOSEDLOOP_MT6835_H_

#include "AbsoluteRotaryEncoder.h"

#if SUPPORT_CLOSED_LOOP && SUPPORT_MT6835

#include "SpiEncoder.h"
#include <General/FreelistManager.h>

class MT6835 : public SpiEncoder, public AbsoluteRotaryEncoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<MT6835>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<MT6835>(p); }

	MT6835(uint32_t p_stepsPerRev, SharedSpiDevice& spiDev, Pin p_csPin) noexcept;
	~MT6835() { MT6835::Disable(); }

	EncoderType GetType() const noexcept override { return EncoderType::rotaryMagnetic; }
	GCodeResult Init(const StringRef& reply) noexcept override;
	void Enable() noexcept override;
	void Disable() noexcept override;
	void AppendDiagnostics(const StringRef& reply) noexcept override;
	void AppendStatus(const StringRef& reply) noexcept override;

protected:
	bool GetRawReading() noexcept override;


private:
	struct StatusRegister
	{
		uint8_t status;
	};

	bool DoSpiTransaction(uint8_t command, uint8_t address, uint8_t& response) noexcept;
	bool GetDiagnosticRegisters(StatusRegister& regs) noexcept;
	uint16_t MaskInput(uint32_t rawInput) noexcept;
};

#endif


#endif /* SRC_CLOSEDLOOP_MT6835_H_ */
