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
	bool GetAngleAndStatus(uint32_t& angle, uint8_t& status) noexcept;				// Get both the raw angle and the status, with CRC check
	bool AppendEncoderStatus(const StringRef& reply, uint32_t& angle) noexcept;		// Read the angle and status registers and append any status warnings to 'reply'
};

#endif


#endif /* SRC_CLOSEDLOOP_MT6835_H_ */
