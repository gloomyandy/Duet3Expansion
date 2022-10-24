/*
 * RelativeEncoder.h
 *
 *  Created on: 6 Sep 2021
 *      Author: Louis
 */

/*
 * To use the RelativeEncoder class, define:
 *
 *  - int32_t GetRelativePosition() to return a value representative of the distance moved relative to the start position
 *
 *  - Plus all the virtual functions required by the Encoder class
 */

#ifndef SRC_CLOSEDLOOP_RELATIVEENCODER_H_
# define SRC_CLOSEDLOOP_RELATIVEENCODER_H_

#include "Encoder.h"

#if SUPPORT_CLOSED_LOOP

class RelativeEncoder : public Encoder
{
public:
	// Constructors
	RelativeEncoder(uint32_t p_stepsPerRev, uint32_t p_countsPerRev) noexcept : Encoder(p_stepsPerRev, (float)p_countsPerRev/(float)p_stepsPerRev) {}

	// Overridden virtual functions

	// Return true if this is an absolute encoder
	bool IsAbsolute() const noexcept override { return false; }

	// Get the current reading
	bool TakeReading() noexcept override;

	// Tell the encoder what the step phase is at a particular count
	void SetKnownPhaseAtCount(uint32_t phase, int32_t count) noexcept override;

	// Clear the accumulated full rotations so as to get the count back to a smaller number
	void ClearFullRevs() noexcept override { currentCount %= (int32_t)countsPerRev; }

protected:
	// Get the relative position since the start
	virtual int32_t GetRelativePosition(bool& error) noexcept = 0;

private:
	uint32_t countsPerRev;
};

#endif

#endif /* SRC_CLOSEDLOOP_RELATIVEENCODER_H_ */
