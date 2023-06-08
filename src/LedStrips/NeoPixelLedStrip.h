/*
 * NeoPixelLedStrip.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_NEOPIXELLEDSTRIP_H_
#define SRC_LEDSTRIPS_NEOPIXELLEDSTRIP_H_

#include "LocalLedStrip.h"
#include <Movement/StepTimer.h>

#if SUPPORT_LED_STRIPS
#if SUPPORT_PIO_NEOPIXEL
#include<WS2812.h>
#endif

#if SUPPORT_PIO_NEOPIXEL
# include <WS2812.h>
#endif

class NeoPixelLedStrip : public LocalLedStrip
{
public:
	NeoPixelLedStrip(bool p_isRGBW) noexcept;

	GCodeResult Configure(CanMessageGenericParser& parser, const StringRef& reply, uint8_t& extra) noexcept override;
	GCodeResult HandleM150(CanMessageGenericParser& parser, const StringRef& reply) noexcept override;

protected:
	size_t GetBytesPerLed() const noexcept override;

private:
	static constexpr uint32_t DefaultNeoPixelSpiClockFrequency = 2500000;				// must be between about 2MHz and about 4MHz
	static constexpr uint32_t MinNeoPixelResetTicks = (250 * StepTimer::StepClockRate)/1000000;	// 250us minimum Neopixel reset time on later chips

	GCodeResult BitBangNeoPixelData(const LedParams& params) noexcept;
#if SUPPORT_DMA_NEOPIXEL
	GCodeResult SpiSendNeoPixelData(const LedParams& params) noexcept;
#elif SUPPORT_PIO_NEOPIXEL
	GCodeResult PioSendNeoPixelData(const LedParams& params) noexcept;
	static WS2812* ws2812Device;
#endif
	unsigned int numAlreadyInBuffer = 0;												// number of pixels already store in the buffer
	bool isRGBW;
	bool needStartDelay = true;
};

#endif

#endif /* SRC_LEDSTRIPS_NEOPIXELLEDSTRIP_H_ */
