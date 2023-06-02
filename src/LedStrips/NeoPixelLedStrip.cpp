/*
 * NeoPixelLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/NeoPixelLedStrip.h>

#if SUPPORT_LED_STRIPS

#include <Movement/StepTimer.h>
#if SUPPORT_PIO_NEOPIXEL
NeoPixelLedStrip* NeoPixelLedStrip::activePIOStrip = nullptr;
WS2812* NeoPixelLedStrip::ws2812Device = nullptr;
#endif

NeoPixelLedStrip::NeoPixelLedStrip(bool p_isRGBW) noexcept
	: LocalLedStrip((p_isRGBW) ? LedStripType::NeoPixel_RGBW : LedStripType::NeoPixel_RGB, DefaultNeoPixelSpiClockFrequency),
	  isRGBW(p_isRGBW)
{
#if SUPPORT_PIO_NEOPIXEL
	// We currently support only a single PIO based strip, this can use any pin.
	if (activePIOStrip == nullptr)
	{
		// no strip currently using PIO, so we can use it
		activePIOStrip = this;
		useDma = true;
	}
	else
	{
		useDma = false;
	}
#endif
}

#if SUPPORT_PIO_NEOPIXEL
NeoPixelLedStrip::~NeoPixelLedStrip()
{
	if (activePIOStrip == this)
	{
		if (ws2812Device != nullptr)
		{
			delete ws2812Device;
			ws2812Device = nullptr;
		}
		activePIOStrip = nullptr;
	}	
}
#endif

GCodeResult NeoPixelLedStrip::Configure(CanMessageGenericParser& parser, const StringRef& reply, uint8_t& extra) noexcept
{
	bool seen = false;
	GCodeResult rslt = CommonConfigure(parser, reply, seen, extra);
	if (seen)
	{
#if SUPPORT_PIO_NEOPIXEL
	// We currently support only a single PIO based strip, this can use any pin.
	if (UsesDma())
	{
		if (ws2812Device != nullptr)
		{
			delete ws2812Device;
		}
		ws2812Device = new WS2812(port.GetPin(), isRGBW, DmaChanWS2812);
	}
#endif
		return rslt;
	}

	return CommonReportDetails(reply);
}

GCodeResult NeoPixelLedStrip::HandleM150(CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
#if SUPPORT_DMA_NEOPIXEL
	if (DmaInProgress())													// if we are sending something
	{
		return GCodeResult::notFinished;
	}
#endif

	if (needStartDelay && StepTimer::GetTimerTicks() - whenTransferFinished < MinNeoPixelResetTicks)
	{
		return GCodeResult::notFinished;									// give the NeoPixels time to reset
	}

	LedParams params;
	params.GetM150Params(parser);
	params.ApplyBrightness();

#if SUPPORT_DMA_NEOPIXEL
	if (UsesDma())
	{
		SpiSendNeoPixelData(params);
	}
	else
#elif SUPPORT_PIO_NEOPIXEL
	if (UsesDma())
	{
		PioSendNeoPixelData(params);
	}
	else
#endif
	{
		BitBangNeoPixelData(params);
	}
	return GCodeResult::ok;
}

// Return the number of buffer bytes we need per LED
size_t NeoPixelLedStrip::GetBytesPerLed() const noexcept
{
	const size_t bytesPerLed = (isRGBW) ? 4 : 3;
#if SUPPORT_PIO_NEOPIXEL
	// PIO strings always use 4 bytes per pixel
	return (useDma) ? 4 : bytesPerLed;
#else
	return (useDma) ? bytesPerLed * 4 : bytesPerLed;
#endif
}

#if SUPPORT_DMA_NEOPIXEL

// Encode one NeoPixel byte into the buffer.
// A 0 bit is encoded as 1000
// A 1 bit is encoded as 1110
// All encoding is MSB first
static void EncodeNeoPixelByte(uint8_t *p, uint8_t val) noexcept
{
	static constexpr uint8_t EncodedByte[4] = { 0b10001000, 0b10001110, 0b11101000, 0b11101110 };

# if USE_16BIT_SPI
	// Swap bytes for 16-bit DMA
	*p++ = EncodedByte[(val >> 4) & 3];
	*p++ = EncodedByte[val >> 6];
	*p++ = EncodedByte[val & 3];
	*p++ = EncodedByte[(val >> 2) & 3];
# else
	*p++ = EncodedByte[val >> 6];
	*p++ = EncodedByte[(val >> 4) & 3];
	*p++ = EncodedByte[(val >> 2) & 3];
	*p++ = EncodedByte[val & 3];
# endif
}

// Send data to NeoPixel LEDs by DMA to SPI
GCodeResult NeoPixelLedStrip::SpiSendNeoPixelData(const LedParams& params) noexcept
{
	const unsigned int bytesPerLed = (isRGBW) ? 16 : 12;
	unsigned int numLeds = params.numLeds;
	uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
	while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + chunkBufferSize)
	{
		EncodeNeoPixelByte(p, (uint8_t)params.green);
		p += 4;
		EncodeNeoPixelByte(p, (uint8_t)params.red);
		p += 4;
		EncodeNeoPixelByte(p, (uint8_t)params.blue);
		p += 4;
		if (isRGBW)
		{
			EncodeNeoPixelByte(p, (uint8_t)params.white);
			p += 4;
		}
		--numLeds;
		++numAlreadyInBuffer;
	}

	if (!params.following)
	{
		DmaSendChunkBuffer(bytesPerLed * numAlreadyInBuffer);		// send data by DMA to SPI
		numAlreadyInBuffer = 0;
		needStartDelay = true;
	}
	return GCodeResult::ok;
}
#elif SUPPORT_PIO_NEOPIXEL
// Send data to NeoPixel LEDs by DMA to SPI
GCodeResult NeoPixelLedStrip::PioSendNeoPixelData(const LedParams& params) noexcept
{
	const unsigned int bytesPerLed = 4;
	unsigned int numLeds = params.numLeds;
	uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
	while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + chunkBufferSize)
	{
		*p++ = isRGBW ? (uint8_t)params.white : 0;
		*p++ = (uint8_t)params.blue;
		*p++ = (uint8_t)params.red;
		*p++ = (uint8_t)params.green;
		--numLeds;
		++numAlreadyInBuffer;
	}
	if (!params.following)
	{
		if (ws2812Device != nullptr)
		{
			ws2812Device->SendData((uint32_t *)chunkBuffer, numAlreadyInBuffer);
		}
		numAlreadyInBuffer = 0;
		needStartDelay = true;
	}
	return GCodeResult::ok;

}
#endif

// Bit bang data to Neopixels
constexpr uint32_t NanosecondsToCycles(uint32_t ns) noexcept
{
	return (ns * (uint64_t)SystemCoreClockFreq)/1000000000u;
}

constexpr uint32_t T0H = NanosecondsToCycles(350);
constexpr uint32_t T0L = NanosecondsToCycles(850);
constexpr uint32_t T1H = NanosecondsToCycles(800);
constexpr uint32_t T1L = NanosecondsToCycles(475);

// Send data to NeoPixel LEDs by bit banging
#if RP2040
// When bit-banging Neopixels we can't afford to wait for instructions to be fetched from flash memory
[[gnu::optimize("03")]] __attribute__((section(".time_critical")))
#endif
GCodeResult NeoPixelLedStrip::BitBangNeoPixelData(const LedParams& params) noexcept
#endif
{
	const unsigned int bytesPerLed = (isRGBW) ? 4 : 3;
	unsigned int numLeds = params.numLeds;
	uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
	while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + chunkBufferSize)
	{
		*p++ = (uint8_t)params.green;
		*p++ = (uint8_t)params.red;
		*p++ = (uint8_t)params.blue;
		if (isRGBW)
		{
			*p++ = (uint8_t)params.white;
		}
		--numLeds;
		++numAlreadyInBuffer;
	}

	if (!params.following)
	{
		const uint8_t *q = chunkBuffer;
		uint32_t nextDelay = T0L;

		IrqDisable();
		uint32_t lastTransitionTime = GetCurrentCycles();
		while (q < p)
		{
			uint8_t c = *q++;
			for (unsigned int i = 0; i < 8; ++i)
			{
				// The high-level time is critical, the low-level time is not.
				// On the SAME5x the high-level time easily gets extended too much, so do as little work as possible during that time.
				lastTransitionTime = DelayCycles(lastTransitionTime, nextDelay);
				const uint32_t thisDelay = (c & 0x80) ? T1H : T0H;
				nextDelay = (c & 0x80) ? T1L : T0L;
				if (port.GetTotalInvert())
				{
					port.FastDigitalWriteLow();
					lastTransitionTime = DelayCycles(lastTransitionTime, thisDelay);
					port.FastDigitalWriteHigh();
				}
				else
				{
					port.FastDigitalWriteHigh();
					lastTransitionTime = DelayCycles(lastTransitionTime, thisDelay);
					port.FastDigitalWriteLow();
				}
				c <<= 1;
			}
		}
		IrqEnable();
		numAlreadyInBuffer = 0;
		whenTransferFinished = StepTimer::GetTimerTicks();
		needStartDelay = true;
	}
	return GCodeResult::ok;
}

#endif

// End
