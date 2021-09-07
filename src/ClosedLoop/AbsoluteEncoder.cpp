/*
 * AbsoluteEncoder.cpp
 *
 *  Created on: 6 Sep 2021
 *      Author: Louis
 */

#ifdef SUPPORT_CLOSED_LOOP
# if SUPPORT_CLOSED_LOOP

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
int32_t AbsoluteEncoder<MAX, LUT_RESOLUTION>::GetReading() noexcept {

	bool error;
	int32_t currentAngle = GetAbsolutePosition(error);

	if (error) {
		//TODO how to report an error?
		return fullRotations * MAX + lastAngle;
	}

	// Apply LUT correction (if the LUT is loaded)
	// (These divisions should be efficient because LUT_RESOLUTION is a power of 2)
	if (LUTLoaded) {
		int windowStartIndex = currentAngle / LUT_RESOLUTION;
		float windowStart = correctionLUT[windowStartIndex];
		int windowOffset = currentAngle % LUT_RESOLUTION;

		// Handle the zero-crossing
		if (windowStartIndex == zeroCrossingIndex && zeroCrossingOffset >= windowOffset) {
			windowStart = 0;
			windowOffset -= zeroCrossingOffset;
		}

		currentAngle = windowStart + windowOffset;
	}

	// Accumulate the full rotations if one has occurred
	int32_t difference = currentAngle - lastAngle;
	if (abs(difference) > (int32_t)MAX/2) {
		fullRotations += (difference < 0) - (difference > 0);	// Add -1 if diff > 0, +1 if diff < 0
	}
	lastAngle = currentAngle;

	// Return the position plus the accumulated rotations
	return fullRotations * MAX + lastAngle;
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
void AbsoluteEncoder<MAX, LUT_RESOLUTION>::LoadLUT() noexcept {
	// TODO: Read back LUT from NVRAM (fourier transform -> array)

	// Find the zero-crossing index and offset
	float prevVal = correctionLUT[0];
	for (unsigned int i = 1; i<(MAX/LUT_RESOLUTION); i++) {
		float curVal = correctionLUT[i];
		if (abs(prevVal - curVal) > MAX/2) {
			zeroCrossingIndex = i-1;
			zeroCrossingOffset = round(MAX - prevVal);
			break;
		}
		prevVal = curVal;
	}

	// Mark the LUT as loaded
	LUTLoaded = true;
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
void AbsoluteEncoder<MAX, LUT_RESOLUTION>::StoreLUT() noexcept {
	// TODO: Verify all LUT values are present

	// TODO: Store LUT to NVRAM (as fourier transform)

	// Read back the LUT (Ensures that the fourier transformed version is used)
	LoadLUT();
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
void AbsoluteEncoder<MAX, LUT_RESOLUTION>::ClearLUT() noexcept {
	LUTLoaded = false;
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
void AbsoluteEncoder<MAX, LUT_RESOLUTION>::ScrubLUT() noexcept {
	// TODO: Remove LUT from NVRAM
}

template<unsigned int MAX, unsigned int LUT_RESOLUTION>
void AbsoluteEncoder<MAX, LUT_RESOLUTION>::StoreLUTValueForPosition(int16_t encoder_reading, float real_world_position) noexcept {
	if (real_world_position < 0) {real_world_position = MAX + real_world_position;}
	correctionLUT[encoder_reading / LUT_RESOLUTION] = real_world_position;
}

# endif
#endif
