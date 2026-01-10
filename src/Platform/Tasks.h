/*
 * Startup.h
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#ifndef SRC_TASKS_H_
#define SRC_TASKS_H_

#include "RepRapFirmware.h"
#include <RTOSIface/RTOSIface.h>
#include <new>			// for std::align_val_t

[[noreturn]] void AppMain() noexcept;

namespace Tasks
{
	ptrdiff_t GetNeverUsedRam() noexcept;
	void *AllocPermanent(size_t sz, std::align_val_t align = (std::align_val_t)
#if SAME70
		__STDCPP_DEFAULT_NEW_ALIGNMENT__
#else
		// gcc defines __STDCPP_DEFAULT_NEW_ALIGNMENT__ as 8, which is wasteful of memory on ARM Cortex M4 and M0+ processors
		sizeof(float)
#endif
		) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;
	uint32_t DoDivide(uint32_t a, uint32_t b) noexcept;
	void *DoMemoryLeak() noexcept;
	uint32_t DoMemoryRead(const uint32_t* addr) noexcept;
	void *GetNVMBuffer(const uint32_t *_ecv_array null stk) noexcept;

	Module GetSpinningModule() noexcept;

	static constexpr uint32_t MaxHeatTaskTicksInSpinState = 4000;	// timeout before we reset the processor if the heat task doesn't run
	static constexpr uint32_t MaxMainTaskTicksInSpinState = 60000;	// timeout before we reset the processor if the main task doesn't run
}

// Functions called by CanMessageBuffer in CANlib
void *MessageBufferAlloc(size_t sz, std::align_val_t align) noexcept;
void MessageBufferDelete(void *ptr, std::align_val_t align) noexcept;

#endif /* SRC_TASKS_H_ */
