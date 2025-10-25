/*
 * MoveSegment.cpp
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 */

#include "MoveSegment.h"

// Static members

MoveSegment *MoveSegment::freeList = nullptr;
unsigned int MoveSegment::numCreated = 0;

// Allocate a MoveSegment, from the freelist if possible, else create a new one
MoveSegment *MoveSegment::Allocate(MoveSegment *p_next) noexcept
{
	const auto iflags = IrqSave();
	MoveSegment * ms = freeList;
	if (ms != nullptr)
	{
		freeList = ms->GetNext();
		IrqRestore(iflags);
		ms->nextAndFlags = reinterpret_cast<uint32_t>(p_next);
	}
	else
	{
		++numCreated;
		IrqRestore(iflags);
		ms = new MoveSegment(p_next);
	}
	return ms;
}

// Release a MoveSegment
void MoveSegment::ReleaseAll(MoveSegment *item) noexcept
{
	while (item != nullptr)
	{
		MoveSegment *itemToRelease = item;
		item = item->GetNext();
		Release(itemToRelease);
	}
}

void MoveSegment::DebugPrint() const noexcept
{
	debugPrintf("s=%" PRIu32 " t=%" PRIu32 " d=%.2f u=%.4e a=%.4e f=%02" PRIx32 "\n", startTime, duration, (double)distance, (double)CalcU(), (double)a, GetFlags().all);
}

/*static*/ void MoveSegment::DebugPrintList(const MoveSegment *segs) noexcept
{
	if (segs == nullptr)
	{
		debugPrintf("null seg\n");
	}
	else
	{
		while (segs != nullptr)
		{
			segs->DebugPrint();
			segs = segs->GetNext();
		}
	}
}

// End
