/*
 * CustomCommandHandler.cpp
 *
 *  Created on: 6 Jan 2025
 *      Author: David
 */

#include "CustomCommandHandler.h"

#include <CAN/CanInterface.h>
#include <CanMessageFormats.h>
#include <CanMessageGenericTables.h>
#include <CanMessageGenericParser.h>

GCodeResult CustomCommandHandler::ProcessM655(CanMessageGeneric& msg, const StringRef& reply)
{
	// This is an outline of how to process a custom command

	// 1. Create a parser to extract parameters from the message
	CanMessageGenericParser parser(msg, M655Params);

	// 2. Extract the expected parameters from the message and do any necessary checking.
	//    This example just checks for parameters that are present and echoes their values in the reply.
	reply.printf("Board %u doesn't handle custom command with parameters:", CanInterface::GetCanAddress());
	float fval;
	uint16_t uival;
	int32_t ival;
	String<StringLength50> strval;

	if (parser.GetStringParam('A', strval.GetRef()))
	{
		reply.catf(" A=%s", strval.c_str());
	}
	if (parser.GetStringParam('C', strval.GetRef()))
	{
		reply.catf(" C=%s", strval.c_str());
	}
	if (parser.GetFloatParam('E', fval))
	{
		reply.catf(" E=%.4g", (double)fval);
	}
	if (parser.GetFloatParam('F', fval))
	{
		reply.catf(" F=%.4g", (double)fval);
	}
	if (parser.GetUintParam('P', uival))
	{
		reply.catf(" P=%u", uival);
	}
	if (parser.GetIntParam('R', ival))
	{
		reply.catf(" R=%" PRIi32, ival);
	}
	if (parser.GetIntParam('S', ival))
	{
		reply.catf(" S=%" PRIi32, ival);
	}

	// 3. Do whatever was requested

	// 4. Return ok, warning, or error. If returning warning or error then 'reply' should be populated with a suitable message.
	return GCodeResult::warning;
}

// End
