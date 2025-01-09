/*
 * CustomCommandHandler.h
 *
 *  Created on: 6 Jan 2025
 *      Author: David
 */

#ifndef SRC_COMMANDPROCESSING_CUSTOMCOMMANDHANDLER_H_
#define SRC_COMMANDPROCESSING_CUSTOMCOMMANDHANDLER_H_

#include <RepRapFirmware.h>

struct CanMessageGeneric;

namespace CustomCommandHandler
{
	GCodeResult ProcessM655(CanMessageGeneric& msg, const StringRef& reply);
}

#endif /* SRC_COMMANDPROCESSING_CUSTOMCOMMANDHANDLER_H_ */
