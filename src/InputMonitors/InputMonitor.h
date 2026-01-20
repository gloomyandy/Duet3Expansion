/*
 * InputMonitor.h
 *
 *  Created on: 17 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_INPUTMONITOR_H_
#define SRC_ENDSTOPS_INPUTMONITOR_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>
#include <Hardware/IoPorts.h>
#include <RTOSIface/RTOSIface.h>
#include <General/FreelistManager.h>

struct CanMessageCreateInputMonitorV1;
struct CanMessageChangeInputMonitorV1;
struct CanMessageInputChangedV1;
class CanMessageBuffer;

class InputMonitor
{
public:
	DECLARE_FREELIST_NEW_DELETE(InputMonitor)

	InputMonitor() noexcept { }
	~InputMonitor();

#if SUPPORT_AS5601
	void UpdateState(bool newState) noexcept;
#endif

#if SUPPORT_LDC1612
	void SetTriggered() noexcept;
	void AnalogInterrupt(int32_t reading) noexcept;			// this is public if LKDC1612 support is configured
#endif

	static void Init() noexcept;
	static void Spin() noexcept;

	static GCodeResult Create(const CanMessageCreateInputMonitorV1& msg, size_t dataLength, const StringRef& reply, uint8_t& extra) noexcept;
	static GCodeResult Change(const CanMessageChangeInputMonitorV1& msg, const StringRef& reply, uint8_t& extra) noexcept;

	static uint32_t AddStateChanges(CanMessageInputChangedV1 *msg) noexcept;
	static void ReadInputs(CanMessageBuffer *buf) noexcept;

	static unsigned int AddAnalogHandleDataV0(uint8_t *buffer, size_t spaceLeft) noexcept;

	static void CommonDigitalPortInterrupt(CallbackParameter cbp) noexcept;
	static void CommonAnalogPortInterrupt(CallbackParameter cbp, int32_t reading) noexcept;

private:
	bool IsDigital() const noexcept { return threshold == 0; }
	bool Activate() noexcept;
	void Deactivate() noexcept;
	void DigitalInterrupt() noexcept;
	int32_t GetAnalogValue() const noexcept;

#if SUPPORT_LDC1612
	GCodeResult SetDriveLevel(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept;
	GCodeResult SelectTouchMode(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept;
#else
	void AnalogInterrupt(uint32_t reading) noexcept;			// this is private unless LKDC1612 support is configured
#endif

	static bool Delete(uint16_t hndl) noexcept;
	static ReadLockedPointer<InputMonitor> Find(uint16_t hndl) noexcept;

	InputMonitor *next;
	IoPort port;
	uint32_t whenLastSent;
	int32_t threshold;
	uint16_t handle;
	uint16_t minInterval;
	bool active;
	volatile bool state;
	volatile bool sendDue;
#if SUPPORT_LDC1612
	bool isLdcInTouchMode;
#endif

	static InputMonitor * volatile monitorsList;
	static ReadWriteLock listLock;
};

#endif /* SRC_ENDSTOPS_INPUTMONITOR_H_ */
