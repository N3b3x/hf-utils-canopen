/**
 * @file CanOpenUtils.h
 * @brief Helper utilities for creating CANopen frames.
 */

#ifndef UTILITIES_CANOPEN_CANOPENUTILS_H_
#define UTILITIES_CANOPEN_CANOPENUTILS_H_

#include <cstdint>
#include "CanFrame.h"

namespace CanOpen {

/**
 * @brief NMT command specifiers.
 */
enum class NmtCommand : uint8_t {
    StartNode          = 0x01,
    StopNode           = 0x02,
    EnterPreOperational = 0x80,
    ResetNode          = 0x81,
    ResetCommunication = 0x82
};

/**
 * @brief Build a CAN frame containing an NMT command.
 * @param nodeId Node ID of the target
 * @param cmd Command specifier
 * @return Populated CAN frame
 */
CanFrame BuildNmt(uint8_t nodeId, NmtCommand cmd) noexcept;

/**
 * @brief Build an expedited SDO download frame.
 *
 * This helper only supports 1, 2 or 4 byte writes.
 *
 * @param nodeId Node ID of the target device
 * @param index Object dictionary index
 * @param subIndex Sub-index
 * @param data Value to write
 * @param size Size of the data in bytes (1,2,4)
 * @return Populated CAN frame
 */
CanFrame BuildSdoDownload(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                          uint32_t data, uint8_t size) noexcept;

} // namespace CanOpen

#endif // UTILITIES_CANOPEN_CANOPENUTILS_H_

