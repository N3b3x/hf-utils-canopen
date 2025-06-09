#ifndef UTILITIES_CANOPEN_CANOPENMOTORUTILS_H_
#define UTILITIES_CANOPEN_CANOPENMOTORUTILS_H_

#include <cstdint>
#include "CanOpenUtils.h"

namespace CanOpen {

/**
 * @brief Operation modes for drive control.
 */
enum class OperationMode : int8_t {
    ProfilePosition = 1, ///< Profile position mode
    ProfileVelocity = 3, ///< Profile velocity mode
    Homing          = 6, ///< Homing mode
    CyclicPosition  = 8, ///< Cyclic synchronous position mode
    CyclicVelocity  = 9  ///< Cyclic synchronous velocity mode
};

/**
 * @brief Build a SDO upload request frame.
 * @param nodeId Node ID of the target device
 * @param index Object dictionary index
 * @param subIndex Sub-index
 * @return Populated CAN frame
 */
FlexCan::Frame BuildSdoUpload(uint8_t nodeId, uint16_t index,
                              uint8_t subIndex) noexcept;

/**
 * @brief Build a heartbeat frame.
 * @param nodeId Node ID of the device
 * @param state Node state encoded in the heartbeat
 * @return Populated CAN frame
 */
FlexCan::Frame BuildHeartbeat(uint8_t nodeId, uint8_t state) noexcept;

/**
 * @brief Build a control word download frame (index 0x6040).
 * @param nodeId Node ID of the device
 * @param controlWord Control word value
 * @return Populated CAN frame
 */
FlexCan::Frame BuildControlWord(uint8_t nodeId, uint16_t controlWord) noexcept;

/**
 * @brief Build a mode of operation download frame (index 0x6060).
 * @param nodeId Node ID of the device
 * @param mode Desired operation mode
 * @return Populated CAN frame
 */
FlexCan::Frame BuildModeOfOperation(uint8_t nodeId, OperationMode mode) noexcept;

/**
 * @brief Build a target velocity download frame (index 0x60FF).
 * @param nodeId Node ID of the device
 * @param velocity Target velocity
 * @return Populated CAN frame
 */
FlexCan::Frame BuildTargetVelocity(uint8_t nodeId, int32_t velocity) noexcept;

/**
 * @brief Build a target position download frame (index 0x607A).
 * @param nodeId Node ID of the device
 * @param position Target position
 * @return Populated CAN frame
 */
FlexCan::Frame BuildTargetPosition(uint8_t nodeId, int32_t position) noexcept;

} // namespace CanOpen

#endif // UTILITIES_CANOPEN_CANOPENMOTORUTILS_H_
