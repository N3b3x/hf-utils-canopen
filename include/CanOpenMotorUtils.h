#ifndef UTILITIES_CANOPEN_CANOPENMOTORUTILS_H_
#define UTILITIES_CANOPEN_CANOPENMOTORUTILS_H_

#include <cstdint>
#include "CanOpenUtils.h"

namespace CanOpen {

/**
 * @brief Operation modes for drive control.
 */
enum class OperationMode : int8_t {
    ProfilePosition     = 1,  ///< Profile position mode
    ProfileVelocity     = 3,  ///< Profile velocity mode
    ProfileTorque       = 4,  ///< Profile torque mode
    Homing              = 6,  ///< Homing mode
    InterpolatedPosition = 7, ///< Interpolated position mode
    CyclicPosition      = 8,  ///< Cyclic synchronous position mode
    CyclicVelocity      = 9,  ///< Cyclic synchronous velocity mode
    CyclicTorque        = 10  ///< Cyclic synchronous torque mode
};

/**
 * @brief Common CiA DS402 drive states encoded in the status word.
 */
enum class DriveState : uint16_t {
    NotReadyToSwitchOn  = 0x0000,
    SwitchOnDisabled    = 0x0040,
    ReadyToSwitchOn     = 0x0021,
    SwitchedOn          = 0x0023,
    OperationEnabled    = 0x0027,
    QuickStopActive     = 0x0007,
    FaultReactionActive = 0x000F,
    Fault               = 0x0008
};

/**
 * @brief Helper values for common control word commands.
 */
enum class ControlWordCommand : uint16_t {
    Shutdown        = 0x0006,
    SwitchOn        = 0x0007,
    DisableVoltage  = 0x0000,
    QuickStop       = 0x0002,
    DisableOperation = 0x0007,
    EnableOperation = 0x000F,
    FaultReset      = 0x0080
};

/**
 * @brief Build a SDO upload request frame.
 * @param nodeId Node ID of the target device
 * @param index Object dictionary index
 * @param subIndex Sub-index
 * @return Populated CAN frame
 */
CanFrame BuildSdoUpload(uint8_t nodeId, uint16_t index,
                        uint8_t subIndex) noexcept;

/**
 * @brief Build a heartbeat frame.
 * @param nodeId Node ID of the device
 * @param state Node state encoded in the heartbeat
 * @return Populated CAN frame
 */
CanFrame BuildHeartbeat(uint8_t nodeId, uint8_t state) noexcept;

/**
 * @brief Build a control word download frame (index 0x6040).
 * @param nodeId Node ID of the device
 * @param controlWord Control word value
 * @return Populated CAN frame
 */
CanFrame BuildControlWord(uint8_t nodeId, uint16_t controlWord) noexcept;

/**
 * @brief Build a mode of operation download frame (index 0x6060).
 * @param nodeId Node ID of the device
 * @param mode Desired operation mode
 * @return Populated CAN frame
 */
CanFrame BuildModeOfOperation(uint8_t nodeId, OperationMode mode) noexcept;

/**
 * @brief Build a target velocity download frame (index 0x60FF).
 * @param nodeId Node ID of the device
 * @param velocity Target velocity
 * @return Populated CAN frame
 */
CanFrame BuildTargetVelocity(uint8_t nodeId, int32_t velocity) noexcept;

/**
 * @brief Build a target position download frame (index 0x607A).
 * @param nodeId Node ID of the device
 * @param position Target position
 * @return Populated CAN frame
 */
CanFrame BuildTargetPosition(uint8_t nodeId, int32_t position) noexcept;

/**
 * @brief Request the status word (index 0x6041).
 */
CanFrame BuildStatusWordRequest(uint8_t nodeId) noexcept;

/**
 * @brief Request the actual position (index 0x6064).
 */
CanFrame BuildActualPositionRequest(uint8_t nodeId) noexcept;

/**
 * @brief Request the actual velocity (index 0x606C).
 */
CanFrame BuildActualVelocityRequest(uint8_t nodeId) noexcept;

/**
 * @brief Build a target torque download frame (index 0x6071).
 */
CanFrame BuildTargetTorque(uint8_t nodeId, int16_t torque) noexcept;

/**
 * @brief Build a target acceleration download frame (index 0x60C5).
 */
CanFrame BuildTargetAcceleration(uint8_t nodeId, int32_t acceleration) noexcept;

/**
 * @brief Build a max profile velocity download frame (index 0x607F).
 */
CanFrame BuildMaxProfileVelocity(uint8_t nodeId, uint32_t velocity) noexcept;

/**
 * @brief Build a profile acceleration download frame (index 0x6083).
 */
CanFrame BuildProfileAcceleration(uint8_t nodeId, uint32_t accel) noexcept;

/**
 * @brief Build a profile deceleration download frame (index 0x6084).
 */
CanFrame BuildProfileDeceleration(uint8_t nodeId, uint32_t decel) noexcept;

} // namespace CanOpen

#endif // UTILITIES_CANOPEN_CANOPENMOTORUTILS_H_
