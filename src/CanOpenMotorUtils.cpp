/**
 * @file CanOpenMotorUtils.cpp
 * @brief Helper functions for CANopen motor control objects.
 */

#include "CanOpenMotorUtils.h"

namespace CanOpen {

/**
 * @brief Build an SDO upload request frame.
 *
 * @param nodeId Node ID of the target device.
 * @param index Object dictionary index.
 * @param subIndex Sub-index.
 * @return Populated CAN frame.
 */
FlexCan::Frame BuildSdoUpload(uint8_t nodeId, uint16_t index,
                              uint8_t subIndex) noexcept {
    FlexCan::Frame frame{};
    frame.id = 0x600 + nodeId;
    frame.dlc = 8;
    frame.extended = false;
    frame.rtr = false;

    frame.data[0] = 0x40; // initiate upload request
    frame.data[1] = static_cast<uint8_t>(index & 0xFF);
    frame.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    frame.data[3] = subIndex;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    return frame;
}

/**
 * @brief Build a heartbeat frame.
 *
 * @param nodeId Node ID of the device.
 * @param state Node state encoded in the heartbeat.
 * @return Populated CAN frame.
 */
FlexCan::Frame BuildHeartbeat(uint8_t nodeId, uint8_t state) noexcept {
    FlexCan::Frame frame{};
    frame.id = 0x700 + nodeId;
    frame.dlc = 1;
    frame.extended = false;
    frame.rtr = false;
    frame.data[0] = state;
    return frame;
}

/**
 * @brief Build a control word download frame (index 0x6040).
 *
 * @param nodeId Node ID of the device.
 * @param controlWord Control word value.
 * @return Populated CAN frame.
 */
FlexCan::Frame BuildControlWord(uint8_t nodeId, uint16_t controlWord) noexcept {
    return BuildSdoDownload(nodeId, 0x6040, 0x00, controlWord, 2);
}

/**
 * @brief Build a mode of operation download frame (index 0x6060).
 *
 * @param nodeId Node ID of the device.
 * @param mode Desired operation mode.
 * @return Populated CAN frame.
 */
FlexCan::Frame BuildModeOfOperation(uint8_t nodeId, OperationMode mode) noexcept {
    return BuildSdoDownload(nodeId, 0x6060, 0x00,
                            static_cast<uint8_t>(mode), 1);
}

/**
 * @brief Build a target velocity download frame (index 0x60FF).
 *
 * @param nodeId Node ID of the device.
 * @param velocity Target velocity.
 * @return Populated CAN frame.
 */
FlexCan::Frame BuildTargetVelocity(uint8_t nodeId, int32_t velocity) noexcept {
    return BuildSdoDownload(nodeId, 0x60FF, 0x00,
                            static_cast<uint32_t>(velocity), 4);
}

/**
 * @brief Build a target position download frame (index 0x607A).
 *
 * @param nodeId Node ID of the device.
 * @param position Target position.
 * @return Populated CAN frame.
 */
FlexCan::Frame BuildTargetPosition(uint8_t nodeId, int32_t position) noexcept {
    return BuildSdoDownload(nodeId, 0x607A, 0x00,
                            static_cast<uint32_t>(position), 4);
}

} // namespace CanOpen
