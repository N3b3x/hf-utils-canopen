/**
 * @file CanOpenUtils.cpp
 * @brief Implementation of CANopen utility helpers.
 */

#include "CanOpenUtils.h"

namespace CanOpen {

CanFrame BuildNmt(uint8_t nodeId, NmtCommand cmd) noexcept {
    CanFrame frame{};
    frame.id = 0x000;
    frame.dlc = 2;
    frame.data[0] = static_cast<uint8_t>(cmd);
    frame.data[1] = nodeId;
    frame.extended = false;
    frame.rtr = false;
    return frame;
}

CanFrame BuildSdoDownload(uint8_t nodeId, uint16_t index, uint8_t subIndex,
                          uint32_t data, uint8_t size) noexcept {
    CanFrame frame{};
    frame.id = 0x600 + nodeId;
    frame.dlc = 8;
    frame.extended = false;
    frame.rtr = false;

    uint8_t command = 0;
    switch (size) {
    case 1:
        command = 0x2F; // expedited, 1 byte
        break;
    case 2:
        command = 0x2B; // expedited, 2 bytes
        break;
    case 4:
    default:
        command = 0x23; // expedited, 4 bytes
        size = 4;
        break;
    }

    frame.data[0] = command;
    frame.data[1] = static_cast<uint8_t>(index & 0xFF);
    frame.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    frame.data[3] = subIndex;
    frame.data[4] = static_cast<uint8_t>(data & 0xFF);
    frame.data[5] = static_cast<uint8_t>((data >> 8) & 0xFF);
    frame.data[6] = static_cast<uint8_t>((data >> 16) & 0xFF);
    frame.data[7] = static_cast<uint8_t>((data >> 24) & 0xFF);

    // For 1 or 2 byte writes, remaining bytes are don't care
    if (size < 4) {
        for (uint8_t i = 4 + size; i < 8; ++i) {
            frame.data[i] = 0x00;
        }
    }

    return frame;
}

} // namespace CanOpen

