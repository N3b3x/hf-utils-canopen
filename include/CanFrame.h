#ifndef HF_CANOPEN_CANFRAME_H_
#define HF_CANOPEN_CANFRAME_H_

#include <cstdint>

namespace CanOpen {

/** Basic CAN frame structure used by the utilities. */
struct CanFrame {
    uint32_t id{0};       ///< CAN identifier
    uint8_t dlc{0};       ///< Data length code
    bool extended{false}; ///< Use extended identifier format
    bool rtr{false};      ///< Remote transmission request
    uint8_t data[8]{};    ///< Payload bytes
};

} // namespace CanOpen

#endif // HF_CANOPEN_CANFRAME_H_
