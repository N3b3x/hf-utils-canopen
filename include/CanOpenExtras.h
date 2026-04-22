/**
 * @file CanOpenExtras.h
 * @brief Header-only extensions to the `CanOpen` namespace:
 *          - Frame parsers (SDO response, heartbeat, emergency)
 *          - DS402 status-word decode / control-word encode helpers
 *          - `FrameCodec` interface + `PassthroughCodec` default
 *
 * @details
 * These helpers complement the existing `CanOpenUtils.h` (build helpers
 * that produce frames) with their inverse — parsers that decode frames
 * received from a slave. Plus a small `FrameCodec` interface so
 * applications can interpose an encryption / authentication layer on
 * the 8-byte payload (the CAN identifier itself stays in clear so
 * COB-ID routing still works).
 *
 * Pattern intentionally mirrors `CanFrame.h` / `CanOpenUtils.h`: header
 * only, single namespace, no allocations, trivially-constexpr-friendly
 * structs. Usable from any consumer that can already include the rest
 * of `hf-utils-canopen`.
 *
 * @copyright HardFOC.
 */

#ifndef HF_UTILS_CANOPEN_CANOPEN_EXTRAS_H_
#define HF_UTILS_CANOPEN_CANOPEN_EXTRAS_H_

#include "CanFrame.h"
#include "CanOpenMotorUtils.h"
#include "CanOpenUtils.h"

#include <cstdint>
#include <cstring>

namespace CanOpen {

// ─────────────────────────────────────────────────────────────────────────────
// SDO response parsing (expedited only — single 8-byte frame)
// ─────────────────────────────────────────────────────────────────────────────

enum class SdoResponseKind : uint8_t {
    Unknown,
    UploadResponse,    ///< 0x4* command specifier — server uploaded data
    DownloadAck,       ///< 0x60 — server acknowledged a download
    AbortTransfer,     ///< 0x80 — abort with code in data[4..7]
};

struct ExpeditedSdoResponse {
    bool valid{false};
    SdoResponseKind kind{SdoResponseKind::Unknown};
    uint8_t nodeId{0};
    uint16_t index{0};
    uint8_t subIndex{0};
    uint8_t data_len{0};
    uint32_t data{0};         ///< Up to 4 bytes of payload (LE)
    uint32_t abort_code{0};   ///< Valid only when kind == AbortTransfer
};

inline ExpeditedSdoResponse ParseSdoResponse(const CanFrame& f) noexcept {
    ExpeditedSdoResponse r{};
    if (f.dlc < 8 || f.id < 0x580 || f.id > 0x5FF) return r;
    r.nodeId = static_cast<uint8_t>(f.id - 0x580);
    r.index = static_cast<uint16_t>(f.data[1]) |
              (static_cast<uint16_t>(f.data[2]) << 8);
    r.subIndex = f.data[3];

    const uint8_t cs = f.data[0];
    if (cs == 0x80) {
        r.kind = SdoResponseKind::AbortTransfer;
        r.abort_code = static_cast<uint32_t>(f.data[4]) |
                       (static_cast<uint32_t>(f.data[5]) << 8) |
                       (static_cast<uint32_t>(f.data[6]) << 16) |
                       (static_cast<uint32_t>(f.data[7]) << 24);
        r.valid = true;
    } else if (cs == 0x60) {
        r.kind = SdoResponseKind::DownloadAck;
        r.valid = true;
    } else if ((cs & 0xE0) == 0x40) {
        r.kind = SdoResponseKind::UploadResponse;
        const bool e = (cs & 0x02) != 0;       ///< expedited
        const bool s = (cs & 0x01) != 0;       ///< size indicated
        if (e && s) {
            r.data_len = static_cast<uint8_t>(4 - ((cs >> 2) & 0x03));
        } else if (e) {
            r.data_len = 4;
        } else {
            r.data_len = 0;
        }
        if (r.data_len > 4) r.data_len = 4;
        uint32_t v = 0;
        for (uint8_t i = 0; i < r.data_len; ++i) {
            v |= static_cast<uint32_t>(f.data[4 + i]) << (8 * i);
        }
        r.data = v;
        r.valid = true;
    }
    return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Heartbeat (NMT slave heartbeat: 0x700 + nodeId)
// ─────────────────────────────────────────────────────────────────────────────

enum class NmtState : uint8_t {
    BootUp = 0x00,
    Stopped = 0x04,
    Operational = 0x05,
    PreOperational = 0x7F,
    Unknown = 0xFF,
};

struct HeartbeatMessage {
    bool valid{false};
    uint8_t nodeId{0};
    NmtState state{NmtState::Unknown};
};

inline HeartbeatMessage ParseHeartbeat(const CanFrame& f) noexcept {
    HeartbeatMessage m{};
    if (f.dlc < 1 || f.id < 0x700 || f.id > 0x77F) return m;
    m.nodeId = static_cast<uint8_t>(f.id - 0x700);
    const uint8_t s = f.data[0] & 0x7F;
    switch (s) {
        case 0x00: m.state = NmtState::BootUp; break;
        case 0x04: m.state = NmtState::Stopped; break;
        case 0x05: m.state = NmtState::Operational; break;
        case 0x7F: m.state = NmtState::PreOperational; break;
        default:   m.state = NmtState::Unknown; break;
    }
    m.valid = true;
    return m;
}

// ─────────────────────────────────────────────────────────────────────────────
// Emergency (0x80 + nodeId, 8-byte payload: error code (LE) + reg + 5 vendor)
// ─────────────────────────────────────────────────────────────────────────────

struct EmergencyMessage {
    bool valid{false};
    uint8_t nodeId{0};
    uint16_t error_code{0};
    uint8_t error_register{0};
    uint8_t vendor[5]{};
};

inline EmergencyMessage ParseEmergency(const CanFrame& f) noexcept {
    EmergencyMessage e{};
    if (f.dlc < 8 || f.id < 0x081 || f.id > 0x0FF) return e;
    e.nodeId = static_cast<uint8_t>(f.id - 0x080);
    e.error_code = static_cast<uint16_t>(f.data[0]) |
                   (static_cast<uint16_t>(f.data[1]) << 8);
    e.error_register = f.data[2];
    std::memcpy(e.vendor, &f.data[3], 5);
    e.valid = true;
    return e;
}

// ─────────────────────────────────────────────────────────────────────────────
// DS402 status-word / control-word helpers
// ─────────────────────────────────────────────────────────────────────────────

inline DriveState DecodeStatusWord(uint16_t sw) noexcept {
    constexpr uint16_t kMask = 0x006F;
    const uint16_t m = sw & kMask;
    if ((m & 0x004F) == 0x0000) return DriveState::NotReadyToSwitchOn;
    if ((m & 0x004F) == 0x0040) return DriveState::SwitchOnDisabled;
    if ((m & 0x006F) == 0x0021) return DriveState::ReadyToSwitchOn;
    if ((m & 0x006F) == 0x0023) return DriveState::SwitchedOn;
    if ((m & 0x006F) == 0x0027) return DriveState::OperationEnabled;
    if ((m & 0x006F) == 0x0007) return DriveState::QuickStopActive;
    if ((m & 0x004F) == 0x000F) return DriveState::FaultReactionActive;
    if ((m & 0x004F) == 0x0008) return DriveState::Fault;
    return DriveState::NotReadyToSwitchOn;
}

inline const char* DriveStateName(DriveState s) noexcept {
    switch (s) {
        case DriveState::NotReadyToSwitchOn:  return "NotReadyToSwitchOn";
        case DriveState::SwitchOnDisabled:    return "SwitchOnDisabled";
        case DriveState::ReadyToSwitchOn:     return "ReadyToSwitchOn";
        case DriveState::SwitchedOn:          return "SwitchedOn";
        case DriveState::OperationEnabled:    return "OperationEnabled";
        case DriveState::QuickStopActive:     return "QuickStopActive";
        case DriveState::FaultReactionActive: return "FaultReactionActive";
        case DriveState::Fault:               return "Fault";
    }
    return "?";
}

/// Suggested control word to drive `current` toward `desired` (one transition).
inline ControlWordCommand NextControlWord(DriveState current, DriveState desired) noexcept {
    if (current == DriveState::Fault) return ControlWordCommand::FaultReset;
    if (desired == DriveState::OperationEnabled) {
        switch (current) {
            case DriveState::SwitchOnDisabled: return ControlWordCommand::Shutdown;
            case DriveState::ReadyToSwitchOn:  return ControlWordCommand::SwitchOn;
            case DriveState::SwitchedOn:       return ControlWordCommand::EnableOperation;
            default:                           return ControlWordCommand::EnableOperation;
        }
    }
    if (desired == DriveState::SwitchOnDisabled) return ControlWordCommand::DisableVoltage;
    return ControlWordCommand::Shutdown;
}

// ─────────────────────────────────────────────────────────────────────────────
// FrameCodec — interface for encrypted / authenticated CAN payloads
// ─────────────────────────────────────────────────────────────────────────────
//
// Wraps the 8-byte payload only — `id`, `dlc`, `extended`, `rtr` MUST be
// left untouched so CANopen routing still works. A real implementation
// can carry an AES-GCM tag in a companion frame, prepend a nonce, etc.

class FrameCodec {
public:
    virtual ~FrameCodec() = default;
    /// Encode payload in-place before send. Return true to send.
    virtual bool EncodeOutgoing(CanFrame& inout) noexcept = 0;
    /// Decode payload in-place after receive. Return true to deliver to parser.
    virtual bool DecodeIncoming(CanFrame& inout) noexcept = 0;
};

class PassthroughCodec final : public FrameCodec {
public:
    bool EncodeOutgoing(CanFrame&) noexcept override { return true; }
    bool DecodeIncoming(CanFrame&) noexcept override { return true; }
};

inline FrameCodec& DefaultCodec() noexcept {
    static PassthroughCodec inst;
    return inst;
}

}  // namespace CanOpen

#endif  // HF_UTILS_CANOPEN_CANOPEN_EXTRAS_H_
