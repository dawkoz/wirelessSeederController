#pragma once

#include <Arduino.h>
#include <string.h>

#include "machine_settings.h"   // NETWORK_ID, ESPNOW_CHANNEL, SEND_INTERVAL_MS, LINK_TIMEOUT_MS

// ---------------------------------------------------------------------------
// Shared ESP-NOW wire protocol for the tractor / seeder / dispenser boards.
//
// All three boards include this same file, so the byte layout can never drift
// between them. Changing anything here means reflashing EVERY board.
//
// Addressing is broadcast (FF:FF:FF:FF:FF:FF). No board knows any other
// board's MAC address, so a module can be swapped for a new one without
// touching firmware anywhere. Packets are told apart by the `type` and
// `sender` fields below, not by who they arrived from.
// ---------------------------------------------------------------------------

// Two fixed bytes at the start of every packet ("Kverneland Seeder"). ESP-NOW
// hands us any frame that lands on our channel, including from unrelated
// projects - checking these first throws out anything that isn't ours before
// it can be interpreted as data.
static constexpr uint8_t PROTOCOL_MAGIC_0 = 'K';
static constexpr uint8_t PROTOCOL_MAGIC_1 = 'S';

// Bump when the layout of any struct below changes. Receivers drop packets
// that don't match, so a half-updated set of boards fails loudly instead of
// quietly misreading each other.
// v3: added dispenser on/off and the automated calibration run.
// v4: clog alarm and unclogging - DispenserMode replaces the old calibration
//     state enum, TractorCommand carries counters for Anuluj/Odetkaj and the
//     tractor's upTimeMs.
// v5: TractorCommand carries wheelMmPerPulse. The wheel sensor turned out to be
//     on the metering drive, whose ratio to the ground wheel depends on the
//     seed-size gear, so the distance per pulse is an operator setting - one
//     measured value per gear - instead of a constant.
// v6: TractorCommand carries burstAngleFactor, the burst-mode calibration: the
//     angle each wheel pulse's burst turns is set on the tractor as a factor,
//     not worked out from the grams-per-100-revolutions calibration.
// v7: burst metering no longer stops for a clog. A stalled burst is tried
//     again with the next pulse, and after BURST_FAILS_BEFORE_UNCLOG in a row
//     the dispenser unclogs by itself (DispenserMode::AutoUnclogging).
//     DispenserStatus counts the failed bursts, and the tractor blinks on each.
// v8: the seeding simulation - TractorCommand.simulationRun asks for it,
//     DispenserMode::Simulating and SimulationDone report it, and
//     DispenserStatus carries its counts.
static constexpr uint8_t PROTOCOL_VERSION = 8;

static const uint8_t BROADCAST_ADDRESS[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum class NodeId : uint8_t {
    Tractor   = 1,
    Seeder    = 2,
    Dispenser = 3,
};

enum class MsgType : uint8_t {
    SeederTelemetry = 1,
    TractorCommand  = 2,
    DispenserStatus = 3,
};

// `linkFlags` reports which nodes the SENDER of the packet has heard from
// recently. This is how the tractor finds out whether the seeder and the
// dispenser can hear each other - something it has no direct way to observe.
static constexpr uint8_t LINK_HEARD_TRACTOR   = 1 << 0;
static constexpr uint8_t LINK_HEARD_SEEDER    = 1 << 1;
static constexpr uint8_t LINK_HEARD_DISPENSER = 1 << 2;

enum class DispenserFault : uint8_t {
    None        = 0,
    NoSpeedData = 1,  // seeder unheard, cannot meter at all
    OverSpeed   = 2,  // required RPM is beyond what the motor can deliver
};

// What the dispenser is doing. DispenserStatus.faultCode is only meaningful
// in Normal and Simulating.
enum class DispenserMode : uint8_t {
    Normal          = 0,   // metering to ground speed (motor off while not moving, off, or no seeder)
    Calibrating     = 1,   // automated calibration run turning the shaft
    CalibrationDone = 2,   // run finished, motor off, until the tractor withdraws the request
    Refused         = 3,   // run refused or stopped because the machine is moving
    Clogged         = 4,   // shaft blocked, motor off, waiting for Anuluj or Odetkaj
    Unclogging      = 5,   // running the reverse/forward sequence
    AutoUnclogging  = 6,   // burst metering: the same sequence, started by the dispenser
                           // itself after failed bursts; back to Normal after it, no alarm
    Simulating      = 7,   // burst metering as in work, standing still, on pulses at
                           // SIMULATION_SPEED_MM_S - the tractor's endurance test
    SimulationDone  = 8,   // SIMULATION_MINUTES are up, motor off, the counts final,
                           // until the tractor withdraws the request
};

struct __attribute__((packed)) MessageHeader {
    uint8_t magic0;
    uint8_t magic1;
    uint8_t version;
    uint8_t networkId;
    MsgType type;
    NodeId  sender;
    uint8_t linkFlags;
    uint8_t reserved;   // keeps the header at 8 bytes and leaves room to grow
};
static_assert(sizeof(MessageHeader) == 8, "MessageHeader layout changed - reflash ALL boards");

// Seeder -> everyone
struct __attribute__((packed)) SeederTelemetry {
    MessageHeader header;
    uint32_t upTimeMs;
    uint32_t wheelPulses;      // cumulative since boot, never reset (see below)
    uint16_t turbineRPM;
    uint16_t womRPM;
    uint16_t groundSpeedMmS;
    uint8_t  wheelTurning;
    uint8_t  tramlineRelayOn;  // actual relay state, not the commanded one
};
static_assert(sizeof(SeederTelemetry) == 24, "SeederTelemetry layout changed - reflash ALL boards");

// Tractor -> everyone
struct __attribute__((packed)) TractorCommand {
    MessageHeader header;
    uint8_t  tramlineNumber;     // 0-based; display adds 1
    uint8_t  tramlineRelayOn;
    uint8_t  dispenserEnabled;   // operator's Wl./Wyl. setting
    uint8_t  calibrationRun;     // level-triggered: 1 = perform a calibration
                                 // run now, 0 = don't / stop. Repeated at the
                                 // normal send rate, so it survives a lost
                                 // packet without needing an ack.
    uint16_t doseKgPerHa;
    uint32_t gramsPer100Rev;     // dispenser calibration, broadcast so the
                                 // dispenser never holds its own copy
    uint32_t upTimeMs;           // tractor millis(); a smaller value than last
                                 // time means it rebooted
    uint8_t  clogClearSeq;       // +1 each time the operator picks Anuluj on the clog screen
    uint8_t  unclogSeq;          // +1 each time the operator picks Odetkaj
    // clogClearSeq/unclogSeq are counters, not flags or one-packet pulses: a
    // flag held high would repeat the action on every packet, and a pulse sent
    // once is lost with one dropped packet. A counter repeated in every packet
    // survives loss and is acted on exactly once per change. The dispenser
    // ignores a change that arrives right after a link gap or a tractor reboot
    // (the resync rule in dispenser_logic.h), so it never acts on boot.
    uint16_t wheelMmPerPulse;    // distance covered between two wheel pulses,
                                 // for the seed-size gear the operator has
                                 // selected. The seeder turns pulses into
                                 // speed with it, the dispenser turns them into
                                 // distance. Never sent as 0; see below.
    uint16_t burstAngleFactor;   // burst metering's calibration, 0-999 from the
                                 // tractor: the angle each pulse's burst turns,
                                 // in thousandths of a reference angle
                                 // (burstRevsPerPulse() in dispenser_logic.h).
                                 // Unused in continuous metering, which goes by
                                 // gramsPer100Rev and the dose.
    uint8_t  simulationRun;      // level-triggered like calibrationRun: 1 = run
                                 // the seeding simulation now, 0 = don't / stop
};
static_assert(sizeof(TractorCommand) == 29, "TractorCommand layout changed - reflash ALL boards");

// Both receivers clamp the wire value the same way: anything outside the limits
// - including the 0 a board that does not set it yet would send - falls back to
// the default rather than metering on nonsense.
inline uint16_t validWheelMmPerPulse(uint16_t value)
{
    if (value < WHEEL_MM_PER_PULSE_MIN || value > WHEEL_MM_PER_PULSE_MAX) return WHEEL_MM_PER_PULSE_DEFAULT;
    return value;
}

// Dispenser -> everyone
struct __attribute__((packed)) DispenserStatus {
    MessageHeader  header;
    uint16_t       targetShaftRPM;
    uint16_t       measuredShaftRPM;
    uint16_t       motorCommandedPermille;   // 0..1000
    uint8_t        motorRunning;
    DispenserFault faultCode;
    DispenserMode  mode;
    uint8_t        progressPercent;   // 0..100 while Calibrating, Unclogging,
                                      // AutoUnclogging or Simulating, 100 in
                                      // CalibrationDone and SimulationDone, 0 otherwise
    uint8_t        burstFailures;     // burst metering: +1 for every burst that
                                      // stalled (BURST_FAIL_MS), wrapping 255 -> 0. A
                                      // counter repeated in every packet, like the
                                      // tractor's clogClearSeq: the tractor blinks
                                      // its yellow LED once per change.
    uint16_t       simBursts;         // the seeding simulation: bursts fired so far,
    uint16_t       simFailures;       // of them failed,
    uint8_t        simUnclogs;        // and unclog sequences they led to. Kept after
                                      // it ends, until the next one starts.
};
static_assert(sizeof(DispenserStatus) == 24, "DispenserStatus layout changed - reflash ALL boards");

// Why wheelPulses is cumulative rather than "pulses since the last packet":
// ESP-NOW is lossy, and a per-interval count that goes missing is gone for
// good, biasing distance and average speed low. A cumulative counter heals
// itself - a receiver that misses several packets still computes the correct
// delta as soon as the next one arrives. It also gives total distance free.

inline void fillHeader(MessageHeader &header, MsgType type, NodeId sender, uint8_t linkFlags)
{
    header.magic0    = PROTOCOL_MAGIC_0;
    header.magic1    = PROTOCOL_MAGIC_1;
    header.version   = PROTOCOL_VERSION;
    header.networkId = NETWORK_ID;
    header.type      = type;
    header.sender    = sender;
    header.linkFlags = linkFlags;
    header.reserved  = 0;
}

// Which sender may send which message type. Validity includes this, so a
// mislabelled packet can never mark a board alive or reach the packet copies.
inline bool senderMatchesType(MsgType type, NodeId sender)
{
    switch (type) {
        case MsgType::SeederTelemetry: return sender == NodeId::Seeder;
        case MsgType::TractorCommand:  return sender == NodeId::Tractor;
        case MsgType::DispenserStatus: return sender == NodeId::Dispenser;
        default:                       return false;
    }
}

// Every receive path must call this before copying anything out of the buffer.
// Checking the length is what stops a packet of the wrong size being memcpy'd
// past the end of the incoming data.
inline bool headerValid(const uint8_t *data, int len, MsgType expectedType, size_t expectedSize)
{
    if (data == nullptr) return false;
    if (len != (int)expectedSize) return false;

    MessageHeader header;
    memcpy(&header, data, sizeof(header));

    return header.magic0    == PROTOCOL_MAGIC_0
        && header.magic1    == PROTOCOL_MAGIC_1
        && header.version   == PROTOCOL_VERSION
        && header.networkId == NETWORK_ID
        && header.type      == expectedType
        && senderMatchesType(header.type, header.sender);
}

// Tracks when each peer was last heard from. Lives here so all three boards
// agree on what "connected" means. Written from the ESP-NOW receive callback
// and read from loop(); 32-bit aligned accesses are atomic on the ESP32, so
// no locking is needed.
class LinkTracker {
public:
    void noteReceived(NodeId sender)
    {
        uint8_t i = (uint8_t)sender;
        if (i < 4) lastRxMs[i] = millis();
    }

    bool isAlive(NodeId node) const
    {
        uint32_t last = lastRxMs[(uint8_t)node];
        return last != 0 && (millis() - last) < LINK_TIMEOUT_MS;
    }

    // UINT32_MAX if never heard from at all.
    uint32_t silentForMs(NodeId node) const
    {
        uint32_t last = lastRxMs[(uint8_t)node];
        return (last == 0) ? UINT32_MAX : (millis() - last);
    }

    uint8_t flags() const
    {
        uint8_t f = 0;
        if (isAlive(NodeId::Tractor))   f |= LINK_HEARD_TRACTOR;
        if (isAlive(NodeId::Seeder))    f |= LINK_HEARD_SEEDER;
        if (isAlive(NodeId::Dispenser)) f |= LINK_HEARD_DISPENSER;
        return f;
    }

private:
    volatile uint32_t lastRxMs[4] = {0, 0, 0, 0};   // indexed by NodeId
};
