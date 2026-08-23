#pragma once
// MavlinkLink.hpp — MAVLink 2 telemetry + ballistic-drop command over UDP
// (lesson 34 / 7.1, extends HW11).
//
// Sends from the drone-simulator's physics loop so the flight is visible in
// QGroundControl (or the course checker) on 127.0.0.1:14550 by default:
//   * HEARTBEAT              ~1 Hz
//   * GLOBAL_POSITION_INT +
//     ATTITUDE               >=2 Hz, driven by update() (called every
//                             physics tick; internally rate-limited)
//   * COMMAND_LONG(MAV_CMD_USER_1) at the ballistic release moment, retried
//     up to 5 times until a matching COMMAND_ACK(MAV_RESULT_ACCEPTED)
//     arrives. Retries never block telemetry: everything is driven from the
//     same update() call, polled from the physics thread.
//
// Frames are packed/parsed by hand with mavlink/c_library_v2
// (mavlink_msg_*_pack / mavlink_parse_char) over a plain UDP socket — no
// MAVSDK. sysid/compid are constant across every message this link sends.
//
// Thread-safety: triggerDrop() may be called from a different thread than
// update() (the mission thread detects the drop point, the physics thread
// owns the socket and calls update()); everything else is single-threaded,
// used only from update()'s caller.

#include <cstdint>
#include <string>
#include <deque>
#include <mutex>
#include <atomic>
#include <chrono>
#include <netinet/in.h>
#include "dto/Coord.hpp"

namespace mavlink_link
{

class MavlinkLink
{
public:
    // host/port — destination the telemetry/commands are sent to
    // (QGroundControl's default listen address is 127.0.0.1:14550).
    MavlinkLink(std::string host, uint16_t port,
                uint8_t sysid = 1, uint8_t compid = 1 /* MAV_COMP_ID_AUTOPILOT1 */);
    ~MavlinkLink();

    MavlinkLink(const MavlinkLink&)            = delete;
    MavlinkLink& operator=(const MavlinkLink&) = delete;

    // Resolves the destination and opens the UDP socket. Returns false (and
    // logs to stderr) on failure; the link is then inert (update()/
    // triggerDrop() become no-ops) rather than crashing the simulation.
    bool open();

    // Call once per physics tick with the latest state. Internally:
    //   - sends HEARTBEAT if >=1s elapsed since the last one;
    //   - sends GLOBAL_POSITION_INT + ATTITUDE if >= the telemetry period
    //     elapsed (rate-limited well above the required 2 Hz floor);
    //   - drains any pending inbound datagrams (COMMAND_ACK);
    //   - pumps the drop-command retry state machine (see triggerDrop()).
    // t_ms    — simulation time since start, ms (monotonic -> time_boot_ms)
    // pos     — local (x, y) position, metres
    // altM    — altitude, metres
    // velMs   — local (vx, vy) velocity, m/s
    // yawRad  — heading, radians
    void update(uint32_t t_ms, Coord pos, float altM, Coord velMs, float yawRad);

    // Queues a ballistic release event: a COMMAND_LONG(MAV_CMD_USER_1) with
    // the drop point (param5/param6 = lat/lon degrees, param7 = altitude
    // metres). update() sends it and retries (<=5 attempts, one at a time)
    // until a matching COMMAND_ACK arrives or the attempts run out — logging
    // either way. Multiple drops (one per target) are queued and handled
    // strictly in order, one handshake at a time.
    void triggerDrop(Coord dropPointLocal, float altM);

    // True once no drop handshake is in flight and the queue is empty —
    // callers can use this to know it's safe to stop feeding update().
    bool dropsSettled() const;

private:
    using Clock = std::chrono::steady_clock;

    enum class DropPhase { Idle, PendingSend, WaitingAck, Done, GaveUp };

    struct DropRequest { Coord local; float altM; };

    void sendHeartbeatIfDue(Clock::time_point now);
    void sendTelemetryIfDue(Clock::time_point now, uint32_t t_ms,
                            Coord pos, float altM, Coord velMs, float yawRad);
    void drainIncoming();
    void pumpDrop(Clock::time_point now);
    void sendDropCommand(const DropRequest& req);

    std::string host_;
    uint16_t    port_;
    uint8_t     sysid_;
    uint8_t     compid_;

    int         sock_ = -1;
    sockaddr_in destAddr_{};   // where telemetry/commands are sent (sendto)

    Clock::time_point lastHeartbeat_{};
    Clock::time_point lastTelemetry_{};

    // Drop handshake state — owned by the update()/pumpDrop() caller thread.
    mutable std::mutex     dropQueueMutex_;
    std::deque<DropRequest> dropQueue_;

    // Written only by the update()/pumpDrop() caller thread; read by
    // dropsSettled() from other threads, hence atomic.
    std::atomic<DropPhase> dropPhase_{ DropPhase::Idle };
    DropRequest        activeDrop_{};
    int                dropAttempts_ = 0;
    Clock::time_point  dropSentAt_{};

    static constexpr int kMaxDropAttempts = 5;
};

} // namespace mavlink_link
