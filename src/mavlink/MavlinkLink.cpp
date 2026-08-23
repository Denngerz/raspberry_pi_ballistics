#include "mavlink/MavlinkLink.hpp"
#include "mavlink/GeoRef.hpp"

// Generated headers: pack/parse frames by hand, no MAVSDK. Included only
// here (kept out of the header) so the SYSTEM include path in CMakeLists.txt
// is enough to silence their warnings without leaking into every translation
// unit that includes MavlinkLink.hpp.
#include <common/mavlink.h>

#include <cmath>
#include <cstring>
#include <iostream>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace mavlink_link
{

namespace
{
    constexpr auto kHeartbeatPeriod = std::chrono::milliseconds(1000);  // ~1 Hz
    constexpr auto kTelemetryPeriod = std::chrono::milliseconds(100);   // 10 Hz (>= required 2 Hz)
    constexpr auto kDropAckTimeout  = std::chrono::milliseconds(700);

    // Best-effort UDP send: the peer (QGC / checker) may not be up yet, or a
    // frame may get dropped — that is expected and handled by the caller
    // (heartbeats/telemetry just resume next tick, drop commands retry).
    //
    // Deliberately sendto()/recvfrom() on an unconnected socket rather than
    // connect()+send()/recv(): a connected UDP socket only accepts datagrams
    // whose *source port* exactly matches the peer we connected to, but a
    // reply (e.g. COMMAND_ACK) may legitimately come back from a different
    // local port on the peer's side. Filtering by source address is enough.
    void sendRaw(int sock, const sockaddr_in& dest, const mavlink_message_t& msg)
    {
        if (sock < 0) return;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        ::sendto(sock, buf, len, 0, reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
    }
}

MavlinkLink::MavlinkLink(std::string host, uint16_t port, uint8_t sysid, uint8_t compid)
    : host_(std::move(host)), port_(port), sysid_(sysid), compid_(compid)
{
}

MavlinkLink::~MavlinkLink()
{
    if (sock_ >= 0)
        ::close(sock_);
}

bool MavlinkLink::open()
{
    addrinfo hints{};
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    addrinfo* res = nullptr;
    int rc = ::getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res);
    if (rc != 0)
    {
        std::cerr << "[mavlink] getaddrinfo(" << host_ << ':' << port_
                  << ") failed: " << gai_strerror(rc) << '\n';
        return false;
    }

    sock_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock_ < 0)
    {
        perror("[mavlink] socket");
        freeaddrinfo(res);
        return false;
    }

    // Non-blocking: update() must never stall the physics loop waiting on
    // a COMMAND_ACK that may never come.
    int flags = fcntl(sock_, F_GETFL, 0);
    fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

    // Remember the destination for sendto(); deliberately not connect()ed —
    // see the note on sendRaw()/drainIncoming() above.
    std::memcpy(&destAddr_, res->ai_addr, sizeof(destAddr_));
    freeaddrinfo(res);

    // Far enough in the past that the first update() call sends immediately.
    lastHeartbeat_ = Clock::now() - std::chrono::hours(1);
    lastTelemetry_ = Clock::now() - std::chrono::hours(1);

    std::cout << "[mavlink] streaming to " << host_ << ':' << port_
              << " sysid=" << (int)sysid_ << " compid=" << (int)compid_ << '\n';
    return true;
}

void MavlinkLink::update(uint32_t t_ms, Coord pos, float altM, Coord velMs, float yawRad)
{
    if (sock_ < 0) return;

    Clock::time_point now = Clock::now();
    sendHeartbeatIfDue(now);
    sendTelemetryIfDue(now, t_ms, pos, altM, velMs, yawRad);
    drainIncoming();
    pumpDrop(now);
}

void MavlinkLink::sendHeartbeatIfDue(Clock::time_point now)
{
    if (now - lastHeartbeat_ < kHeartbeatPeriod) return;
    lastHeartbeat_ = now;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(sysid_, compid_, &msg,
                                MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                                0 /*base_mode*/, 0 /*custom_mode*/, MAV_STATE_ACTIVE);
    sendRaw(sock_, destAddr_, msg);
}

void MavlinkLink::sendTelemetryIfDue(Clock::time_point now, uint32_t t_ms,
                                     Coord pos, float altM, Coord velMs, float yawRad)
{
    if (now - lastTelemetry_ < kTelemetryPeriod) return;
    lastTelemetry_ = now;

    double lat, lon;
    localToGeodetic(pos.x, pos.y, lat, lon);

    // Coord::dirVec()/angleOf() use the math convention (0 = +x/east, CCW);
    // MAVLink wants compass/NED yaw (0 = north, CW) for both ATTITUDE.yaw
    // and GLOBAL_POSITION_INT.hdg.
    double yawNed     = toCompassYaw((double)yawRad);
    double headingDeg = yawNed * (180.0 / kPi);
    if (headingDeg < 0.0) headingDeg += 360.0;

    mavlink_message_t msg;

    // GLOBAL_POSITION_INT: local x is east, local y is north (matches the
    // lat/lon formula above), so NED vx=north comes from velMs.y and
    // vy=east from velMs.x. No terrain model -> relative_alt == alt.
    mavlink_msg_global_position_int_pack(sysid_, compid_, &msg,
        t_ms,
        (int32_t)std::lround(lat * 1e7),
        (int32_t)std::lround(lon * 1e7),
        (int32_t)std::lround(altM * 1000.0f),
        (int32_t)std::lround(altM * 1000.0f),
        (int16_t)std::lround(velMs.y * 100.0f),
        (int16_t)std::lround(velMs.x * 100.0f),
        0,
        (uint16_t)std::lround(headingDeg * 100.0));
    sendRaw(sock_, destAddr_, msg);

    mavlink_msg_attitude_pack(sysid_, compid_, &msg,
        t_ms, 0.0f /*roll*/, 0.0f /*pitch*/, (float)yawNed,
        0.0f /*rollspeed*/, 0.0f /*pitchspeed*/, 0.0f /*yawspeed*/);
    sendRaw(sock_, destAddr_, msg);
}

void MavlinkLink::drainIncoming()
{
    uint8_t buf[512];
    for (;;)
    {
        // recvfrom() (not recv()) on this unconnected socket: see the note
        // on sendRaw() for why we don't filter by source port.
        ssize_t n = ::recvfrom(sock_, buf, sizeof buf, 0, nullptr, nullptr);
        if (n <= 0) break;   // EAGAIN/EWOULDBLOCK (nothing pending) or error

        mavlink_message_t rxMsg;
        mavlink_status_t  rxStatus;
        for (ssize_t i = 0; i < n; ++i)
        {
            // Own parse channel (distinct from the TX channel used implicitly
            // by mavlink_msg_*_pack) so RX framing state never interferes
            // with our own sequence counter.
            if (!mavlink_parse_char(MAVLINK_COMM_1, buf[i], &rxMsg, &rxStatus))
                continue;
            if (rxMsg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) continue;

            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&rxMsg, &ack);

            if (dropPhase_.load() == DropPhase::WaitingAck &&
                ack.command == MAV_CMD_USER_1 &&
                ack.result  == MAV_RESULT_ACCEPTED)
            {
                dropPhase_.store(DropPhase::Done);
                std::cout << "[mavlink] DROP acked after " << dropAttempts_
                          << " attempt(s)\n";
            }
        }
    }
}

void MavlinkLink::pumpDrop(Clock::time_point now)
{
    DropPhase phase = dropPhase_.load();

    if (phase == DropPhase::Idle || phase == DropPhase::Done || phase == DropPhase::GaveUp)
    {
        std::lock_guard<std::mutex> lock(dropQueueMutex_);
        if (dropQueue_.empty())
        {
            dropPhase_.store(DropPhase::Idle);
            return;
        }
        activeDrop_   = dropQueue_.front();
        dropQueue_.pop_front();
        dropAttempts_ = 0;
        phase = DropPhase::PendingSend;
        dropPhase_.store(phase);
    }

    switch (phase)
    {
    case DropPhase::PendingSend:
        sendDropCommand(activeDrop_);
        ++dropAttempts_;
        dropSentAt_ = now;
        dropPhase_.store(DropPhase::WaitingAck);
        break;

    case DropPhase::WaitingAck:
        if (now - dropSentAt_ >= kDropAckTimeout)
        {
            if (dropAttempts_ < kMaxDropAttempts)
            {
                dropPhase_.store(DropPhase::PendingSend);   // resend next tick
            }
            else
            {
                std::cerr << "[mavlink] DROP: no ACK after " << kMaxDropAttempts
                          << " attempts, giving up\n";
                dropPhase_.store(DropPhase::GaveUp);
            }
        }
        break;

    default:
        break;
    }
}

void MavlinkLink::sendDropCommand(const DropRequest& req)
{
    double lat, lon;
    localToGeodetic(req.local.x, req.local.y, lat, lon);

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(sysid_, compid_, &msg,
        0 /*target_system: broadcast*/, 0 /*target_component: broadcast*/,
        MAV_CMD_USER_1,
        (uint8_t)dropAttempts_,   // confirmation: 0 = first try, 1.. = retransmission
        0.0f, 0.0f, 0.0f, 0.0f,   // param1..4 unused
        (float)lat, (float)lon,  // param5/param6 — drop point, degrees
        req.altM);                // param7 — altitude, metres

    sendRaw(sock_, destAddr_, msg);

    std::cout << "[mavlink] DROP command_long sent (attempt " << (dropAttempts_ + 1)
              << "/" << kMaxDropAttempts << ") lat=" << lat << " lon=" << lon
              << " alt=" << req.altM << '\n';
}

void MavlinkLink::triggerDrop(Coord dropPointLocal, float altM)
{
    std::lock_guard<std::mutex> lock(dropQueueMutex_);
    dropQueue_.push_back({ dropPointLocal, altM });
}

bool MavlinkLink::dropsSettled() const
{
    DropPhase phase = dropPhase_.load();
    if (phase == DropPhase::PendingSend || phase == DropPhase::WaitingAck)
        return false;

    std::lock_guard<std::mutex> lock(dropQueueMutex_);
    return dropQueue_.empty();
}

} // namespace mavlink_link
