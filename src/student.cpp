// student.cpp — onboard autopilot for the drone (HW11, lesson 4.4).
//
// Closed control loop over UART + GPIO:
//   * read TELEMETRY / TARGET / AMMO frames from the serial link;
//   * guidance: use the ballistic solver to compute a release point with lead;
//   * control: every tick send a CONTROL frame (accel, turnRate) to steer;
//   * raise the START line on boot, pulse the DROP line at the release moment.
//
// The same binary runs both in simulation (socat + gpio-sim) and on a real
// Raspberry Pi.

#include "drone_link.h"
#include "AnalyticalSolver.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "dto/AmmoParams.hpp"
#include "dto/Coord.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <gpiod.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <string>

using namespace dlink;

// ===========================================================================
// Guidance + control (pure functions: no stored state, everything is passed in)
// ===========================================================================

// Empirical correction for the physics used by the reference simulator.
// The analytical model OVER-estimates the horizontal projectile range (it
// reports ~44 m where the reference simulator throws ~34 m — strong projectile
// drag) but UNDER-estimates the fall time (4.55 s vs ~5.47 s). Range and time
// are therefore corrected independently:
//   kRangeScale — scales the horizontal range. If the projectile falls SHORT
//                 of the target increase it; if it OVERSHOOTS decrease it.
//   kTimeScale  — scales the fall time used to lead a moving target (has no
//                 effect on a stationary target).
static constexpr double kRangeScale = 0.775;
static constexpr double kTimeScale  = 1.20;

// Bundle describing where and how a ballistic release should happen.
struct DropPlan
{
    Coord  dropPoint;   // point the vehicle must reach to release
    Coord  predicted;   // target position at projectile impact
    double hLength;     // horizontal range of the projectile
    double tof;         // projectile time of flight
};

// Given a shooter (position, heading, speed, altitude), a projectile spec and a
// moving target, returns the point from which releasing makes the projectile
// land on the target. The target is led by the flight time, and the release
// point is pulled back from the target by the projectile range along the line
// of sight.
static DropPlan planDrop(IBallisticSolver& solver,
                         Coord dronePos,
                         float droneDir,
                         float speed,
                         float alt,
                         const AmmoParams& ammo,
                         Coord targetPos,
                         Coord targetVel)
{
    Ballistics b = solver.solve(alt, speed, ammo);
    if (b.hLength < 0.0 || b.flightTime < 0.0)   // no valid solution -> aim straight at the target
    {
        b.hLength = 0.0;
        b.flightTime = 0.0;
    }

    double hLen = b.hLength    * kRangeScale;
    double tof  = b.flightTime * kTimeScale;

    Coord predicted = targetPos + targetVel * (float)tof;

    Coord dir = normalize(predicted - dronePos);
    if (dir.x == 0.f && dir.y == 0.f)
        dir = dirVec(droneDir);

    return { predicted - dir * (float)hLen, predicted, hLen, tof };
}

// Clamps a value to the range [-1, 1].
static float clamp1(float v)
{
    return v > 1.f ? 1.f : (v < -1.f ? -1.f : v);
}

// Signed angle (radians) from the current heading to the direction of a goal
// point. Positive means the goal lies to the left of the heading.
static float headingError(Coord pos, float dir, Coord goal)
{
    return normalizeAngle(angleOf(goal - pos) - dir);
}

// Normalized steering command toward a goal point.
struct ControlCmd
{
    float accel;      // [-1..1]
    float turnRate;   // [-1..1]
};

// Turns a goal point into normalized steering: a proportional turn toward the
// goal, and throttle that is full when aimed at the goal and falls off (down to
// a gentle brake) as the heading error grows. turnSign flips the turn direction
// if the platform's convention is inverted.
static ControlCmd steer(Coord pos,
                        float dir,
                        Coord goal,
                        float turnGain = 2.5f,
                        float turnSign = +1.0f)
{
    float err = headingError(pos, dir, goal);

    ControlCmd c;
    c.turnRate = clamp1(turnSign * turnGain * err);
    c.accel    = clamp1(std::cos(err));
    if (c.accel < 0.f)
        c.accel *= 0.5f;   // softer brake so the vehicle keeps moving

    return c;
}

// True when it is time to trigger the one-shot release: the vehicle is passing
// the release point inside a lateral corridor of width hitRadius with the
// heading aligned, or the predicted impact is already well within hitRadius.
static bool readyToDrop(Coord pos,
                        float dir,
                        float speed,
                        const DropPlan& p,
                        float hitRadius,
                        float alignRad = 0.12f)
{
    if (p.hLength <= 0.0 || speed < 0.5f)
        return false;
    if (hitRadius <= 0.f)
        hitRadius = 1.f;
    if (std::fabs(headingError(pos, dir, p.dropPoint)) > alignRad)
        return false;

    Coord u = dirVec(dir);
    Coord toDrop = p.dropPoint - pos;

    float along   = toDrop.x * u.x + toDrop.y * u.y;          // distance still ahead
    float lateral = std::fabs(toDrop.x * u.y - toDrop.y * u.x); // sideways offset

    Coord impact = pos + u * (float)p.hLength;
    float miss   = length(p.predicted - impact);

    return (along <= 0.f && lateral <= hitRadius) || (miss <= 0.5f * hitRadius);
}

// ===========================================================================
// UART
// ===========================================================================

// Opens a serial port in raw, non-blocking 8N1 mode at 115200 baud.
// O_NONBLOCK makes subsequent read() calls return immediately (-1/EAGAIN when
// no data is available) instead of blocking.
static int openUart(const char* dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        perror("open");
        return -1;
    }

    termios tio{};
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);                       // 8N1, no character processing
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);            // both directions must match
    tio.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &tio);

    return fd;
}

// Encodes and writes a single control frame (two normalized floats) to the port.
static void sendControl(int fd, float accel, float turnRate)
{
    Control c{ accel, turnRate };

    uint8_t out[64];
    size_t  m = encode(PKT_CONTROL, &c, sizeof c, out);

    ssize_t w = write(fd, out, m);
    (void)w;
}

// ===========================================================================
// CLI + main loop
// ===========================================================================

// Parsed command-line options.
struct Args
{
    std::string uart     = "/dev/ttyAMA1";   // sim: /tmp/ttyA
    std::string gpiochip = "gpiochip0";      // sim: name printed by the reference tool

    unsigned startLine = 24;
    unsigned dropLine  = 23;
};

// Parses command-line arguments into an Args struct, keeping defaults for any
// option that is not supplied.
static Args parseArgs(int argc, char** argv)
{
    Args a;

    for (int i = 1; i < argc; ++i)
    {
        std::string k = argv[i];
        auto next = [&](const std::string& def) {
            return (i + 1 < argc) ? std::string(argv[++i]) : def;
        };

        if      (k == "--uart")       a.uart      = next(a.uart);
        else if (k == "--gpiochip")   a.gpiochip  = next(a.gpiochip);
        else if (k == "--start-line") a.startLine = (unsigned)std::atoi(next("24").c_str());
        else if (k == "--drop-line")  a.dropLine  = (unsigned)std::atoi(next("23").c_str());
    }

    return a;
}

int main(int argc, char** argv)
{
    Args args = parseArgs(argc, argv);

    // --- Serial link ---
    int fd = openUart(args.uart.c_str());
    if (fd < 0)
    {
        fprintf(stderr, "UART open failed: %s\n", args.uart.c_str());
        return 1;
    }

    // --- GPIO (libgpiod v2): START + DROP as outputs ---
    std::string chipPath = "/dev/" + args.gpiochip;

    gpiod_chip* chip = gpiod_chip_open(chipPath.c_str());
    if (!chip)
    {
        perror("gpiod_chip_open");
        return 1;
    }

    gpiod_line_settings* settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    unsigned offsets[2] = { args.startLine, args.dropLine };

    gpiod_line_config* lineCfg = gpiod_line_config_new();
    gpiod_line_config_add_line_settings(lineCfg, offsets, 2, settings);

    gpiod_request_config* reqCfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(reqCfg, "drone");

    gpiod_line_request* request = gpiod_chip_request_lines(chip, reqCfg, lineCfg);
    if (!request)
    {
        perror("request_lines");
        return 1;
    }

    // Raise START and hold it: signals "ready" so the peer starts the run.
    gpiod_line_request_set_value(request, args.startLine, GPIOD_LINE_VALUE_ACTIVE);

    // Emits a single short pulse on the DROP line (release trigger).
    auto pulseDrop = [&]() {
        gpiod_line_request_set_value(request, args.dropLine, GPIOD_LINE_VALUE_ACTIVE);
        usleep(150000);
        gpiod_line_request_set_value(request, args.dropLine, GPIOD_LINE_VALUE_INACTIVE);
    };

    // --- Guidance ---
    auto solver = std::make_unique<AnalyticalSolver>();

    // --- Ammo / mission parameters (filled from the AMMO frame) ---
    AmmoParams ammo;
    float      hitRadius = 3.f;
    bool       haveAmmo  = false;

    // --- Target tracking (filled from TARGET frames) ---
    int   chosenTargetId = -1;      // lock onto the first target seen
    bool  haveTarget     = false;
    Coord targetPos{};
    Coord targetVel{};
    float targetTime     = -1.f;    // timestamp of the last target update

    // --- Release latch ---
    bool dropped = false;

    // --- Latest vehicle state (filled from TELEMETRY frames) ---
    Coord pos{};
    float dir       = 0.f;
    float speed     = 0.f;
    float alt       = 0.f;
    float telemTime = 0.f;

    // --- Serial receive buffer + frame parser ---
    Parser  parser;
    uint8_t buf[256];

    printf("student up: uart=%s chip=%s START=%u DROP=%u\n",
           args.uart.c_str(), args.gpiochip.c_str(), args.startLine, args.dropLine);

    while (true)
    {
        int n = read(fd, buf, sizeof(buf));

        uint8_t type;
        uint8_t len;
        uint8_t payload[260];

        for (int i = 0; i < n; ++i)
        {
            if (!parser.feed(buf[i], type, payload, len))
                continue;

            if (type == PKT_AMMO)
            {
                AmmoCfg a;
                memcpy(&a, payload, sizeof a);

                ammo.name = std::string(a.name, strnlen(a.name, sizeof a.name));
                ammo.mass = a.mass;
                ammo.drag = a.drag;
                ammo.lift = a.lift;

                hitRadius = a.hitRadius;
                haveAmmo  = true;

                printf("AMMO %s hitR=%.2f nTargets=%d\n",
                       ammo.name.c_str(), a.hitRadius, a.nTargets);
            }
            else if (type == PKT_TARGET)
            {
                TargetPos t;
                memcpy(&t, payload, sizeof t);

                if (chosenTargetId < 0)
                    chosenTargetId = t.id;
                if ((int)t.id != chosenTargetId)
                    continue;

                Coord now{ t.x, t.y };

                // Estimate target velocity from successive positions (for lead).
                if (haveTarget && telemTime > targetTime)
                {
                    float d = telemTime - targetTime;
                    if (d > 1e-4f)
                        targetVel = targetVel * 0.5f + ((now - targetPos) / d) * 0.5f;
                }

                targetPos  = now;
                targetTime = telemTime;
                haveTarget = true;
            }
            else if (type == PKT_TELEMETRY)
            {
                Telemetry tel;
                memcpy(&tel, payload, sizeof tel);

                pos       = { tel.x, tel.y };
                dir       = tel.dir;
                speed     = tel.speed;
                alt       = tel.z;
                telemTime = tel.t_ms / 1000.f;

                // Emit one control command per telemetry frame.
                if (haveAmmo && haveTarget)
                {
                    DropPlan p = planDrop(*solver, pos, dir, speed, alt,
                                          ammo, targetPos, targetVel);

                    ControlCmd c = steer(pos, dir, p.dropPoint);
                    sendControl(fd, c.accel, c.turnRate);

                    if (!dropped && readyToDrop(pos, dir, speed, p, hitRadius))
                    {
                        pulseDrop();
                        dropped = true;
                        printf("DROP at (%.1f,%.1f) v=%.1f hLen=%.1f tof=%.2f\n",
                               pos.x, pos.y, speed, p.hLength, p.tof);
                    }
                }
                else
                {
                    sendControl(fd, 0.f, 0.f);   // no goal yet: keep the link alive
                }
            }
        }

        usleep(2000);
    }

    // (unreachable in the infinite loop; on real hardware release resources)
    gpiod_line_request_release(request);
    gpiod_chip_close(chip);
    close(fd);
    return 0;
}