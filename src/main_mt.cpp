#include "../include/ComponentsFabric.hpp"
#include "../include/interfaces/IBallisticSolver.hpp"
#include "../include/interfaces/IConfigLoader.hpp"
#include "../include/mt/DronePhysics.hpp"
#include "../include/mt/ThreadSafeTargetProvider.hpp"
#include "../include/mt/MissionRunner.hpp"
#include "../include/mavlink/MavlinkLink.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace
{
    // Splits "host:port" into its parts, falling back to defaults for
    // anything missing/unparsable.
    struct Destination { std::string host; uint16_t port; };

    Destination parseDestination(const std::string& s, const Destination& def)
    {
        auto colon = s.rfind(':');
        if (colon == std::string::npos)
            return def;

        Destination d;
        d.host = s.substr(0, colon);
        try { d.port = static_cast<uint16_t>(std::stoi(s.substr(colon + 1))); }
        catch (...) { return def; }
        return d.host.empty() ? Destination{ def.host, d.port } : d;
    }
}

int main(int argc, char** argv)
{
    ComponentsFabric fabric;

    // Load configuration and ammo through the existing loader.
    auto loader = fabric.createLoader(LoaderType::FILE);
    if (!loader->load("data/config.json", "data/ammo.json"))
    {
        std::cerr << "Failed to load configuration\n";
        return 1;
    }
    DroneConfig cfg  = loader->getConfig();
    AmmoParams  ammo = loader->getAmmoParams(cfg.ammoName.c_str());

    // Args: an optional "table" positional switches the solver, an optional
    // "--mavlink host:port" picks the MAVLink UDP destination (default
    // 127.0.0.1:14550, QGroundControl's default listen address).
    SolverType  solverType = SolverType::ANALYTICAL;
    Destination mavDest{ "127.0.0.1", 14550 };

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "table")
        {
            solverType = SolverType::TABLE;
        }
        else if (arg == "--mavlink" && i + 1 < argc)
        {
            mavDest = parseDestination(argv[++i], mavDest);
        }
    }
    auto solver = fabric.createSolver(solverType);

    // Three independently owned components, each with its own thread.
    auto provider = std::make_unique<ThreadSafeTargetProvider>(
        cfg.arrayTimeStep, cfg.targetTimeStep, cfg.timeScale);
    if (!provider->loadFromFile("data/targets.json"))
    {
        std::cerr << "Failed to load targets\n";
        return 1;
    }

    auto physics = std::make_unique<DronePhysics>(cfg, cfg.startPos, cfg.initialDir);

    MissionRunner mission(cfg, ammo, std::move(solver),
                          physics.get(), provider.get());

    // MAVLink 2 telemetry over UDP (lesson 34/7.1): streamed from the same
    // physics loop that already drives the simulation, alongside the
    // ballistic-drop COMMAND_LONG/COMMAND_ACK handshake.
    mavlink_link::MavlinkLink mavlink(mavDest.host, mavDest.port);
    bool mavlinkUp = mavlink.open();
    if (mavlinkUp)
    {
        physics->setTelemetryHook([&mavlink, &cfg](const DroneTelemetry& t) {
            uint32_t t_ms = static_cast<uint32_t>(t.timeSecSinceStart * 1000.0f);
            mavlink.update(t_ms, t.pos, cfg.altitude, t.speed, t.direction);
        });
        mission.setDropHook([&mavlink](Coord dropPointLocal, float altitudeM) {
            mavlink.triggerDrop(dropPointLocal, altitudeM);
        });
    }

    std::thread providerThread(&ThreadSafeTargetProvider::run, provider.get());
    std::thread physicsThread(&DronePhysics::run, physics.get());
    std::thread missionThread(&MissionRunner::run, &mission);

    // Wait until every thread is up before releasing them, so the simulation
    // starts synchronised.
    while (!provider->isThreadReady() ||
           !physics->isThreadReady()  ||
           !mission.isThreadReady())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    provider->start();
    physics->start();
    mission.start();

    missionThread.join();   // the mission is the only thread main waits on

    // The mission can finish while a drop's ACK handshake is still retrying
    // (each attempt waits up to ~0.7s, up to 5 attempts) — keep the physics
    // loop (and with it, MAVLink update()) running until it settles, capped
    // so a peer that never acks can't hang the program.
    if (mavlinkUp)
    {
        constexpr auto kMaxWait = std::chrono::seconds(5);
        auto deadline = std::chrono::steady_clock::now() + kMaxWait;
        while (!mavlink.dropsSettled() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    physics->stop();        // flag + join for the worker threads
    provider->stop();
    providerThread.join();
    physicsThread.join();

    if (!mission.writeLog("simulation.json"))
        std::cerr << "Failed to write simulation.json\n";

    std::cout << "Mission complete -> simulation.json\n";
    return 0;
}
