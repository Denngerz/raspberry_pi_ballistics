cmake_minimum_required(VERSION 3.16)
project(raspberry_pi_ballistics CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra)

find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

# --- ballistic core: solver, config/target loading, component factory ------
# Shared by every executable below; none of it needs MAVLink or threading.
add_library(ballistics_core STATIC
    src/AnalyticalSolver.cpp
    src/TableSolver.cpp
    src/BallisticTable.cpp
    src/ComponentsFabric.cpp
    src/FileConfigLoader.cpp
    src/JsonTargetProvider.cpp
)
target_include_directories(ballistics_core PUBLIC include)
target_link_libraries(ballistics_core PUBLIC nlohmann_json::nlohmann_json)

# --- mission: single-threaded simulator (HW9), logs simulation.json --------
add_executable(mission
    src/main.cpp
    src/MissionProcessor.cpp
    src/states/States.cpp
)
target_link_libraries(mission PRIVATE ballistics_core)

# --- drone: multithreaded simulator (HW10) + MAVLink 2 telemetry (HW11+) ---
# This is "the drone simulator" the lesson 34/7.1 homework extends: its
# physics loop now also streams MAVLink over UDP (127.0.0.1:14550 by
# default) and runs the ballistic-drop COMMAND_LONG/COMMAND_ACK handshake.
add_executable(drone
    src/main_mt.cpp
    src/mt/DronePhysics.cpp
    src/mt/DroneStates.cpp
    src/mt/MissionRunner.cpp
    src/mt/ThreadSafeTargetProvider.cpp
    src/mavlink/MavlinkLink.cpp
)
target_link_libraries(drone PRIVATE ballistics_core Threads::Threads)

# mavlink/c_library_v2 is header-only and vendored by cloning it next to this
# file (see README): git clone --depth 1
# https://github.com/mavlink/c_library_v2.git
# SYSTEM keeps its generated headers from tripping our -Wall -Wextra.
set(MAVLINK_DIR ${CMAKE_SOURCE_DIR}/c_library_v2)
if(NOT EXISTS ${MAVLINK_DIR}/common/mavlink.h)
    message(FATAL_ERROR
        "mavlink/c_library_v2 not found at ${MAVLINK_DIR}. Clone it first:\n"
        "  git clone --depth 1 https://github.com/mavlink/c_library_v2.git")
endif()
target_include_directories(drone SYSTEM PRIVATE ${MAVLINK_DIR})

# --- student: onboard autopilot (HW11, lesson 4.4) — needs libgpiod --------
# Real-hardware / socat+gpio-sim target; only built when libgpiod is
# available, since it's irrelevant to (and shouldn't block) the simulator.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(GPIOD libgpiod)
endif()
if(GPIOD_FOUND)
    add_executable(student src/student.cpp)
    target_include_directories(student PRIVATE ${GPIOD_INCLUDE_DIRS})
    target_link_libraries(student PRIVATE ballistics_core ${GPIOD_LIBRARIES})
else()
    message(STATUS "libgpiod not found — skipping 'student' target")
endif()
