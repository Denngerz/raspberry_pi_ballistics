#pragma once
// GeoRef.hpp — local (x,y) simulator plane -> geodetic lat/lon (lesson 34 / 7.1).
//
// The simulator's position is local metres from the start point; MAVLink
// wants degrees. We anchor the plane's origin to a fixed reference point and
// use a flat-Earth approximation (good enough over the few hundred metres a
// mission covers):
//   lat = lat0 + y / 111320
//   lon = lon0 + x / (111320 * cos(lat0))
// 111320 m is the length of one degree of latitude; longitude's degree
// shrinks with cos(latitude).

#include <cmath>

namespace mavlink_link
{

constexpr double kRefLat0 = 50.4501;      // reference origin, degrees
constexpr double kRefLon0 = 30.5234;
constexpr double kMetersPerDegLat = 111320.0;
constexpr double kPi = 3.14159265358979323846;

// Converts local metres (x = east, y = north of the reference point) to
// geodetic latitude/longitude in degrees.
inline void localToGeodetic(double x, double y, double& lat, double& lon)
{
    lat = kRefLat0 + (y / kMetersPerDegLat);
    lon = kRefLon0 + (x / (kMetersPerDegLat * std::cos(kRefLat0 * kPi / 180.0)));
}

// Converts a local heading (radians, math convention: 0 = +x/east, increasing
// counter-clockwise toward +y/north — what Coord::dirVec()/angleOf() use) to
// a compass/NED yaw (radians, 0 = north, increasing clockwise toward east —
// what MAVLink's ATTITUDE.yaw and GLOBAL_POSITION_INT.hdg expect). The two
// conventions differ by both a 90 degree offset AND a sign flip, so this is
// not a simple additive correction.
inline double toCompassYaw(double mathAngleRad)
{
    double yaw = kPi / 2.0 - mathAngleRad;
    while (yaw >   kPi) yaw -= 2.0 * kPi;
    while (yaw <= -kPi) yaw += 2.0 * kPi;
    return yaw;
}

} // namespace mavlink_link
