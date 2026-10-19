#ifndef ARTERY_MCM_MCGEOMETRY_H_
#define ARTERY_MCM_MCGEOMETRY_H_

#include "artery/mcm/Mcm_m.h"
#include <cstdint>
#include <vector>

namespace artery
{

namespace mcm
{

/**
 * Offset in a local tangent plane (east/north), in metres.
 */
struct Enu
{
    double east = 0.0;
    double north = 0.0;
};

inline Enu operator+(const Enu& a, const Enu& b) { return Enu { a.east + b.east, a.north + b.north }; }
inline Enu operator-(const Enu& a, const Enu& b) { return Enu { a.east - b.east, a.north - b.north }; }
inline Enu operator*(double f, const Enu& a) { return Enu { f * a.east, f * a.north }; }
inline double dot(const Enu& a, const Enu& b) { return a.east * b.east + a.north * b.north; }
double length(const Enu&);

/**
 * WGS84 position in degrees.
 */
struct Geo
{
    double latitude = 0.0;
    double longitude = 0.0;
};

/**
 * Conversion between WGS84 and a local tangent plane around a reference position.
 * Uses the meridian and prime vertical radii of the WGS84 ellipsoid at the reference
 * latitude; over the few hundred metres of a trajectory horizon the error is in the
 * order of millimetres.
 */
Enu toEnu(const Geo& reference, const Geo& position);
Geo toGeo(const Geo& reference, const Enu& offset);

/** unit vector pointing into heading direction (degree, clockwise from north) */
Enu headingVector(double headingDeg);
/** unit vector pointing to the left of heading direction */
Enu leftVector(double headingDeg);
/** heading of a direction vector (degree, clockwise from north, [0, 360)) */
double headingOf(const Enu& direction);
/** wrap an angle difference into (-180, 180] */
double wrapAngle(double deg);

/** WGS84 reference position of an MCM (basic container) */
Geo referenceOf(const McMessage&);
/** heading of an MCM sender (basic container) in degree */
double headingOf(const McMessage&);

using McPoints = std::vector<McTrajectoryPoint>;

McPoints referencePoints(const McMessage&);
McPoints requestedPoints(const McMessage&);
void setReferencePoints(McMessage&, const McPoints&);
void setRequestedPoints(McMessage&, const McPoints&);

/**
 * Announced state at a time after the generation time of an MCM.
 * The reference position counts as trajectory point at time 0 (thesis Section 4.3).
 */
struct TrajectorySample
{
    bool valid = false;     //< false if t lies outside [0, horizon]
    Enu position;           //< relative to the reference position
    double speed = 0.0;     //< m/s
    double heading = 0.0;   //< degree, direction of the secant between enclosing points
};

TrajectorySample sampleTrajectory(const McPoints&, double t);
/** as sampleTrajectory, but continues the last segment at constant speed beyond the horizon */
TrajectorySample sampleExtrapolated(const McPoints&, double t);
/** time of last trajectory point in seconds, 0 if there are no points */
double horizonOf(const McPoints&);

/**
 * Footprint of a vehicle at one instant in a common local tangent plane.
 * The position is the centre of the front bumper, the vehicle extends
 * length metres against its heading.
 */
struct Body
{
    Enu front;
    double heading = 0.0;   //< degree, clockwise from north, direction of travel
    double roadHeading = -1.0;  //< degree, direction of the lane centre line; negative if unknown
    double speed = 0.0;     //< m/s
    double length = 5.0;    //< m
    double width = 1.8;     //< m
};

struct ConflictRules
{
    double minGap = 2.5;        //< m, standstill gap
    double headway = 1.0;       //< s, time gap of the follower
    double lateralMargin = 0.3; //< m, additional lateral clearance
};

/**
 * True if two bodies are too close: their lateral extents overlap and the longitudinal
 * gap is below minGap + headway * speed of the follower. Both are judged parallel to the
 * road (roadHeading of a, else of b, else the heading of a): during a lane change the
 * heading is tilted by a few degrees, which would shift distant vehicles sideways by
 * the distance times the tangent of that angle. Headings are assumed to be nearly
 * parallel (motorway traffic).
 */
bool conflicts(const Body& a, const Body& b, const ConflictRules&);

/** conversions between message units and SI units */
inline int32_t toTenthMicrodegree(double deg) { return static_cast<int32_t>(deg * 1e7 + (deg >= 0.0 ? 0.5 : -0.5)); }
inline double fromTenthMicrodegree(int32_t value) { return value * 1e-7; }

} // namespace mcm
} // namespace artery

#endif /* ARTERY_MCM_MCGEOMETRY_H_ */
