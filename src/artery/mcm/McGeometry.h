#ifndef ARTERY_MCM_MCGEOMETRY_H_
#define ARTERY_MCM_MCGEOMETRY_H_

#include <cstdint>

namespace artery
{

class McMessage;

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

TrajectorySample sampleTrajectory(const McMessage&, double t);
/** time of last trajectory point in seconds, 0 if the message has no trajectory */
double horizonOf(const McMessage&);

/** conversions between message units and SI units */
inline int32_t toTenthMicrodegree(double deg) { return static_cast<int32_t>(deg * 1e7 + (deg >= 0.0 ? 0.5 : -0.5)); }
inline double fromTenthMicrodegree(int32_t value) { return value * 1e-7; }

} // namespace mcm
} // namespace artery

#endif /* ARTERY_MCM_MCGEOMETRY_H_ */
