#include "artery/mcm/McGeometry.h"
#include "artery/mcm/Mcm_m.h"
#include <algorithm>
#include <cmath>

namespace artery
{
namespace mcm
{

namespace
{

constexpr double pi = 3.14159265358979323846;
constexpr double deg2rad = pi / 180.0;
constexpr double rad2deg = 180.0 / pi;

// WGS84 ellipsoid
constexpr double semiMajorAxis = 6378137.0;
constexpr double flattening = 1.0 / 298.257223563;
constexpr double eccentricitySquared = flattening * (2.0 - flattening);

void radiiAt(double latitudeDeg, double& meridian, double& primeVertical)
{
    const double s = std::sin(latitudeDeg * deg2rad);
    const double w = 1.0 - eccentricitySquared * s * s;
    primeVertical = semiMajorAxis / std::sqrt(w);
    meridian = semiMajorAxis * (1.0 - eccentricitySquared) / (w * std::sqrt(w));
}

} // namespace

double length(const Enu& v)
{
    return std::hypot(v.east, v.north);
}

Enu toEnu(const Geo& ref, const Geo& pos)
{
    double meridian, primeVertical;
    radiiAt(ref.latitude, meridian, primeVertical);
    Enu enu;
    enu.east = (pos.longitude - ref.longitude) * deg2rad * primeVertical * std::cos(ref.latitude * deg2rad);
    enu.north = (pos.latitude - ref.latitude) * deg2rad * meridian;
    return enu;
}

Geo toGeo(const Geo& ref, const Enu& offset)
{
    double meridian, primeVertical;
    radiiAt(ref.latitude, meridian, primeVertical);
    Geo geo;
    geo.latitude = ref.latitude + offset.north / meridian * rad2deg;
    geo.longitude = ref.longitude + offset.east / (primeVertical * std::cos(ref.latitude * deg2rad)) * rad2deg;
    return geo;
}

Enu headingVector(double headingDeg)
{
    return Enu { std::sin(headingDeg * deg2rad), std::cos(headingDeg * deg2rad) };
}

Enu leftVector(double headingDeg)
{
    return Enu { -std::cos(headingDeg * deg2rad), std::sin(headingDeg * deg2rad) };
}

double headingOf(const Enu& d)
{
    double h = std::atan2(d.east, d.north) * rad2deg;
    return h < 0.0 ? h + 360.0 : h;
}

double wrapAngle(double deg)
{
    deg = std::fmod(deg, 360.0);
    if (deg > 180.0) {
        deg -= 360.0;
    } else if (deg <= -180.0) {
        deg += 360.0;
    }
    return deg;
}

Geo referenceOf(const McMessage& msg)
{
    return Geo { fromTenthMicrodegree(msg.getLatitude()), fromTenthMicrodegree(msg.getLongitude()) };
}

double headingOf(const McMessage& msg)
{
    return msg.getHeading() * 0.1;
}

McPoints referencePoints(const McMessage& msg)
{
    McPoints points(msg.getReferenceArraySize());
    for (unsigned i = 0; i < points.size(); ++i) {
        points[i] = msg.getReference(i);
    }
    return points;
}

McPoints requestedPoints(const McMessage& msg)
{
    McPoints points(msg.getRequestedArraySize());
    for (unsigned i = 0; i < points.size(); ++i) {
        points[i] = msg.getRequested(i);
    }
    return points;
}

void setReferencePoints(McMessage& msg, const McPoints& points)
{
    msg.setReferenceArraySize(points.size());
    for (unsigned i = 0; i < points.size(); ++i) {
        msg.setReference(i, points[i]);
    }
}

void setRequestedPoints(McMessage& msg, const McPoints& points)
{
    msg.setRequestedArraySize(points.size());
    for (unsigned i = 0; i < points.size(); ++i) {
        msg.setRequested(i, points[i]);
    }
}

double horizonOf(const McPoints& points)
{
    return points.empty() ? 0.0 : points.back().timeOffset * 1e-3;
}

TrajectorySample sampleTrajectory(const McPoints& points, double t)
{
    TrajectorySample sample;
    if (points.empty() || t < 0.0 || t > horizonOf(points)) {
        return sample;
    }

    // point 0 is the reference position at generation time
    double t0 = 0.0;
    Enu p0;
    double v0 = points.front().speed * 0.01;
    for (const McTrajectoryPoint& point : points) {
        const double t1 = point.timeOffset * 1e-3;
        const Enu p1 { point.deltaEast * 0.01, point.deltaNorth * 0.01 };
        const double v1 = point.speed * 0.01;
        if (t <= t1) {
            const double f = t1 > t0 ? (t - t0) / (t1 - t0) : 1.0;
            sample.valid = true;
            sample.position = p0 + f * (p1 - p0);
            sample.speed = v0 + f * (v1 - v0);
            const Enu secant = p1 - p0;
            sample.heading = length(secant) > 1e-3 ? headingOf(secant) : point.heading * 0.1;
            return sample;
        }
        t0 = t1;
        p0 = p1;
        v0 = v1;
    }
    return sample;
}

TrajectorySample sampleExtrapolated(const McPoints& points, double t)
{
    const double horizon = horizonOf(points);
    if (points.empty() || t <= horizon) {
        return sampleTrajectory(points, std::max(0.0, t));
    }
    TrajectorySample sample = sampleTrajectory(points, horizon);
    const McTrajectoryPoint& last = points.back();
    sample.speed = last.speed * 0.01;
    sample.position = sample.position + ((t - horizon) * sample.speed) * headingVector(sample.heading);
    return sample;
}

bool conflicts(const Body& a, const Body& b, const ConflictRules& rules)
{
    const Enu forward = headingVector(a.heading);
    const Enu left = leftVector(a.heading);
    const Enu delta = b.front - a.front;
    const double lateral = dot(delta, left);
    if (std::abs(lateral) >= 0.5 * (a.width + b.width) + rules.lateralMargin) {
        return false;
    }

    // longitudinal extents along the heading of a: a in [-length, 0], b in [front - length, front]
    const double bFront = dot(delta, forward);
    const double bRear = bFront - b.length;
    if (bRear >= 0.0) {
        return bRear < rules.minGap + rules.headway * a.speed; // b ahead, a follows
    } else if (bFront <= -a.length) {
        return -a.length - bFront < rules.minGap + rules.headway * b.speed; // b follows a
    } else {
        return true; // overlapping
    }
}

} // namespace mcm
} // namespace artery
