#include "artery/mcm/McTrajectoryPlanner.h"
#include "artery/mcm/Mcm_m.h"
#include <omnetpp.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace artery
{

using libsumo::TraCIPosition;

namespace
{

constexpr double pi = 3.14159265358979323846;

TraCIPosition makePosition(double x, double y)
{
    TraCIPosition p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return p;
}

double distance(const TraCIPosition& a, const TraCIPosition& b)
{
    return std::hypot(b.x - a.x, b.y - a.y);
}

double polylineLength(const std::vector<TraCIPosition>& line)
{
    double total = 0.0;
    for (std::size_t i = 1; i < line.size(); ++i) {
        total += distance(line[i - 1], line[i]);
    }
    return total;
}

/** point at arc length s and unit tangent there; extrapolates beyond both ends */
void pointAt(const std::vector<TraCIPosition>& line, double s, TraCIPosition& point, TraCIPosition& tangent)
{
    double walked = 0.0;
    for (std::size_t i = 1; i < line.size(); ++i) {
        const double segment = distance(line[i - 1], line[i]);
        if (segment < 1e-9) {
            continue;
        }
        const bool last = (i + 1 == line.size());
        if (s <= walked + segment || last) {
            const double f = (s - walked) / segment;
            tangent = makePosition((line[i].x - line[i - 1].x) / segment, (line[i].y - line[i - 1].y) / segment);
            point = makePosition(line[i - 1].x + f * (line[i].x - line[i - 1].x), line[i - 1].y + f * (line[i].y - line[i - 1].y));
            return;
        }
        walked += segment;
    }
    point = line.empty() ? makePosition(0.0, 0.0) : line.front();
    tangent = makePosition(1.0, 0.0);
}

} // namespace

double McLongitudinalProfile::speed(double t) const
{
    const double ta = std::min(std::max(t, 0.0), accelDuration);
    return std::min(maxSpeed, std::max(0.0, v0 + accel * ta));
}

double McLongitudinalProfile::distance(double t) const
{
    t = std::max(t, 0.0);
    double ta = std::min(t, accelDuration);
    if (accel < 0.0 && v0 + accel * ta < 0.0) {
        ta = -v0 / accel; // standstill before end of acceleration phase
    } else if (accel > 0.0 && v0 + accel * ta > maxSpeed) {
        ta = std::max(0.0, (maxSpeed - v0) / accel); // maximum speed reached earlier
    }
    const double accelDistance = v0 * ta + 0.5 * accel * ta * ta;
    return accelDistance + speed(t) * std::max(0.0, t - ta);
}

double McLateralProfile::offset(double t) const
{
    if (t <= start || duration <= 0.0) {
        return t <= start ? d0 : d0 + delta;
    }
    const double x = std::min(1.0, (t - start) / duration);
    double f = x;
    switch (shape) {
        case Shape::Linear:
            break;
        case Shape::Sinusoidal:
            // lateral speed 1 - cos: no jump in lateral speed and acceleration at start and end
            f = x - std::sin(2.0 * pi * x) / (2.0 * pi);
            break;
        case Shape::Quintic:
            // minimum jerk polynomial, common model of lane change trajectories
            f = x * x * x * (10.0 - 15.0 * x + 6.0 * x * x);
            break;
    }
    return d0 + f * delta;
}

McLateralProfile::Shape McLateralProfile::parseShape(const std::string& name)
{
    if (name == "linear") {
        return Shape::Linear;
    } else if (name == "sinusoidal") {
        return Shape::Sinusoidal;
    } else if (name == "quintic") {
        return Shape::Quintic;
    }
    throw omnetpp::cRuntimeError("unknown lateral profile shape '%s' (linear, sinusoidal, quintic)", name.c_str());
}

TraCIPosition McPlan::at(double t, TraCIPosition* direction) const
{
    TraCIPosition onPath, tangent;
    pointAt(path, s0 + lon.distance(t - t0), onPath, tangent);
    const double d = lat.offset(t);
    if (direction) {
        *direction = tangent;
    }
    return makePosition(onPath.x - d * tangent.y, onPath.y + d * tangent.x);
}

McTrajectoryPlanner::McTrajectoryPlanner(traci::API& api, const std::string& id) :
    mApi(api), mVehicleId(id)
{
}

McTrajectoryPlanner::LaneState McTrajectoryPlanner::laneState() const
{
    LaneState state;
    state.laneId = mApi.vehicle.getLaneID(mVehicleId);
    state.edgeId = mApi.vehicle.getRoadID(mVehicleId);
    state.laneIndex = mApi.vehicle.getLaneIndex(mVehicleId);
    state.lanePosition = mApi.vehicle.getLanePosition(mVehicleId);
    state.lateralPosition = mApi.vehicle.getLateralLanePosition(mVehicleId);
    state.internal = !state.laneId.empty() && state.laneId.front() == ':';
    return state;
}

double McTrajectoryPlanner::laneWidth(const std::string& laneId) const
{
    return mApi.lane.getWidth(laneId);
}

std::string McTrajectoryPlanner::neighbourLane(const LaneState& state, int direction) const
{
    if (state.internal) {
        return std::string();
    }
    const int index = state.laneIndex + direction;
    if (index < 0 || index >= mApi.edge.getLaneNumber(state.edgeId)) {
        return std::string();
    }
    return state.edgeId + "_" + std::to_string(index);
}

bool McTrajectoryPlanner::laneEnds(const LaneState& state) const
{
    for (const auto& entry : mApi.vehicle.getBestLanes(mVehicleId)) {
        if (entry.laneID == state.laneId) {
            return !entry.allowsContinuation;
        }
    }
    return false;
}

TraCIPosition McTrajectoryPlanner::vehiclePosition() const
{
    return mApi.vehicle.getPosition(mVehicleId);
}

double McTrajectoryPlanner::vehicleSpeed() const
{
    return mApi.vehicle.getSpeed(mVehicleId);
}

mcm::Geo McTrajectoryPlanner::vehicleGeo() const
{
    const auto geo = mApi.convertGeo(vehiclePosition());
    return mcm::Geo { geo.latitude, geo.longitude };
}

const McTrajectoryPlanner::Polyline& McTrajectoryPlanner::laneShape(const std::string& laneId) const
{
    auto found = mShapes.find(laneId);
    if (found == mShapes.end()) {
        found = mShapes.emplace(laneId, mApi.lane.getShape(laneId).value).first;
    }
    return found->second;
}

double McTrajectoryPlanner::laneLength(const std::string& laneId) const
{
    auto found = mLengths.find(laneId);
    if (found == mLengths.end()) {
        found = mLengths.emplace(laneId, mApi.lane.getLength(laneId)).first;
    }
    return found->second;
}

McTrajectoryPlanner::Polyline McTrajectoryPlanner::buildPath(const LaneState& state, double length) const
{
    // lanes continuing the current one along the route (without internal junction lanes)
    std::vector<std::string> lanes { state.laneId };
    const auto bestLanes = mApi.vehicle.getBestLanes(mVehicleId);
    const libsumo::TraCIBestLanesData* best = nullptr;
    for (const auto& entry : bestLanes) {
        if (entry.laneID == state.laneId) {
            best = &entry;
            break;
        }
    }
    if (!best && state.internal && !bestLanes.empty()) {
        // on a junction the best lanes refer to the next edge: continue on the lane
        // which starts where the internal lane ends
        const TraCIPosition exit = laneShape(state.laneId).back();
        double closest = std::numeric_limits<double>::infinity();
        for (const auto& entry : bestLanes) {
            const double gap = distance(exit, laneShape(entry.laneID).front());
            if (gap < closest) {
                closest = gap;
                best = &entry;
            }
        }
    }
    if (best) {
        for (const auto& lane : best->continuationLanes) {
            if (!lane.empty() && lane != lanes.back()) {
                lanes.push_back(lane);
            }
        }
    }

    const Polyline& current = laneShape(state.laneId);
    const double shapeLength = polylineLength(current);
    const double sumoLength = laneLength(state.laneId);
    const double s0 = sumoLength > 0.0 ? state.lanePosition * shapeLength / sumoLength : state.lanePosition;

    // start some metres behind the vehicle, so that positions slightly behind the
    // anchor (e.g. after braking harder than planned) can still be projected
    Polyline path;
    TraCIPosition point, tangent;
    pointAt(current, s0 - 20.0, point, tangent);
    path.push_back(point);
    double walked = 0.0;
    for (std::size_t i = 1; i < current.size(); ++i) {
        walked += distance(current[i - 1], current[i]);
        if (walked > s0 - 20.0 + 1e-3) {
            path.push_back(current[i]);
        }
    }

    for (std::size_t l = 1; l < lanes.size() && polylineLength(path) < length + 20.0; ++l) {
        for (const TraCIPosition& p : laneShape(lanes[l])) {
            if (distance(path.back(), p) > 1e-2) {
                path.push_back(p);
            }
        }
    }

    // route ends or no continuation known: extend straight along the last direction
    const double built = polylineLength(path);
    if (built < length + 20.0) {
        TraCIPosition end, direction;
        pointAt(path.size() > 1 ? path : current, path.size() > 1 ? built : shapeLength, end, direction);
        const double extra = length + 20.0 - built + 1.0;
        path.push_back(makePosition(path.back().x + extra * direction.x, path.back().y + extra * direction.y));
    }
    return path;
}

McPlan McTrajectoryPlanner::planLane(const LaneState& state, double now, double pathLength) const
{
    McPlan plan;
    plan.path = buildPath(state, pathLength);
    plan.arc.reserve(plan.path.size());
    double total = 0.0;
    for (std::size_t i = 0; i < plan.path.size(); ++i) {
        total += i > 0 ? distance(plan.path[i - 1], plan.path[i]) : 0.0;
        plan.arc.push_back(total);
    }
    const Projection p = project(plan, vehiclePosition());
    plan.s0 = p.s;
    plan.t0 = now;
    plan.lon.v0 = vehicleSpeed();
    plan.lat.d0 = p.lateral;
    plan.lat.start = now;
    return plan;
}

void McTrajectoryPlanner::anchor(McPlan& plan, double now, double speed) const
{
    plan.s0 = project(plan, vehiclePosition()).s;
    plan.t0 = now;
    plan.lon.v0 = speed;
}

McTrajectoryPlanner::Projection McTrajectoryPlanner::project(const McPlan& plan, const TraCIPosition& p) const
{
    Projection best;
    double closest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < plan.path.size(); ++i) {
        const TraCIPosition& a = plan.path[i - 1];
        const TraCIPosition& b = plan.path[i];
        const double segment = distance(a, b);
        if (segment < 1e-9) {
            continue;
        }
        const double dx = (b.x - a.x) / segment;
        const double dy = (b.y - a.y) / segment;
        double u = (p.x - a.x) * dx + (p.y - a.y) * dy;
        // the first and last segment extend beyond the path
        if (i > 1) {
            u = std::max(u, 0.0);
        }
        if (i + 1 < plan.path.size()) {
            u = std::min(u, segment);
        }
        const double fx = a.x + u * dx;
        const double fy = a.y + u * dy;
        const double gap = std::hypot(p.x - fx, p.y - fy);
        if (gap < closest) {
            closest = gap;
            best.s = (i - 1 < plan.arc.size() ? plan.arc[i - 1] : 0.0) + u;
            best.lateral = dx * (p.y - a.y) - dy * (p.x - a.x);
        }
    }
    return best;
}

McTrajectoryPlanner::Frame McTrajectoryPlanner::frame() const
{
    // linearised conversion of SUMO's planar coordinates into east/north offsets
    auto toGeo = [this](double x, double y) {
        const auto geo = mApi.convertGeo(makePosition(x, y));
        return mcm::Geo { geo.latitude, geo.longitude };
    };
    Frame f;
    f.origin = vehiclePosition();
    const mcm::Geo origin = toGeo(f.origin.x, f.origin.y);
    f.unitX = 0.1 * mcm::toEnu(origin, toGeo(f.origin.x + 10.0, f.origin.y));
    f.unitY = 0.1 * mcm::toEnu(origin, toGeo(f.origin.x, f.origin.y + 10.0));
    return f;
}

mcm::McPoints McTrajectoryPlanner::sample(const McPlan& plan, double now, unsigned numPoints, double spacing) const
{
    const Frame f = frame();
    mcm::McPoints points(numPoints);
    mcm::Enu previous;
    double previousHeading = 0.0;
    for (unsigned i = 0; i < numPoints; ++i) {
        const double t = now + (i + 1) * spacing;
        const TraCIPosition p = plan.at(t);
        const mcm::Enu enu = (p.x - f.origin.x) * f.unitX + (p.y - f.origin.y) * f.unitY;

        McTrajectoryPoint& point = points[i];
        point.timeOffset = static_cast<uint16_t>(std::lround((i + 1) * spacing * 1000.0));
        point.deltaEast = static_cast<int32_t>(std::lround(enu.east * 100.0));
        point.deltaNorth = static_cast<int32_t>(std::lround(enu.north * 100.0));
        point.speed = static_cast<uint16_t>(std::lround(plan.lon.speed(t - plan.t0) * 100.0));
        const mcm::Enu secant = enu - previous;
        const double heading = mcm::length(secant) > 1e-3 ? mcm::headingOf(secant) : previousHeading;
        point.heading = static_cast<uint16_t>(std::lround(heading * 10.0)) % 3600;
        previous = enu;
        previousHeading = heading;
    }
    return points;
}

std::vector<mcm::Body> McTrajectoryPlanner::footprints(const McPlan& plan, const std::vector<double>& times,
        double length, double width) const
{
    const Frame f = frame();
    auto toEnu = [&f](const TraCIPosition& p) {
        return (p.x - f.origin.x) * f.unitX + (p.y - f.origin.y) * f.unitY;
    };

    std::vector<mcm::Body> bodies;
    bodies.reserve(times.size());
    for (double t : times) {
        mcm::Body body;
        body.front = toEnu(plan.at(t));
        // direction of travel including the lateral movement
        const mcm::Enu ahead = toEnu(plan.at(t + 0.1));
        const mcm::Enu secant = ahead - body.front;
        TraCIPosition tangent;
        plan.at(t, &tangent);
        body.heading = mcm::length(secant) > 1e-3 ? mcm::headingOf(secant) :
            mcm::headingOf(tangent.x * f.unitX + tangent.y * f.unitY);
        body.speed = plan.lon.speed(t - plan.t0);
        body.length = length;
        body.width = width;
        bodies.push_back(body);
    }
    return bodies;
}

} // namespace artery
