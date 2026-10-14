#include "artery/mcm/McTrajectoryPlanner.h"
#include "artery/mcm/McGeometry.h"
#include "artery/mcm/Mcm_m.h"
#include <omnetpp.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace artery
{

using libsumo::TraCIPosition;

double McLongitudinalProfile::speed(double t) const
{
    const double ta = std::min(std::max(t, 0.0), accelDuration);
    return std::max(0.0, v0 + accel * ta);
}

double McLongitudinalProfile::distance(double t) const
{
    double ta = std::min(std::max(t, 0.0), accelDuration);
    if (accel < 0.0 && v0 + accel * ta < 0.0) {
        ta = -v0 / accel; // standstill before end of acceleration phase
    }
    const double accelDistance = v0 * ta + 0.5 * accel * ta * ta;
    return accelDistance + speed(t) * std::max(0.0, t - ta);
}

double McLateralProfile::offset(double t) const
{
    if (t <= start || duration <= 0.0) {
        return t <= start ? d0 : d0 + delta;
    }
    const double f = std::min(1.0, (t - start) / duration);
    return d0 + f * delta;
}

namespace
{

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

    Polyline path;
    const Polyline& current = laneShape(state.laneId);
    const double shapeLength = polylineLength(current);
    const double sumoLength = laneLength(state.laneId);
    const double s0 = sumoLength > 0.0 ? state.lanePosition * shapeLength / sumoLength : state.lanePosition;

    TraCIPosition point, tangent;
    pointAt(current, s0, point, tangent);
    path.push_back(point);
    double walked = 0.0;
    for (std::size_t i = 1; i < current.size(); ++i) {
        walked += distance(current[i - 1], current[i]);
        if (walked > s0 + 1e-3) {
            path.push_back(current[i]);
        }
    }

    for (std::size_t l = 1; l < lanes.size() && polylineLength(path) < length; ++l) {
        for (const TraCIPosition& p : laneShape(lanes[l])) {
            if (distance(path.back(), p) > 1e-2) {
                path.push_back(p);
            }
        }
    }

    // route ends or no continuation known: extend straight along the last direction
    const double built = polylineLength(path);
    if (built < length) {
        TraCIPosition end, direction;
        pointAt(path.size() > 1 ? path : current, path.size() > 1 ? built : shapeLength, end, direction);
        const double extra = length - built + 1.0;
        path.push_back(makePosition(path.back().x + extra * direction.x, path.back().y + extra * direction.y));
    }
    return path;
}

void McTrajectoryPlanner::plan(McMessage& msg, const LaneState& state, const McLongitudinalProfile& lon,
        const McLateralProfile& lat, unsigned numPoints, double spacing) const
{
    const double horizon = numPoints * spacing;
    const Polyline path = buildPath(state, lon.distance(horizon) + 10.0);

    // anchor the path at the actual vehicle position (front bumper as in the CAM)
    const TraCIPosition vehicle = mApi.vehicle.getPosition(mVehicleId);
    TraCIPosition foot, tangent;
    pointAt(path, 0.0, foot, tangent);
    const TraCIPosition anchor = makePosition(foot.x - lat.d0 * tangent.y, foot.y + lat.d0 * tangent.x);
    const TraCIPosition correction = makePosition(vehicle.x - anchor.x, vehicle.y - anchor.y);
    if (std::hypot(correction.x, correction.y) > 0.5) {
        EV_STATICCONTEXT;
        EV_WARN << "MCM trajectory anchor of " << mVehicleId << " is off by "
            << std::hypot(correction.x, correction.y) << " m\n";
    }

    // linearised conversion of SUMO's planar coordinates into east/north offsets
    using mcm::Geo;
    using mcm::Enu;
    auto toGeo = [this](double x, double y) {
        const auto geo = mApi.convertGeo(makePosition(x, y));
        return Geo { geo.latitude, geo.longitude };
    };
    const Geo origin = toGeo(vehicle.x, vehicle.y);
    const Enu unitX = 0.1 * mcm::toEnu(origin, toGeo(vehicle.x + 10.0, vehicle.y));
    const Enu unitY = 0.1 * mcm::toEnu(origin, toGeo(vehicle.x, vehicle.y + 10.0));

    msg.setPointsArraySize(numPoints);
    Enu previous;
    const double fallbackHeading = mcm::headingOf(msg);
    for (unsigned i = 0; i < numPoints; ++i) {
        const double t = (i + 1) * spacing;
        TraCIPosition onPath, dir;
        pointAt(path, lon.distance(t), onPath, dir);
        const double d = lat.offset(t);
        const double x = onPath.x - d * dir.y + correction.x - vehicle.x;
        const double y = onPath.y + d * dir.x + correction.y - vehicle.y;
        const Enu enu = x * unitX + y * unitY;

        McTrajectoryPoint point;
        point.timeOffset = static_cast<uint16_t>(std::lround(t * 1000.0));
        point.deltaEast = static_cast<int32_t>(std::lround(enu.east * 100.0));
        point.deltaNorth = static_cast<int32_t>(std::lround(enu.north * 100.0));
        point.speed = static_cast<uint16_t>(std::lround(lon.speed(t) * 100.0));
        const Enu secant = enu - previous;
        const double heading = mcm::length(secant) > 1e-3 ? mcm::headingOf(secant) : fallbackHeading;
        point.heading = static_cast<uint16_t>(std::lround(heading * 10.0)) % 3600;
        msg.setPoints(i, point);
        previous = enu;
    }
}

} // namespace artery
