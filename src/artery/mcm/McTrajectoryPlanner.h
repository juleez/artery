#ifndef ARTERY_MCM_MCTRAJECTORYPLANNER_H_
#define ARTERY_MCM_MCTRAJECTORYPLANNER_H_

#include "artery/mcm/McGeometry.h"
#include "traci/API.h"
#include <map>
#include <string>
#include <vector>

namespace artery
{

/**
 * Longitudinal part of a plan: constant acceleration for a while, constant
 * speed afterwards. Times are relative to the anchor time of the plan.
 * Lane keeping is planned at constant speed (thesis Section 5.2); a lane change may
 * include a speed change chosen by the gap selection, a yielding vehicle plans its
 * deceleration or acceleration.
 */
struct McLongitudinalProfile
{
    double v0 = 0.0;            //< m/s at the anchor time
    double accel = 0.0;         //< m/s^2, negative for deceleration
    double accelDuration = 0.0; //< s
    double maxSpeed = 1e9;      //< m/s, an acceleration ends here

    double speed(double t) const;
    double distance(double t) const;
};

/**
 * Lateral part of a plan, as offset from the centre line of the plan's path
 * (positive to the left): d(t) moves from d0 to d0 + delta within
 * [start, start + duration]. Times are absolute simulation times in seconds,
 * so that the lateral movement keeps its timing when a plan is anchored anew.
 */
struct McLateralProfile
{
    enum class Shape { Linear, Sinusoidal, Quintic };

    double d0 = 0.0;        //< m
    double delta = 0.0;     //< m
    double start = 0.0;     //< s, simulation time
    double duration = 0.0;  //< s
    Shape shape = Shape::Quintic;

    double offset(double t) const;
    double end() const { return start + duration; }
    bool moving(double t) const { return duration > 0.0 && t > start && t < end(); }

    static Shape parseShape(const std::string&);
};

/**
 * A plan in SUMO's planar coordinates: the vehicle follows the path (centre line
 * of its lane and the lanes continuing it along the route) with the longitudinal
 * profile and keeps the lateral profile's offset from it. During a lane change
 * the path remains the one of the origin lane.
 */
struct McPlan
{
    std::vector<libsumo::TraCIPosition> path;
    std::vector<double> arc;        //< cumulative arc length of the path vertices
    double s0 = 0.0;                //< arc length of the vehicle at the anchor time
    double t0 = 0.0;                //< anchor time (simulation time in s)
    McLongitudinalProfile lon;
    McLateralProfile lat;
    McManeuverType type = MC_KEEP_LANE;

    bool empty() const { return path.size() < 2; }
    /** planned position and direction of travel of the front bumper at an absolute time */
    libsumo::TraCIPosition at(double t, libsumo::TraCIPosition* direction = nullptr) const;
};

/**
 * Builds and samples plans from the vehicle state in SUMO.
 *
 * Sampled plans are converted into east/north offsets of the current vehicle
 * position, so that receivers need no map.
 */
class McTrajectoryPlanner
{
public:
    struct LaneState
    {
        std::string laneId;
        std::string edgeId;
        int laneIndex = -1;
        double lanePosition = 0.0;
        double lateralPosition = 0.0;   //< positive to the left
        bool internal = false;          //< on a junction
    };

    struct Projection
    {
        double s = 0.0;         //< arc length along the path
        double lateral = 0.0;   //< signed offset from the path, positive to the left
    };

    McTrajectoryPlanner(traci::API&, const std::string& vehicleId);

    LaneState laneState() const;
    double laneWidth(const std::string& laneId) const;
    /** id of the neighbouring lane (+1 left, -1 right), empty if there is none */
    std::string neighbourLane(const LaneState&, int direction) const;
    /** true if the current lane does not allow to continue the route (e.g. acceleration lane) */
    bool laneEnds(const LaneState&) const;
    /** distance from the vehicle to the end of its current lane */
    double remainingLength(const LaneState&) const;

    libsumo::TraCIPosition vehiclePosition() const;
    double vehicleSpeed() const;
    /** WGS84 position of the vehicle, origin of the tangent plane of sample() and footprints() */
    mcm::Geo vehicleGeo() const;

    /**
     * New plan along the current lane, anchored at the current vehicle state:
     * constant speed, the lateral offset is kept.
     */
    McPlan planLane(const LaneState&, double now, double pathLength) const;
    /** anchor an existing plan (same path and lateral profile) at the current vehicle state */
    void anchor(McPlan&, double now, double speed) const;
    Projection project(const McPlan&, const libsumo::TraCIPosition&) const;

    /** trajectory points relative to the current vehicle position, first point at now + spacing */
    mcm::McPoints sample(const McPlan&, double now, unsigned numPoints, double spacing) const;
    /** planned footprints in the tangent plane at the current vehicle position */
    std::vector<mcm::Body> footprints(const McPlan&, const std::vector<double>& times,
            double length, double width) const;

private:
    using Polyline = std::vector<libsumo::TraCIPosition>;

    struct Frame
    {
        libsumo::TraCIPosition origin;  //< SUMO position of the vehicle
        mcm::Enu unitX;                 //< east/north of one metre along SUMO's x axis
        mcm::Enu unitY;
    };

    Frame frame() const;
    const Polyline& laneShape(const std::string& laneId) const;
    double laneLength(const std::string& laneId) const;
    Polyline buildPath(const LaneState&, double length) const;

    traci::API& mApi;
    std::string mVehicleId;
    mutable std::map<std::string, Polyline> mShapes;
    mutable std::map<std::string, double> mLengths;
};

} // namespace artery

#endif /* ARTERY_MCM_MCTRAJECTORYPLANNER_H_ */
