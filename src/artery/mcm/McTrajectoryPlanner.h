#ifndef ARTERY_MCM_MCTRAJECTORYPLANNER_H_
#define ARTERY_MCM_MCTRAJECTORYPLANNER_H_

#include "traci/API.h"
#include <map>
#include <string>
#include <vector>

namespace artery
{

class McMessage;

/**
 * Longitudinal part of an announced trajectory: constant acceleration for a
 * while, constant speed afterwards. The honest default is constant speed
 * (thesis Section 5.2); an accepting partner announces its deceleration.
 */
struct McLongitudinalProfile
{
    double v0 = 0.0;            //< m/s at generation time
    double accel = 0.0;         //< m/s^2, negative for deceleration
    double accelDuration = 0.0; //< s

    double speed(double t) const;
    double distance(double t) const;
};

/**
 * Lateral part of an announced trajectory, relative to the centre line of the
 * current lane: d(t) moves linearly from d0 to d0 + delta within [start, start + duration].
 * A linear profile is what SUMO drives with the lanechange.duration option.
 */
struct McLateralProfile
{
    double d0 = 0.0;        //< m, lateral offset at generation time (positive to the left)
    double delta = 0.0;     //< m
    double start = 0.0;     //< s after generation time
    double duration = 0.0;  //< s

    double offset(double t) const;
};

/**
 * Builds announced trajectories from the vehicle state in SUMO.
 *
 * The vehicle plans in road coordinates along the centre line of its lane and
 * the lanes continuing it along the route; the points are then converted into
 * east/north offsets of the reference position, so that receivers need no map.
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

    McTrajectoryPlanner(traci::API&, const std::string& vehicleId);

    LaneState laneState() const;
    double laneWidth(const std::string& laneId) const;
    /** id of the neighbouring lane (+1 left, -1 right), empty if there is none */
    std::string neighbourLane(const LaneState&, int direction) const;
    /** true if the current lane does not allow to continue the route (e.g. acceleration lane) */
    bool laneEnds(const LaneState&) const;

    /**
     * Fill the trajectory points of an MCM whose reference position is the
     * current position of the vehicle.
     */
    void plan(McMessage&, const LaneState&, const McLongitudinalProfile&, const McLateralProfile&,
            unsigned numPoints, double spacing) const;

private:
    using Polyline = std::vector<libsumo::TraCIPosition>;

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
