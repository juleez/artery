#include "artery/mcm/McLog.h"
#include <omnetpp/cexception.h>

namespace artery
{
namespace mcm
{

McLog& McLog::instance()
{
    static McLog log;
    return log;
}

void McLog::open(const std::string& prefix)
{
    if (prefix == mPrefix) {
        return;
    }

    mPrefix = prefix;
    openFile(mStations, "stations", "station_id,vehicle_id,service");
    // partners: vehicles in conflict with a request (requester) or the requester (yielder), separated by ';'
    openFile(mManeuvers, "maneuvers",
        "station_id,vehicle_id,maneuver_id,role,type,partners,"
        "t_announce,t_commit,t_lateral_start,t_end,outcome,negotiated,acks,nacks,lane_change_duration,abort_after");
    // decision of the partner: accepted, busy, cost or not_cooperative; state of the partner before the decision;
    // gap_longitudinal along the partner's heading (positive: requester ahead), offset_lateral positive to the left;
    // requester_speed from its basic container, requested_end_speed at the end of the requested trajectory;
    // target_speed and acceleration of the yield plan, empty if declined
    openFile(mRequests, "requests",
        "time,station_id,vehicle_id,state,requester_id,request_id,requested_type,decision,"
        "distance,gap_longitudinal,offset_lateral,speed,requester_speed,requested_end_speed,target_speed,acceleration");
    // trajectories: timeOffset:deltaEast:deltaNorth:speed:heading separated by ';',
    // responses: station_id:trajectory_id:accept:reason separated by ';'
    openFile(mTransmissions, "mcm-tx",
        "time,generation_time,station_id,state,reference_id,reference_type,requested_id,requested_type,"
        "generation_delta_time,latitude,longitude,heading,speed,length,width,bytes,reference,requested,responses");
}

void McLog::flush()
{
    mStations.flush();
    mManeuvers.flush();
    mRequests.flush();
    mTransmissions.flush();
}

void McLog::openFile(std::ofstream& file, const std::string& name, const char* header)
{
    if (file.is_open()) {
        file.close();
    }
    const std::string path = mPrefix + "-" + name + ".csv";
    file.open(path, std::ios::out | std::ios::trunc);
    if (!file) {
        throw omnetpp::cRuntimeError("cannot open MCM log file %s", path.c_str());
    }
    file << header << "\n";
}

} // namespace mcm
} // namespace artery
