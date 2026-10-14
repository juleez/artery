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
    openFile(mManeuvers, "maneuvers",
        "station_id,vehicle_id,maneuver_id,role,type,partner_id,partner_maneuver_id,"
        "t_announce,t_lateral_start,t_end,outcome,accept_received,cancel_sent");
    openFile(mTransmissions, "mcm-tx",
        "time,generation_time,station_id,maneuver_id,phase,type,partner_id,partner_maneuver_id,cancel,cancelled_maneuver_id,"
        "generation_delta_time,latitude,longitude,heading,bytes,points");
}

void McLog::flush()
{
    mStations.flush();
    mManeuvers.flush();
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
