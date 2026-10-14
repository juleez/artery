#ifndef ARTERY_MCM_MCLOG_H_
#define ARTERY_MCM_MCLOG_H_

#include <fstream>
#include <string>

namespace artery
{
namespace mcm
{

/**
 * CSV records of the maneuver coordination (thesis Section 5.5).
 *
 * All services of one run share the files <prefix>-<name>.csv; the prefix is a
 * service parameter, usually "${resultdir}/${configname}-${runnumber}".
 * Files are (re-)opened whenever a service asks for a different prefix, e.g.
 * when Cmdenv executes several runs in one process.
 */
class McLog
{
public:
    static McLog& instance();

    /** open the files of a run, no-op if they are already open */
    void open(const std::string& prefix);
    bool isOpen() const { return !mPrefix.empty(); }
    void flush();

    /** one line per station: station id, SUMO id, service type */
    std::ofstream& stations() { return mStations; }
    /** one line per maneuver when it ends */
    std::ofstream& maneuvers() { return mManeuvers; }
    /** one line per transmitted MCM including its trajectory */
    std::ofstream& transmissions() { return mTransmissions; }

private:
    McLog() = default;
    void openFile(std::ofstream&, const std::string& name, const char* header);

    std::string mPrefix;
    std::ofstream mStations;
    std::ofstream mManeuvers;
    std::ofstream mTransmissions;
};

} // namespace mcm
} // namespace artery

#endif /* ARTERY_MCM_MCLOG_H_ */
