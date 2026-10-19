#include "artery/mcm/McService.h"

namespace artery
{

Define_Module(McService)

void McService::initialize()
{
    BaseMcService::initialize();
    mAbortProbability = par("abortProbability");
    mAbortAnnounced = par("abortAnnounced");
}

BaseMcService::Execution McService::decideExecution(const Maneuver& maneuver)
{
    Execution execution;
    // no abort back into an ending lane (merge from an acceleration lane)
    if (maneuver.type != MC_MERGE && mAbortProbability > 0.0 && uniform(0.0, 1.0) < mAbortProbability) {
        const double progress = par("abortProgress").doubleValue();
        execution.announce = mAbortAnnounced;
        if (progress <= 0.0) {
            execution.drive = false;
        } else {
            execution.abortAfter = progress * maneuver.laneChangeDuration;
        }
    }
    return execution;
}

} // namespace artery
