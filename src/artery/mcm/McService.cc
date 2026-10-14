#include "artery/mcm/McService.h"

namespace artery
{

Define_Module(McService)

void McService::initialize()
{
    BaseMcService::initialize();
    mAbortProbability = par("abortProbability");
    mAbortWithCancel = par("abortWithCancel");
}

BaseMcService::Execution McService::decideExecution(const Maneuver&)
{
    if (mAbortProbability > 0.0 && uniform(0.0, 1.0) < mAbortProbability) {
        return mAbortWithCancel ? Execution::AbortWithCancel : Execution::AbortSilent;
    }
    return Execution::Drive;
}

} // namespace artery
