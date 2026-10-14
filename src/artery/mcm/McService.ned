package artery.mcm;

import artery.application.ItsG5Service;

//
// Honest maneuver coordination service, see McService.h.
//
simple McService extends BaseMcService like ItsG5Service
{
    parameters:
        @class(McService);
        // p_abort: probability that an accepted lane change is aborted
        volatile double abortProbability = default(0.0);
        // variant A sends a cancellation, variant B aborts silently
        bool abortWithCancel = default(true);
}
