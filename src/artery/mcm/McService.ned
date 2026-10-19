package artery.mcm;

import artery.application.ItsG5Service;

//
// Honest maneuver coordination service, see McService.h.
//
simple McService extends BaseMcService like ItsG5Service
{
    parameters:
        @class(McService);
        // p_abort: probability that a committed lane change is aborted
        volatile double abortProbability = default(0.0);
        // time of the abort as fraction of the lane change duration, drawn per abort;
        // 0 aborts before the lateral movement starts
        volatile double abortProgress = default(uniform(0, 0.5));
        // variant A announces the return with a new reference trajectory, variant B aborts silently
        bool abortAnnounced = default(true);
}
