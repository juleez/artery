package artery.mcm;

import artery.application.ItsG5Service;

//
// Honest maneuver coordination service of a vehicle which never adapts its plan
// for requested trajectories of others, see NonCooperativeMcService.h.
//
simple NonCooperativeMcService extends McService like ItsG5Service
{
    parameters:
        @class(NonCooperativeMcService);
}
