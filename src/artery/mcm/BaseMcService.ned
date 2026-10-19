package artery.mcm;

import artery.application.ItsG5Service;

//
// Maneuver coordination service without legitimate aborts, see BaseMcService.h.
// Honest vehicles normally run McService, which adds the abort probability.
//
// Requires SUMO's sublane model (--lateral-resolution): the service drives the
// lateral movement of lane changes itself. The vehicle types of equipped vehicles
// need maxSpeedLat and lcAccelLat large enough for the shortest lane change.
//
simple BaseMcService like ItsG5Service
{
    parameters:
        @signal[McmSent](type=artery::McMessage);
        @signal[McmReceived](type=artery::McMessage);
        @statistic[mcmSent](source=McmSent; record=count);
        @statistic[mcmReceived](source=McmReceived; record=count);

        // generation: an MCM is sent when the vehicle deviates from its last reference
        // trajectory, the plan changes or maxInterval has passed
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);
        double longitudinalDeviation @unit(m) = default(1.0m);
        double lateralDeviation @unit(m) = default(0.25m);
        double speedDeviation @unit(mps) = default(0.5mps);

        // announced trajectories: 32 points at 250 ms, 8 s horizon (lead time and the longest lane change plus margin)
        int numPoints = default(32);
        double pointSpacing @unit(s) = default(0.25s);

        // own lane changes
        double leadTime @unit(s) = default(1.0s);               // lane keeping before the lateral movement starts
        // drawn per lane change; leadTime + duration is limited to the trajectory horizon
        volatile double laneChangeDuration @unit(s) = default(uniform(3s, 6s));
        string lateralShape = default("quintic");               // linear, sinusoidal or quintic (S-shaped, minimum jerk)
        volatile double returnDuration @unit(s) = default(2s);  // back into the origin lane after an abort
        double lateralLookahead @unit(s) = default(0.1s);       // one update interval: SUMO applies a command in the next step
        double lateralTolerance @unit(m) = default(0.5m);       // lag behind the plan regarded as blocked
        double blockedTimeout @unit(s) = default(0.3s);
        double requestTimeout @unit(s) = default(0.5s);         // must not exceed leadTime
        double requestRepeatInterval @unit(s) = default(0.1s);
        double maneuverCooldown @unit(s) = default(2.0s);       // minimum time between two own manoeuvres
        double wishPersistence @unit(s) = default(0.5s);        // a lane change wish of SUMO has to last this long
        // SUMO lane change mode: no autonomous lane changes; TraCI requests avoid immediate collisions
        int laneChangeMode = default(256);
        // SUMO lane change reasons which become manoeuvres: strategic (8), speed gain (32), keep right (64)
        int wishReasons = default(104);

        // negotiation (ETSI TR 103 578 concept A1)
        // false: base concept, a request is granted implicitly by adapted reference trajectories;
        // true: extension with explicit ACK/NACK per requested trajectory
        bool explicitResponses = default(false);
        double responseLifetime @unit(s) = default(1s);         // an answer is repeated while the request persists
        double neighbourTimeout @unit(s) = default(1.5s);       // trajectories of silent neighbours are discarded
        double requestRange @unit(m) = default(200m);           // vehicles further away are not checked for conflicts
        double checkStep @unit(s) = default(0.25s);             // time resolution of the conflict check
        // conflict: lateral overlap (plus margin) and gap below minGap + headway * speed of the follower
        double conflictMinGap @unit(m) = default(2.5m);
        double conflictHeadway @unit(s) = default(1.0s);
        double conflictLateralMargin @unit(m) = default(0.3m);
        // a yielder checks requests and its own adaptation with gap and headway increased by these
        // margins, so that the adapted plan also satisfies the check of the requester (hysteresis)
        double yieldMarginGap @unit(m) = default(2m);
        double yieldMarginHeadway @unit(s) = default(0.5s);
        // limits of a yielding vehicle
        double maxYieldDeceleration @unit(mpss) = default(3mpss);
        double maxYieldAcceleration @unit(mpss) = default(0mpss);   // 0: yielding only by braking
        double maxYieldSpeedLoss @unit(mps) = default(5mps);       // below the speed of the requester

        // vehicles without MCM are taken from the own sensors if the node is an artery.envmod.Car
        bool usePerception = default(true);

        // p_drop: probability that a received MCM is discarded on top of the channel losses
        volatile double dropProbability = default(0.0);

        // length on the wire: header, basic container and ids, per trajectory point,
        // header of a requested trajectory, per explicit response
        int headerBytes @unit(B) = default(32B);
        int pointBytes @unit(B) = default(14B);
        int requestHeaderBytes @unit(B) = default(3B);
        int responseBytes @unit(B) = default(8B);
        int trafficClass = default(2);

        // CSV records <logPrefix>-*.csv, disabled if empty
        string logPrefix = default("");
        bool logTransmissions = default(true);

        // draw the last announced trajectories of each vehicle as arrows (TraCI polygons), only useful with sumo-gui
        bool visualizeTrajectory = default(false);
}
