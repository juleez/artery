package artery.mcm;

import artery.application.ItsG5Service;

//
// Maneuver coordination service without legitimate aborts, see BaseMcService.h.
// Honest vehicles normally run McService, which adds the abort probability.
//
simple BaseMcService like ItsG5Service
{
    parameters:
        @signal[McmSent](type=artery::McMessage);
        @signal[McmReceived](type=artery::McMessage);
        @statistic[mcmSent](source=McmSent; record=count);
        @statistic[mcmReceived](source=McmReceived; record=count);

        // intent sharing (keep lane): generation rules as for the CAM
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);
        double headingDelta = default(4.0); // degree
        double positionDelta @unit(m) = default(4.0m);
        double speedDelta @unit(mps) = default(0.5mps);

        // announced trajectory: 20 points at 250 ms, 5 s horizon
        int numPoints = default(20);
        double pointSpacing @unit(s) = default(0.25s);

        // negotiation and execution
        double leadTime @unit(s) = default(1.0s);               // lane keeping before the lateral movement starts
        double requestTimeout @unit(s) = default(0.5s);         // must not exceed leadTime
        double requestRepeatInterval @unit(s) = default(0.1s);
        double executionRepeatInterval @unit(s) = default(1.0s);
        double maneuverCooldown @unit(s) = default(2.0s);       // minimum time between two own maneuvers
        double startTolerance @unit(s) = default(0.5s);         // SUMO has to start the lane change within this time
        double laneChangeDuration @unit(s) = default(3s);       // has to match SUMO's lanechange.duration
        // SUMO lane change mode: no autonomous lane changes; TraCI requests avoid immediate collisions
        int laneChangeMode = default(256);
        // SUMO lane change reasons which become maneuvers: strategic (8), speed gain (32), keep right (64)
        int wishReasons = default(104);

        // partner selection and acceptance
        double partnerSearchDistance @unit(m) = default(100m);
        double maxAcceptDeceleration @unit(mpss) = default(3mpss);
        double acceptMinGap @unit(m) = default(2.5m);
        double acceptHeadway @unit(s) = default(1.0s);
        double assumedVehicleLength @unit(m) = default(5m);     // CAMs in Artery carry no vehicle length

        // p_drop: probability that a received accept or cancellation is discarded
        volatile double dropProbability = default(0.0);

        // length on the wire: header + basic + maneuver container, plus per trajectory point
        int headerBytes @unit(B) = default(28B);
        int pointBytes @unit(B) = default(14B);
        int trafficClass = default(2);

        // CSV records <logPrefix>-*.csv, disabled if empty
        string logPrefix = default("");
        bool logTransmissions = default(true);
}
