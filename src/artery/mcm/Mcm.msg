//
// Simplified Maneuver Coordination Message (MCM), see thesis Section 5.1.2.
//
// Structure follows concept A1 ("agreement seeking") of ETSI TR 103 578 V2.1.1,
// clause 5.2.2: every MCM carries the reference trajectory of the sender, i.e.
// the trajectory it currently plans to drive. A vehicle which needs the help of
// others to drive a manoeuvre adds a requested trajectory. Receivers whose own
// reference trajectory conflicts with the request decide whether they cooperate;
// the base concept answers implicitly with an adapted reference trajectory, the
// extension of the TR adds explicit responses (ACK/NACK) per requested trajectory.
//
// Not the ASN.1 format of the TR: the message travels through Vanetza as an
// opaque OMNeT++ packet and its length on the wire is set explicitly by the
// sending service.
//
// Trajectories are given as offsets east/north of the reference position in the
// basic container, i.e. in a local tangent plane of the sender. No map is needed
// to interpret them: a receiver converts a CAM position into the same plane and
// splits the error along and across the announced trajectory.
//

namespace artery;

enum McManeuverType
{
    MC_KEEP_LANE = 0;
    MC_LANE_CHANGE_LEFT = 1;
    MC_LANE_CHANGE_RIGHT = 2;
    MC_MERGE = 3;
    MC_YIELD = 4;   // longitudinal adaptation to give way to a requested trajectory
}

enum McResponseReason
{
    MC_RESPONSE_ACCEPT = 0;
    MC_RESPONSE_BUSY = 1;           // own manoeuvre or already cooperating
    MC_RESPONSE_COST = 2;           // required adaptation exceeds own limits
    MC_RESPONSE_NOT_COOPERATIVE = 3;
}

struct McTrajectoryPoint
{
    uint16_t timeOffset;    // ms after generation time
    int32_t deltaEast;      // cm, east of the reference position
    int32_t deltaNorth;     // cm, north of the reference position
    uint16_t speed;         // 0.01 m/s
    uint16_t heading;       // 0.1 degree, clockwise from north
}

// explicit answer to a requested trajectory (TR 103 578 extension)
struct McResponse
{
    uint32_t stationId;     // sender of the requested trajectory
    uint16_t trajectoryId;  // id of the requested trajectory
    bool accept;
    uint8_t reason @enum(McResponseReason);
}

packet McMessage
{
    // ITS PDU header
    uint32_t stationId;
    uint16_t generationDeltaTime;   // ms, TAI modulo 65536 as in the CAM

    // basic container, reference point is the centre of the front bumper as in the CAM
    int32_t latitude;               // WGS84, 0.1 microdegree
    int32_t longitude;              // WGS84, 0.1 microdegree
    uint16_t heading;               // 0.1 degree, clockwise from north
    uint16_t speed;                 // 0.01 m/s
    uint16_t vehicleLength;         // 0.1 m
    uint16_t vehicleWidth;          // 0.1 m

    // reference trajectory: what the sender plans to drive
    uint16_t referenceId;           // changes with every new plan (unique per sender)
    uint8_t referenceType @enum(McManeuverType);
    McTrajectoryPoint reference[];  // 250 ms spacing, 5 s horizon

    // requested trajectory, only present if requestedId != 0
    uint16_t requestedId;
    uint8_t requestedType @enum(McManeuverType);
    McTrajectoryPoint requested[];

    // explicit responses to requested trajectories of other vehicles
    McResponse responses[];
}
