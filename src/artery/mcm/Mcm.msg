//
// Simplified Maneuver Coordination Message (MCM), see thesis Section 5.1.2.
//
// Not the ASN.1 format of ETSI TR 103 578: the message travels through Vanetza
// as an opaque OMNeT++ packet and its length on the wire is set explicitly by
// the sending service.
//
// The trajectory is given as offsets east/north of the reference position in the
// basic container, i.e. in a local tangent plane of the sender. No map is needed
// to interpret it: a receiver converts a CAM position into the same plane and
// splits the error along and across the announced trajectory.
//

namespace artery;

enum McManeuverType
{
    MC_KEEP_LANE = 0;
    MC_LANE_CHANGE_LEFT = 1;
    MC_LANE_CHANGE_RIGHT = 2;
    MC_MERGE = 3;
}

enum McPhase
{
    MC_INTENT = 0;
    MC_REQUEST = 1;
    MC_ACCEPT = 2;
    MC_EXECUTE = 3;
}

struct McTrajectoryPoint
{
    uint16_t timeOffset;    // ms after generation time
    int32_t deltaEast;      // cm, east of the reference position
    int32_t deltaNorth;     // cm, north of the reference position
    uint16_t speed;         // 0.01 m/s
    uint16_t heading;       // 0.1 degree, clockwise from north
}

packet McMessage
{
    // ITS PDU header
    uint32_t stationId;
    uint16_t generationDeltaTime;   // ms, TAI modulo 65536 as in the CAM

    // basic container
    int32_t latitude;               // WGS84, 0.1 microdegree
    int32_t longitude;              // WGS84, 0.1 microdegree
    uint16_t heading;               // 0.1 degree, clockwise from north

    // maneuver container
    uint8_t maneuverType @enum(McManeuverType);
    uint8_t phase @enum(McPhase);
    uint16_t maneuverId;            // unique per sender
    uint32_t partnerId;             // 0 if no partner
    uint16_t partnerManeuverId;     // maneuver of the partner this message refers to (accept)
    McTrajectoryPoint points[];     // 250 ms spacing, 5 s horizon

    // cancellation, a message with the flag set carries no trajectory
    bool cancel;
    uint16_t cancelledManeuverId;
}
