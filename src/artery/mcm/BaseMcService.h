#ifndef ARTERY_MCM_BASEMCSERVICE_H_
#define ARTERY_MCM_BASEMCSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/mcm/McGeometry.h"
#include "artery/mcm/McTrajectoryPlanner.h"
#include "artery/utility/Geometry.h"
#include "artery/mcm/Mcm_m.h"
#include <omnetpp/simtime.h>
#include <memory>
#include <string>

namespace traci { class VehicleController; }

namespace artery
{

class LocalDynamicMap;
class Timer;
class VehicleDataProvider;

/**
 * Shared functionality of the maneuver coordination service (thesis Section 5.1.3):
 * intent sharing, two-phase negotiation with request and accept, execution and
 * cancellation, plus the reaction of an accepting partner.
 *
 * Honest and misbehaving variants derive from this class and differ only in the
 * relation between what they announce and what they let the vehicle do
 * (thesis Section 5.3).
 *
 * Lane changes are no longer decided by SUMO itself: autonomous lane changes are
 * switched off via the lane change mode, and every lane change wish of SUMO's lane
 * change model becomes a maneuver of this service, which announces it first and
 * then commands it through TraCI.
 */
class BaseMcService : public ItsG5Service
{
public:
    void initialize() override;
    void finish() override;
    void trigger() override;
    void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
    using omnetpp::cListener::receiveSignal;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, const omnetpp::SimTime&, omnetpp::cObject*) override;

protected:
    enum class State { Idle, Requesting, Accepted, Executing, Partner, Silent };
    enum class Execution { Drive, AbortWithCancel, AbortSilent };

    struct Maneuver
    {
        uint16_t id = 0;
        McManeuverType type = MC_KEEP_LANE;
        const char* role = "requester";
        int direction = 0;                  //< +1 left, -1 right
        uint32_t partnerId = 0;
        uint16_t partnerManeuverId = 0;
        omnetpp::SimTime announced;         //< generation time of the announcing MCM
        omnetpp::SimTime lateralStart;      //< announced start of the lateral movement
        omnetpp::SimTime end;               //< announced end of the maneuver
        std::unique_ptr<McMessage> announcement;
        bool acceptReceived = false;
        bool cancelSent = false;
        // execution: SUMO position and angle when the lane change was commanded
        mcm::Enu commandPosition;
        double commandHeading = 0.0;
        bool started = false;
        // partner role
        double holdSpeed = -1.0;
        omnetpp::SimTime holdFrom;
        bool holding = false;
    };

    /**
     * Decide at the announced start of the lateral movement whether the vehicle
     * actually drives the lane change. The honest service decides by its abort
     * probability; misbehaving variants override this.
     */
    virtual Execution decideExecution(const Maneuver&) { return Execution::Drive; }

    /** message with header and basic container filled from the current vehicle state */
    std::unique_ptr<McMessage> createMessage(McPhase, McManeuverType, uint16_t maneuverId) const;
    void sendMcm(const McMessage&);
    void sendCancel(Maneuver&);

    const McTrajectoryPlanner& planner() const { return *mPlanner; }
    const std::string& vehicleId() const;
    uint32_t stationId() const;
    State state() const { return mState; }

private:
    // own maneuvers
    void checkIntentTriggers(const omnetpp::SimTime& now);
    bool checkLaneChangeWish(const omnetpp::SimTime& now);
    void startLaneChange(int direction, McManeuverType, const omnetpp::SimTime& now);
    void execute(const omnetpp::SimTime& now);
    void checkExecution(const omnetpp::SimTime& now);
    uint32_t findPartner(int direction, double laneWidth) const;
    void sendIntent(const omnetpp::SimTime& now);

    // partner role
    void handleRequest(const McMessage&, const omnetpp::SimTime& now);
    void handleAccept(const McMessage&);
    void handleCancel(const McMessage&);
    void updatePartner(const omnetpp::SimTime& now);

    void finishManeuver(const char* outcome, const omnetpp::SimTime& now);
    void releaseSpeed();
    void resend(const omnetpp::SimTime& now);
    omnetpp::SimTime generationTimeOf(const McMessage&) const;
    bool isDropped(const McMessage&);

    // last announced trajectory as arrow in sumo-gui (TraCI polygons)
    void drawTrajectory(const McMessage&);
    void eraseTrajectory();

    traci::VehicleController* mVehicleController = nullptr;
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    const Timer* mTimer = nullptr;
    const LocalDynamicMap* mLocalDynamicMap = nullptr;
    std::unique_ptr<McTrajectoryPlanner> mPlanner;

    State mState = State::Idle;
    Maneuver mManeuver;
    uint16_t mManeuverCounter = 0;
    omnetpp::SimTime mNextManeuverAllowed;
    omnetpp::SimTime mSilentUntil;
    bool mForceIntent = true;
    bool mStationLogged = false;
    bool mVehicleGone = false;

    // last transmitted MCM (generation rules)
    omnetpp::SimTime mLastSent;
    omnetpp::SimTime mLastIntent;
    Position mLastIntentPosition;
    double mLastIntentHeading = 0.0;
    double mLastIntentSpeed = 0.0;

    // parameters
    omnetpp::SimTime mMinInterval;
    omnetpp::SimTime mMaxInterval;
    double mHeadingDelta;
    double mPositionDelta;
    double mSpeedDelta;
    unsigned mNumPoints;
    double mPointSpacing;
    omnetpp::SimTime mLeadTime;
    omnetpp::SimTime mRequestTimeout;
    omnetpp::SimTime mRequestRepeatInterval;
    omnetpp::SimTime mExecutionRepeatInterval;
    omnetpp::SimTime mManeuverCooldown;
    omnetpp::SimTime mStartTolerance;
    double mLaneChangeDuration;
    int mWishReasons;
    double mPartnerSearchDistance;
    double mMaxAcceptDeceleration;
    double mAcceptMinGap;
    double mAcceptHeadway;
    double mAssumedVehicleLength;
    double mDropProbability;
    unsigned mHeaderBytes;
    unsigned mPointBytes;
    unsigned mTrafficClass;
    bool mLogTransmissions;
    bool mVisualizeTrajectory;
    bool mTrajectoryDrawn = false;
    bool mTraciClosed = false;

    // statistics
    unsigned mSentByPhase[4] = {};
    unsigned mSentCancel = 0;
    unsigned mReceived = 0;
    unsigned mDropped = 0;
    unsigned mDeclinedBusy = 0;
    unsigned mDeclinedLate = 0;
    unsigned mDeclinedDeceleration = 0;
};

} // namespace artery

#endif /* ARTERY_MCM_BASEMCSERVICE_H_ */
