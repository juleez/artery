#ifndef ARTERY_MCM_BASEMCSERVICE_H_
#define ARTERY_MCM_BASEMCSERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/mcm/McGeometry.h"
#include "artery/mcm/McTrajectoryPlanner.h"
#include "artery/mcm/Mcm_m.h"
#include <omnetpp/simtime.h>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace traci { class VehicleController; }

namespace artery
{

class LocalEnvironmentModel;
class Timer;
class VehicleDataProvider;

/**
 * Shared functionality of the maneuver coordination service (thesis Section 5.1.3),
 * following concept A1 "agreement seeking" of ETSI TR 103 578 V2.1.1, clause 5.2.2:
 *
 * - every MCM carries the reference trajectory, i.e. the current plan of the sender
 * - a lane change whose planned trajectory is free of conflicts with the known
 *   trajectories of other vehicles becomes the reference trajectory right away
 * - otherwise the vehicle adds it as requested trajectory; vehicles whose reference
 *   trajectory conflicts with the request decide whether they adapt their plan
 *   (yield). In the base concept the adapted reference trajectory is the only answer
 *   and the requester checks for itself whether its request has become conflict free;
 *   with explicitResponses the extension of the TR adds ACK/NACK per requested trajectory.
 *   Hysteresis: decisions use a margin on gap and headway (decisionMargin*): a yielder judges
 *   requests and its adaptation with it, a lane change is driven without negotiation only if it
 *   is free with it; an implicit grant needs half the margin, an abort a conflict by the plain rules
 * - gap selection: before a lane change is announced, the requester tries speed changes
 *   and longer lead times; the first plan free of conflicts is driven without negotiation,
 *   otherwise the plan with the fewest conflicts becomes the requested trajectory
 * - the request is withdrawn when it has not been granted within requestTimeout
 *
 * Lane changes are not decided by SUMO itself: autonomous lane changes are switched
 * off via the lane change mode, every lane change wish of SUMO's lane change model
 * becomes a manoeuvre of this service. The lateral movement is driven by this service
 * through the sublane model (changeSublane in every update), so that its shape and
 * duration follow the announced profile and it can be aborted at any time.
 *
 * Honest and misbehaving variants derive from this class and differ in the relation
 * between what they announce and what they let the vehicle do (thesis Section 5.3),
 * see the virtual hooks below.
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
    enum class State { Idle, Requesting, Committed, Executing, Yielding };

    /** decision about the lateral movement of a committed lane change */
    struct Execution
    {
        bool drive = true;          //< false: the lateral movement is not started at all
        double abortAfter = -1.0;   //< s after the start of the lateral movement, negative: no abort
        bool announce = true;       //< false: silent, no MCM until the last announcement has expired
    };

    struct Maneuver
    {
        uint16_t id = 0;                    //< id of the requested and later the reference trajectory
        McManeuverType type = MC_KEEP_LANE;
        int direction = 0;                  //< +1 left, -1 right
        double originOffset = 0.0;          //< lateral offset from the origin lane centre at the start
        double laneWidth = 0.0;             //< m, distance between origin and target lane centre
        double laneChangeDuration = 0.0;    //< s
        // speed change of the gap selection, driven from the announcement on (0: constant speed)
        double startSpeed = 0.0;            //< m/s when the manoeuvre was announced
        double targetSpeed = 0.0;           //< m/s
        double speedAccel = 0.0;            //< m/s^2, negative for deceleration
        McPlan plan;                        //< lane change plan along the origin lane
        omnetpp::SimTime announced;         //< first MCM with this manoeuvre
        omnetpp::SimTime committed;         //< request granted or no negotiation needed
        bool negotiated = false;            //< sent as requested trajectory
        std::set<uint32_t> affected;        //< vehicles in conflict when the request was sent
        bool affectedUnknown = false;       //< a vehicle without MCM was in conflict when the request was sent
        std::set<uint32_t> lastConflicts;   //< result of the last conflict check, logged when the manoeuvre ends
        bool lastUnknown = false;
        std::set<uint32_t> acks;
        std::set<uint32_t> nacks;
        Execution execution;
        bool executionDecided = false;
        bool aborting = false;
        const char* abortReason = nullptr;
        omnetpp::SimTime blockedSince;      //< lateral movement lags behind the plan since
    };

    /**
     * Decide at the announced start of the lateral movement whether and how far the
     * vehicle drives the lane change. The honest service aborts with its abort
     * probability; misbehaving variants override this.
     */
    virtual Execution decideExecution(const Maneuver&) { return Execution(); }

    /** last chance to modify an MCM before it is sent (e.g. falsified trajectories) */
    virtual void beforeSend(McMessage&) {}

    const McTrajectoryPlanner& planner() const { return *mPlanner; }
    const std::string& vehicleId() const;
    uint32_t stationId() const;
    State state() const { return mState; }
    const Maneuver& maneuver() const { return mManeuver; }

private:
    struct Neighbour
    {
        std::unique_ptr<McMessage> msg;
        omnetpp::SimTime generated;
        omnetpp::SimTime received;
        omnetpp::SimTime requestSeen;   //< first reception of the current requested trajectory
    };

    struct Yield
    {
        uint32_t requester = 0;
        uint16_t requestId = 0;
        omnetpp::SimTime since;
        omnetpp::SimTime until;
        double startSpeed = 0.0;
        double targetSpeed = 0.0;
        double deceleration = 0.0;  //< signed: negative brakes, positive accelerates
        bool holding = false;
        bool executionSeen = false; //< the request has become the requester's reference trajectory
    };

    struct PendingResponse
    {
        McResponse response;
        omnetpp::SimTime until;
    };

    // own manoeuvres
    void checkLaneChangeWish(const omnetpp::SimTime& now);
    void startManeuver(int direction, McManeuverType, const omnetpp::SimTime& now);
    void updateRequest(const omnetpp::SimTime& now);
    void commit(const omnetpp::SimTime& now);
    void updateCommitted(const omnetpp::SimTime& now);
    void updateExecution(const omnetpp::SimTime& now);
    void beginReturn(const char* reason, bool announce, const omnetpp::SimTime& now);
    McLateralProfile returnProfile(const omnetpp::SimTime& now) const;
    void finishManeuver(const char* outcome, const omnetpp::SimTime& now);

    // partner role
    void handleRequest(const Neighbour&, const omnetpp::SimTime& now);
    /** an accelerating yield plan conflicts with nobody but the requester */
    bool freeAhead(const Neighbour& requester, const McPlan& trial, const omnetpp::SimTime& now) const;
    void updateYield(const omnetpp::SimTime& now);
    void releaseYield(const char* outcome, const omnetpp::SimTime& now);
    void respond(uint32_t requester, uint16_t requestId, bool accept, McResponseReason, const omnetpp::SimTime& now);
    void logRequestDecision(const Neighbour&, int state, const char* decision, const Yield*, const omnetpp::SimTime& now) const;

    // conflicts
    std::vector<double> checkTimes(const omnetpp::SimTime& now, double until) const;
    mcm::Body neighbourBody(const Neighbour&, const mcm::McPoints&, const mcm::Geo& origin, double t) const;
    /**
     * vehicles whose reference trajectory conflicts with the plan but not with the keep lane plan;
     * both are judged with rules (default: the plain conflict rules)
     */
    std::set<uint32_t> conflictsOf(const McPlan& plan, const McPlan& keep, const omnetpp::SimTime& now,
            const omnetpp::SimTime& requestTime, bool& blockedByUnknown,
            const mcm::ConflictRules* rules = nullptr) const;
    /** true if a requested trajectory of another vehicle conflicts with the own plan */
    /** the requested trajectory of n conflicts with own, judged with the stricter rules of the yielder */
    bool affectedBy(const Neighbour&, const McPlan& own, const omnetpp::SimTime& now) const;
    bool findYieldPlan(const Neighbour&, const omnetpp::SimTime& now, Yield&, McPlan&) const;

    // reference trajectory and transmission
    McPlan keepLanePlan(const omnetpp::SimTime& now) const;
    void setReference(const McPlan&, McManeuverType, const omnetpp::SimTime& now, uint16_t id = 0);
    uint16_t nextTrajectoryId();
    void refreshReference(const omnetpp::SimTime& now);
    bool deviatesFromReference(const omnetpp::SimTime& now) const;
    void transmit(const omnetpp::SimTime& now);
    std::unique_ptr<McMessage> createMessage(const omnetpp::SimTime& now);
    void sendMcm(McMessage&);
    void controlLateral(const omnetpp::SimTime& now);
    /** remaining speed change of the own manoeuvre, starting at the given speed */
    McLongitudinalProfile maneuverLongitudinal(double speed) const;
    /** lane keeping with the remaining speed change of the own manoeuvre: alternative of the lane change */
    McPlan maneuverKeepPlan(const omnetpp::SimTime& now) const;
    /** highest speed a manoeuvre may accelerate to: lane speed limit, at least the own desired speed */
    double accelerationLimit() const;
    void controlLongitudinal(const omnetpp::SimTime& now);
    void holdSpeed(double speed);
    void releaseSpeed();

    omnetpp::SimTime generationTimeOf(const McMessage&) const;
    void expireNeighbours(const omnetpp::SimTime& now);

    // last announced trajectories as arrows in sumo-gui (TraCI polygons)
    void drawTrajectory(const std::string& suffix, const mcm::McPoints&, const McMessage&, const libsumo::TraCIColor&, double width);
    void eraseTrajectory(const std::string& suffix);

    traci::VehicleController* mVehicleController = nullptr;
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    const Timer* mTimer = nullptr;
    const LocalEnvironmentModel* mEnvironmentModel = nullptr;
    std::unique_ptr<McTrajectoryPlanner> mPlanner;

    State mState = State::Idle;
    Maneuver mManeuver;
    Yield mYield;
    uint16_t mTrajectoryCounter = 0;
    omnetpp::SimTime mNextManeuverAllowed;
    omnetpp::SimTime mSilentUntil;
    int mWishDirection = 0;
    omnetpp::SimTime mWishSince;
    double mLength = 5.0;
    double mWidth = 1.8;

    // reference trajectory, anchored at the last transmission
    McPlan mReference;
    uint16_t mReferenceId = 0;
    McManeuverType mReferenceType = MC_KEEP_LANE;
    bool mSendNow = true;
    omnetpp::SimTime mLastSent;

    std::map<uint32_t, Neighbour> mNeighbours;
    std::map<std::pair<uint32_t, uint16_t>, PendingResponse> mResponses;
    std::set<std::pair<uint32_t, uint16_t>> mEvaluatedRequests;

    bool mStationLogged = false;
    bool mVehicleGone = false;
    bool mTraciClosed = false;
    std::set<std::string> mDrawn;

    // parameters
    omnetpp::SimTime mMinInterval;
    omnetpp::SimTime mMaxInterval;
    double mLongitudinalDeviation;
    double mLateralDeviation;
    double mSpeedDeviation;
    unsigned mNumPoints;
    double mPointSpacing;
    omnetpp::SimTime mLeadTime;
    omnetpp::SimTime mRequestTimeout;
    omnetpp::SimTime mRequestRepeatInterval;
    omnetpp::SimTime mManeuverCooldown;
    omnetpp::SimTime mWishPersistence;
    McLateralProfile::Shape mLateralShape;
    double mLateralLookahead;
    double mLateralTolerance;
    omnetpp::SimTime mBlockedTimeout;
    double mRequesterAcceleration;
    double mRequesterDeceleration;
    double mMaxRequesterSpeedChange;
    double mRequesterSpeedStep;
    double mMaxExtraLeadTime;
    int mWishReasons;
    bool mExplicitResponses;
    omnetpp::SimTime mResponseLifetime;
    omnetpp::SimTime mNeighbourTimeout;
    double mCheckStep;
    double mRequestRange;
    mcm::ConflictRules mConflictRules;
    mcm::ConflictRules mDecisionRules;  //< mConflictRules plus the decision margin
    mcm::ConflictRules mGrantRules;     //< mConflictRules plus half the decision margin
    double mMaxYieldDeceleration;
    double mMaxYieldAcceleration;
    double mMaxYieldSpeedLoss;
    bool mUsePerception;
    double mDropProbability;
    unsigned mHeaderBytes;
    unsigned mPointBytes;
    unsigned mRequestHeaderBytes;
    unsigned mResponseBytes;
    unsigned mTrafficClass;
    bool mLogTransmissions;
    bool mVisualizeTrajectory;

    // statistics
    unsigned mSent = 0;
    unsigned mSentWithRequest = 0;
    unsigned mSentResponses = 0;
    unsigned mReceived = 0;
    unsigned mDropped = 0;
    unsigned mRequestsAccepted = 0;
    unsigned mRequestsDeclinedBusy = 0;
    unsigned mRequestsDeclinedCost = 0;
};

} // namespace artery

#endif /* ARTERY_MCM_BASEMCSERVICE_H_ */
