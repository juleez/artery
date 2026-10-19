#include "artery/mcm/BaseMcService.h"
#include "artery/mcm/McGeometry.h"
#include "artery/mcm/McLog.h"
#include "artery/application/Timer.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"
#include "traci/API.h"
#include <omnetpp/checkandcast.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/common/byte_order.hpp>
#include <vanetza/geonet/interface.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>

#ifdef WITH_ENVMOD
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/TraCIEnvironmentModelObject.h"
#endif

namespace artery
{

using namespace omnetpp;
using mcm::Body;
using mcm::Enu;
using mcm::Geo;
using mcm::McPoints;

Define_Module(BaseMcService)

namespace
{

const simsignal_t scSignalMcmSent = cComponent::registerSignal("McmSent");
const simsignal_t scSignalMcmReceived = cComponent::registerSignal("McmReceived");

// lane change actions of SUMO's lane change model (libsumo::LCA_*)
constexpr int LCA_LEFT = 1 << 1;
constexpr int LCA_RIGHT = 1 << 2;
constexpr int LCA_STRATEGIC = 1 << 3;

constexpr double pi = 3.14159265358979323846;

double headingDegrees(const VehicleDataProvider& vdp)
{
    double deg = vdp.heading().value() * 180.0 / pi;
    deg = std::fmod(deg, 360.0);
    return deg < 0.0 ? deg + 360.0 : deg;
}

Geo geoOf(const VehicleDataProvider& vdp)
{
    return Geo { vdp.latitude().value(), vdp.longitude().value() };
}

const char* stateName(int state)
{
    static const char* names[] = { "idle", "requesting", "committed", "executing", "yielding" };
    return state >= 0 && state < 5 ? names[state] : "?";
}

std::string joinIds(const std::set<uint32_t>& ids)
{
    std::ostringstream out;
    for (auto it = ids.begin(); it != ids.end(); ++it) {
        out << (it == ids.begin() ? "" : ";") << *it;
    }
    return out.str();
}

void writePoints(std::ostream& out, const McPoints& points)
{
    for (unsigned i = 0; i < points.size(); ++i) {
        const McTrajectoryPoint& p = points[i];
        out << (i ? ";" : "") << p.timeOffset << ":" << p.deltaEast << ":" << p.deltaNorth
            << ":" << p.speed << ":" << p.heading;
    }
}

} // namespace

void BaseMcService::initialize()
{
    ItsG5Service::initialize();
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    mPlanner.reset(new McTrajectoryPlanner(*mVehicleController->getTraCI(), mVehicleController->getVehicleId()));

    mMinInterval = par("minInterval");
    mMaxInterval = par("maxInterval");
    mLongitudinalDeviation = par("longitudinalDeviation");
    mLateralDeviation = par("lateralDeviation");
    mSpeedDeviation = par("speedDeviation");
    mNumPoints = par("numPoints");
    mPointSpacing = par("pointSpacing");
    mLeadTime = par("leadTime");
    mRequestTimeout = par("requestTimeout");
    mRequestRepeatInterval = par("requestRepeatInterval");
    mManeuverCooldown = par("maneuverCooldown");
    mWishPersistence = par("wishPersistence");
    mLateralShape = McLateralProfile::parseShape(par("lateralShape").stdstringValue());
    mLateralLookahead = par("lateralLookahead");
    mLateralTolerance = par("lateralTolerance");
    mBlockedTimeout = par("blockedTimeout");
    mWishReasons = par("wishReasons");
    mExplicitResponses = par("explicitResponses");
    mResponseLifetime = par("responseLifetime");
    mNeighbourTimeout = par("neighbourTimeout");
    mCheckStep = par("checkStep");
    mRequestRange = par("requestRange").doubleValue();
    mConflictRules.minGap = par("conflictMinGap");
    mConflictRules.headway = par("conflictHeadway");
    mConflictRules.lateralMargin = par("conflictLateralMargin");
    mYieldRules = mConflictRules;
    mYieldRules.minGap += par("yieldMarginGap").doubleValue();
    mYieldRules.headway += par("yieldMarginHeadway").doubleValue();
    mMaxYieldDeceleration = par("maxYieldDeceleration");
    mMaxYieldAcceleration = par("maxYieldAcceleration");
    mMaxYieldSpeedLoss = par("maxYieldSpeedLoss");
    mUsePerception = par("usePerception");
    mDropProbability = par("dropProbability");
    mHeaderBytes = par("headerBytes");
    mPointBytes = par("pointBytes");
    mRequestHeaderBytes = par("requestHeaderBytes");
    mResponseBytes = par("responseBytes");
    mTrafficClass = par("trafficClass");
    mLogTransmissions = par("logTransmissions");
    mVisualizeTrajectory = par("visualizeTrajectory");
    if (mVisualizeTrajectory) {
        // polygons must not be removed any more once the TraCI connection is closed
        getSystemModule()->subscribe("traci.close", this);
    }

    if (mRequestTimeout > mLeadTime) {
        throw cRuntimeError("requestTimeout must not exceed leadTime (thesis Section 4.1)");
    }
    if (mLeadTime >= SimTime(mNumPoints * mPointSpacing)) {
        throw cRuntimeError("the lead time has to be shorter than the trajectory horizon");
    }

#ifdef WITH_ENVMOD
    if (mUsePerception) {
        // only present if the vehicle is equipped with sensors (artery.envmod.Car)
        mEnvironmentModel = getFacilities().get_const_ptr<LocalEnvironmentModel>();
    }
#endif

    auto& vehicle = mVehicleController->getTraCI()->vehicle;
    // lane changes are commanded by this service only
    vehicle.setLaneChangeMode(vehicleId(), par("laneChangeMode").intValue());
    mLength = vehicle.getLength(vehicleId());
    mWidth = vehicle.getWidth(vehicleId());

    // give the neighbour table time to fill before the first manoeuvre
    mNextManeuverAllowed = simTime() + mManeuverCooldown;

    const std::string prefix = par("logPrefix").stdstringValue();
    if (!prefix.empty()) {
        mcm::McLog::instance().open(prefix);
    }
}

void BaseMcService::finish()
{
    // the vehicle may already have left SUMO, so do not touch it any more
    mVehicleGone = true;
    if (mManeuver.id != 0) {
        finishManeuver("unfinished", simTime());
    } else if (mState == State::Yielding) {
        releaseYield("unfinished", simTime());
    }
    if (mVisualizeTrajectory) {
        eraseTrajectory("ref");
        eraseTrajectory("req");
        getSystemModule()->unsubscribe("traci.close", this);
    }
    recordScalar("mcmSent", mSent);
    recordScalar("mcmSentWithRequest", mSentWithRequest);
    recordScalar("mcmSentResponses", mSentResponses);
    recordScalar("mcmReceived", mReceived);
    recordScalar("mcmDropped", mDropped);
    recordScalar("requestsAccepted", mRequestsAccepted);
    recordScalar("requestsDeclinedBusy", mRequestsDeclinedBusy);
    recordScalar("requestsDeclinedCost", mRequestsDeclinedCost);
    if (mcm::McLog::instance().isOpen()) {
        mcm::McLog::instance().flush();
    }
    ItsG5Service::finish();
}

const std::string& BaseMcService::vehicleId() const
{
    return mVehicleController->getVehicleId();
}

uint32_t BaseMcService::stationId() const
{
    return mVehicleDataProvider->getStationId();
}

void BaseMcService::trigger()
{
    Enter_Method("trigger");
    const SimTime now = simTime();

    if (!mStationLogged && mcm::McLog::instance().isOpen()) {
        // station id is assigned after initialization of the services
        mcm::McLog::instance().stations() << stationId() << "," << vehicleId() << "," << getNedTypeName() << std::endl;
        mStationLogged = true;
    }

    expireNeighbours(now);

    switch (mState) {
        case State::Idle:
            if (now >= mNextManeuverAllowed) {
                checkLaneChangeWish(now);
            }
            break;
        case State::Requesting:
            updateRequest(now);
            break;
        case State::Committed:
            updateCommitted(now);
            break;
        case State::Executing:
            updateExecution(now);
            break;
        case State::Yielding:
            updateYield(now);
            break;
    }

    transmit(now);
}

// --- own manoeuvres -------------------------------------------------------

void BaseMcService::checkLaneChangeWish(const SimTime& now)
{
    const auto lane = mPlanner->laneState();
    if (lane.internal) {
        mWishDirection = 0;
        return;
    }

    auto& vehicle = mVehicleController->getTraCI()->vehicle;
    int chosen = 0;
    int chosenState = 0;
    for (int direction : { +1, -1 }) {
        // first: state computed by the lane change model, regardless of the lane change mode
        const int state = vehicle.getLaneChangeState(vehicleId(), direction).first;
        const int want = direction > 0 ? LCA_LEFT : LCA_RIGHT;
        if (!(state & want) || !(state & mWishReasons)) {
            continue;
        }
        if (mPlanner->neighbourLane(lane, direction).empty()) {
            continue;
        }
        if (!chosen || ((state & LCA_STRATEGIC) && !(chosenState & LCA_STRATEGIC))) {
            chosen = direction;
            chosenState = state;
        }
    }

    // SUMO's wishes, in particular speed gain, flicker: a wish has to persist
    if (chosen != mWishDirection) {
        mWishDirection = chosen;
        mWishSince = now;
    }
    if (!chosen || now - mWishSince < mWishPersistence) {
        return;
    }

    McManeuverType type = chosen > 0 ? MC_LANE_CHANGE_LEFT : MC_LANE_CHANGE_RIGHT;
    if ((chosenState & LCA_STRATEGIC) && mPlanner->laneEnds(lane)) {
        type = MC_MERGE;
    }
    mWishDirection = 0;
    startManeuver(chosen, type, now);
}

void BaseMcService::startManeuver(int direction, McManeuverType type, const SimTime& now)
{
    const auto lane = mPlanner->laneState();
    const std::string target = mPlanner->neighbourLane(lane, direction);
    if (target.empty()) {
        return;
    }
    const double width = 0.5 * (mPlanner->laneWidth(lane.laneId) + mPlanner->laneWidth(target));
    const double horizon = mNumPoints * mPointSpacing;

    Maneuver m;
    m.id = nextTrajectoryId();
    m.type = type;
    m.direction = direction;
    // drawn per manoeuvre, it has to fit into the trajectory horizon together with the lead time
    m.laneChangeDuration = std::min(par("laneChangeDuration").doubleValue(), horizon - mLeadTime.dbl());

    const double speed = mPlanner->vehicleSpeed();
    // the path has to cover the manoeuvre and the horizon of the last announcement during it
    const double pathLength = std::max(speed, 10.0) * (mLeadTime.dbl() + m.laneChangeDuration + horizon) + 50.0;
    m.plan = mPlanner->planLane(lane, now.dbl(), pathLength);
    m.plan.type = type;
    m.originOffset = m.plan.lat.d0;
    m.laneWidth = width;
    m.plan.lat.delta = direction * width - m.plan.lat.d0;
    m.plan.lat.start = (now + mLeadTime).dbl();
    m.plan.lat.duration = m.laneChangeDuration;
    m.plan.lat.shape = mLateralShape;
    m.announced = now;

    bool blockedByUnknown = false;
    m.affected = conflictsOf(m.plan, keepLanePlan(now), now, now, blockedByUnknown);
    m.affectedUnknown = blockedByUnknown;
    m.lastConflicts = m.affected;
    m.lastUnknown = blockedByUnknown;
    mManeuver = std::move(m);

    // the announced constant speed is held until the lane change is completed (thesis Section 5.2)
    holdSpeed(speed);

    if (mManeuver.affected.empty() && !blockedByUnknown) {
        // free of conflicts: no cooperation needed, the lane change becomes the reference trajectory
        commit(now);
    } else {
        mManeuver.negotiated = true;
        mState = State::Requesting;
        mSendNow = true;
    }
}

void BaseMcService::updateRequest(const SimTime& now)
{
    if (mExplicitResponses && !mManeuver.nacks.empty()) {
        // explicit decline: no need to wait for the timeout
        finishManeuver("declined", now);
        return;
    }

    bool blockedByUnknown = false;
    const auto conflicts = conflictsOf(mManeuver.plan, keepLanePlan(now), now, mManeuver.announced, blockedByUnknown);
    mManeuver.lastConflicts = conflicts;
    mManeuver.lastUnknown = blockedByUnknown;
    bool granted = conflicts.empty() && !blockedByUnknown;
    if (granted && mExplicitResponses) {
        // every vehicle in conflict with the request has to agree, unless it has left the neighbourhood
        for (uint32_t id : mManeuver.affected) {
            if (mNeighbours.count(id) && !mManeuver.acks.count(id)) {
                granted = false;
                break;
            }
        }
    }

    if (granted) {
        commit(now);
    } else if (now - mManeuver.announced >= mRequestTimeout) {
        // withdrawn: the next MCM carries no requested trajectory any more
        finishManeuver("timeout", now);
    }
}

void BaseMcService::commit(const SimTime& now)
{
    mManeuver.committed = now;
    mState = State::Committed;
    // the requested trajectory becomes the reference trajectory, keeping its id
    setReference(mManeuver.plan, mManeuver.type, now, mManeuver.id);
}

void BaseMcService::updateCommitted(const SimTime& now)
{
    if (now.dbl() + mLateralLookahead < mManeuver.plan.lat.start) {
        return;
    }

    if (!mManeuver.executionDecided) {
        mManeuver.execution = decideExecution(mManeuver);
        mManeuver.executionDecided = true;
    }

    if (!mManeuver.execution.drive) {
        if (mManeuver.execution.announce) {
            finishManeuver("aborted_announced", now);
        } else {
            mSilentUntil = mLastSent + SimTime(mNumPoints * mPointSpacing);
            finishManeuver("aborted_silent", now);
        }
        return;
    }

    // last check before the lateral movement, as with the on-board sensors in TR 103 578 clause 5.2.3
    bool blockedByUnknown = false;
    const auto conflicts = conflictsOf(mManeuver.plan, keepLanePlan(now), now, mManeuver.announced, blockedByUnknown);
    mManeuver.lastConflicts = conflicts;
    mManeuver.lastUnknown = blockedByUnknown;
    if (!conflicts.empty() || blockedByUnknown) {
        finishManeuver("failed_gap", now);
        return;
    }

    mState = State::Executing;
    updateExecution(now);
}

void BaseMcService::updateExecution(const SimTime& now)
{
    const Execution& execution = mManeuver.execution;
    if (!mManeuver.aborting && execution.abortAfter >= 0.0 &&
            now.dbl() >= mManeuver.plan.lat.start + execution.abortAfter) {
        beginReturn(execution.announce ? "aborted_announced" : "aborted_silent", execution.announce, now);
    }

    const double actual = mPlanner->project(mManeuver.plan, mPlanner->vehiclePosition()).lateral;

    // monitoring until the lane boundary is crossed: predictions of others may have been wrong
    // (acceleration, own lane changes of vehicles without MCM); return while that is still possible
    const bool beforeBoundary = mManeuver.direction * (actual - mManeuver.originOffset) < 0.5 * mManeuver.laneWidth;
    if (!mManeuver.aborting && beforeBoundary) {
        McPlan back = mManeuver.plan;
        back.lat = returnProfile(now);
        bool blockedByUnknown = false;
        const auto conflicts = conflictsOf(mManeuver.plan, back, now, mManeuver.announced, blockedByUnknown);
        if (!conflicts.empty() || blockedByUnknown) {
            mManeuver.lastConflicts = conflicts;
            mManeuver.lastUnknown = blockedByUnknown;
            beginReturn("failed_conflict", true, now);
        }
    }

    // SUMO refuses lateral movements which would cause a collision: detect a lane change which lags behind
    const double planned = mManeuver.plan.lat.offset(now.dbl());
    if (std::abs(actual - planned) > mLateralTolerance) {
        if (mManeuver.blockedSince == SimTime::ZERO) {
            mManeuver.blockedSince = now;
        } else if (!mManeuver.aborting && now - mManeuver.blockedSince >= mBlockedTimeout) {
            beginReturn("failed_blocked", true, now);
        }
    } else {
        mManeuver.blockedSince = SimTime::ZERO;
    }

    if (now.dbl() >= mManeuver.plan.lat.end() + mLateralLookahead) {
        finishManeuver(mManeuver.aborting ? mManeuver.abortReason : "executed", now);
    } else {
        controlLateral(now);
    }
}

McLateralProfile BaseMcService::returnProfile(const SimTime& now) const
{
    McLateralProfile back;
    back.d0 = mPlanner->project(mManeuver.plan, mPlanner->vehiclePosition()).lateral;
    back.delta = mManeuver.originOffset - back.d0;
    back.start = now.dbl();
    // the duration of the return is fixed when it begins, see beginReturn
    back.duration = 2.0;
    back.shape = mLateralShape;
    return back;
}

void BaseMcService::beginReturn(const char* reason, bool announce, const SimTime& now)
{
    McLateralProfile back = returnProfile(now);
    back.duration = std::max(0.5, par("returnDuration").doubleValue());
    mManeuver.plan.lat = back;
    mManeuver.aborting = true;
    mManeuver.abortReason = reason;
    mManeuver.blockedSince = SimTime::ZERO;

    if (announce) {
        // the new reference trajectory leads back into the origin lane
        mManeuver.plan.type = MC_KEEP_LANE;
        setReference(mManeuver.plan, MC_KEEP_LANE, now);
    } else {
        // silent: no MCM until the last announced trajectory has expired
        mSilentUntil = std::max(mSilentUntil, mLastSent + SimTime(mNumPoints * mPointSpacing));
    }
}

void BaseMcService::finishManeuver(const char* outcome, const SimTime& now)
{
    auto& log = mcm::McLog::instance();
    if (log.isOpen()) {
        log.maneuvers() << stationId() << "," << vehicleId() << "," << mManeuver.id << ",requester,"
            << static_cast<int>(mManeuver.type) << "," << joinIds(mManeuver.affected) << ","
            << mManeuver.announced.dbl() << "," << (mManeuver.committed == SimTime::ZERO ? -1.0 : mManeuver.committed.dbl()) << ","
            << mManeuver.plan.lat.start << "," << now.dbl() << "," << outcome << ","
            << mManeuver.negotiated << "," << joinIds(mManeuver.acks) << "," << joinIds(mManeuver.nacks) << ","
            << mManeuver.laneChangeDuration << "," << mManeuver.execution.abortAfter << ","
            << mManeuver.affectedUnknown << "," << joinIds(mManeuver.lastConflicts) << "," << mManeuver.lastUnknown << std::endl;
    }

    mManeuver = Maneuver();
    mState = State::Idle;
    mNextManeuverAllowed = now + mManeuverCooldown;
    mWishDirection = 0;
    if (!mVehicleGone) {
        releaseSpeed();
        setReference(keepLanePlan(now), MC_KEEP_LANE, now);
    }
}

// --- partner role -----------------------------------------------------------

void BaseMcService::handleRequest(const Neighbour& n, const SimTime& now)
{
    const uint32_t requester = n.msg->getStationId();
    const uint16_t requestId = n.msg->getRequestedId();
    const auto key = std::make_pair(requester, requestId);

    if (mState == State::Yielding && mYield.requester == requester && mYield.requestId == requestId) {
        return; // already cooperating
    }
    if (mEvaluatedRequests.count(key)) {
        // repetition of a request already answered: keep an explicit answer alive
        auto found = mResponses.find(key);
        if (found != mResponses.end()) {
            found->second.until = now + mResponseLifetime;
        }
        return;
    }

    // cheap distance check first, the conflict check needs the own plan from SUMO
    const Enu offset = mcm::toEnu(geoOf(*mVehicleDataProvider), mcm::referenceOf(*n.msg));
    if (mcm::length(offset) > mRequestRange) {
        return;
    }
    if (mReference.empty() || !affectedBy(n, mReference, now)) {
        return; // no conflict with the own plan (yet), evaluated again with the next repetition
    }
    mEvaluatedRequests.insert(key);
    const int state = static_cast<int>(mState);

    if (mState != State::Idle) {
        ++mRequestsDeclinedBusy;
        logRequestDecision(n, state, "busy", nullptr, now);
        respond(requester, requestId, false, MC_RESPONSE_BUSY, now);
        return;
    }

    Yield yield;
    McPlan plan;
    if (!findYieldPlan(n, now, yield, plan)) {
        ++mRequestsDeclinedCost;
        logRequestDecision(n, state, "cost", nullptr, now);
        respond(requester, requestId, false, MC_RESPONSE_COST, now);
        return;
    }

    ++mRequestsAccepted;
    logRequestDecision(n, state, "accepted", &yield, now);
    const double speed = mPlanner->vehicleSpeed();
    mYield = yield;
    mYield.startSpeed = speed;
    mState = State::Yielding;
    setReference(plan, MC_YIELD, now);
    respond(requester, requestId, true, MC_RESPONSE_ACCEPT, now);

    if (std::abs(yield.deceleration) > 0.0 && std::abs(speed - yield.targetSpeed) > 0.05) {
        // slowDown changes the speed linearly over the given duration, in both directions
        const double duration = std::abs(speed - yield.targetSpeed) / std::abs(yield.deceleration);
        mVehicleController->getTraCI()->vehicle.slowDown(vehicleId(), yield.targetSpeed, duration);
    } else {
        holdSpeed(yield.targetSpeed);
        mYield.holding = true;
    }
}

bool BaseMcService::findYieldPlan(const Neighbour& n, const SimTime& now, Yield& yield, McPlan& plan) const
{
    const McPoints requested = mcm::requestedPoints(*n.msg);
    if (requested.empty()) {
        return false;
    }
    const double speed = mPlanner->vehicleSpeed();
    const double requesterSpeed = requested.back().speed * 0.01;
    const double horizon = mNumPoints * mPointSpacing;
    const McPlan base = keepLanePlan(now);
    const double allowed = mVehicleController->getTraCI()->vehicle.getAllowedSpeed(vehicleId());

    // candidates in order of increasing effort: first the smallest change of speed,
    // then the smallest acceleration
    for (double loss = 0.0; loss <= mMaxYieldSpeedLoss + 1e-6; loss += 1.0) {
        std::vector<std::pair<double, double>> candidates; // (target speed, acceleration)
        const double slower = std::min(speed, requesterSpeed) - loss;
        if (slower >= 0.0) {
            if (speed - slower < 0.05) {
                candidates.emplace_back(speed, 0.0);
            } else {
                for (double a = 0.5; a <= mMaxYieldDeceleration + 1e-6; a += 0.5) {
                    candidates.emplace_back(slower, -a);
                }
            }
        }
        const double faster = std::min(allowed, std::max(speed, requesterSpeed) + loss);
        if (mMaxYieldAcceleration > 0.0 && faster - speed > 0.05) {
            for (double a = 0.5; a <= mMaxYieldAcceleration + 1e-6; a += 0.5) {
                candidates.emplace_back(faster, a);
            }
        }

        for (const auto& candidate : candidates) {
            McPlan trial = base;
            trial.lon.v0 = speed;
            trial.lon.accel = candidate.second;
            trial.lon.accelDuration = candidate.second != 0.0 ? std::abs(candidate.first - speed) / std::abs(candidate.second) : 0.0;
            trial.lon.maxSpeed = candidate.second > 0.0 ? candidate.first : 1e9;
            trial.type = MC_YIELD;
            if (!affectedBy(n, trial, now)) {
                plan = trial;
                yield.requester = n.msg->getStationId();
                yield.requestId = n.msg->getRequestedId();
                yield.since = now;
                yield.until = n.generated + SimTime(std::min(mcm::horizonOf(requested), horizon)) + SimTime(0.5);
                yield.targetSpeed = candidate.second != 0.0 ? candidate.first : speed;
                yield.deceleration = candidate.second;
                yield.holding = false;
                return true;
            }
        }
    }
    return false;
}

void BaseMcService::updateYield(const SimTime& now)
{
    auto found = mNeighbours.find(mYield.requester);
    const bool requested = found != mNeighbours.end() && found->second.msg->getRequestedId() == mYield.requestId;
    const bool executing = found != mNeighbours.end() && found->second.msg->getReferenceId() == mYield.requestId;
    mYield.executionSeen = mYield.executionSeen || executing;
    if (!requested && !executing) {
        // request withdrawn, manoeuvre completed or aborted with a new reference trajectory
        releaseYield(mYield.executionSeen ? "partner_done" : "partner_released", now);
        return;
    }
    if (now >= mYield.until) {
        releaseYield("partner_expired", now);
        return;
    }

    if (!mYield.holding) {
        const double duration = mYield.deceleration != 0.0 ?
            std::abs(mYield.startSpeed - mYield.targetSpeed) / std::abs(mYield.deceleration) : 0.0;
        if (now >= mYield.since + SimTime(duration)) {
            holdSpeed(mYield.targetSpeed);
            mYield.holding = true;
        }
    }
}

void BaseMcService::releaseYield(const char* outcome, const SimTime& now)
{
    auto& log = mcm::McLog::instance();
    if (log.isOpen()) {
        log.maneuvers() << stationId() << "," << vehicleId() << "," << mReferenceId << ",yielder,"
            << static_cast<int>(MC_YIELD) << "," << mYield.requester << ","
            << mYield.since.dbl() << "," << mYield.since.dbl() << ",-1," << now.dbl() << "," << outcome << ","
            << false << ",," << "," << "0,-1,0,,0" << std::endl;
    }

    mYield = Yield();
    mState = State::Idle;
    if (!mVehicleGone) {
        releaseSpeed();
        setReference(keepLanePlan(now), MC_KEEP_LANE, now);
    }
}

void BaseMcService::logRequestDecision(const Neighbour& n, int state, const char* decision, const Yield* yield,
    const SimTime& now) const
{
    auto& log = mcm::McLog::instance();
    if (!log.isOpen()) {
        return;
    }
    // position of the requester at the generation of its MCM, relative to the own position
    const Enu offset = mcm::toEnu(geoOf(*mVehicleDataProvider), mcm::referenceOf(*n.msg));
    const double heading = headingDegrees(*mVehicleDataProvider);
    const McPoints requested = mcm::requestedPoints(*n.msg);
    auto& out = log.requests();
    out << now.dbl() << "," << stationId() << "," << vehicleId() << "," << stateName(state) << ","
        << n.msg->getStationId() << "," << n.msg->getRequestedId() << "," << static_cast<int>(n.msg->getRequestedType()) << ","
        << decision << "," << mcm::length(offset) << ","
        << mcm::dot(offset, mcm::headingVector(heading)) << "," << mcm::dot(offset, mcm::leftVector(heading)) << ","
        << mPlanner->vehicleSpeed() << "," << n.msg->getSpeed() * 0.01 << ","
        << (requested.empty() ? 0.0 : requested.back().speed * 0.01) << ",";
    if (yield) {
        out << yield->targetSpeed << "," << yield->deceleration;
    } else {
        out << ",";
    }
    // few lines: flushed at once, so the file can be read while the simulation runs
    out << std::endl;
}

void BaseMcService::respond(uint32_t requester, uint16_t requestId, bool accept, McResponseReason reason, const SimTime& now)
{
    if (!mExplicitResponses) {
        return; // base concept: the (adapted or unchanged) reference trajectory is the answer
    }
    PendingResponse pending;
    pending.response.stationId = requester;
    pending.response.trajectoryId = requestId;
    pending.response.accept = accept;
    pending.response.reason = reason;
    pending.until = now + mResponseLifetime;
    mResponses[std::make_pair(requester, requestId)] = pending;
    mSendNow = true;
}

// --- conflicts ----------------------------------------------------------------

std::vector<double> BaseMcService::checkTimes(const SimTime& now, double until) const
{
    std::vector<double> times;
    const double end = std::min(until, now.dbl() + mNumPoints * mPointSpacing);
    for (double t = now.dbl(); t <= end + 1e-6; t += mCheckStep) {
        times.push_back(t);
    }
    return times;
}

Body BaseMcService::neighbourBody(const Neighbour& n, const McPoints& points, const Geo& origin, double t) const
{
    const auto sample = mcm::sampleExtrapolated(points, t - n.generated.dbl());
    Body body;
    body.front = mcm::toEnu(origin, mcm::referenceOf(*n.msg)) + sample.position;
    body.heading = sample.valid ? sample.heading : mcm::headingOf(*n.msg);
    body.speed = sample.valid ? sample.speed : n.msg->getSpeed() * 0.01;
    body.length = n.msg->getVehicleLength() > 0 ? n.msg->getVehicleLength() * 0.1 : 5.0;
    body.width = n.msg->getVehicleWidth() > 0 ? n.msg->getVehicleWidth() * 0.1 : 1.8;
    return body;
}

std::set<uint32_t> BaseMcService::conflictsOf(const McPlan& plan, const McPlan& keep, const SimTime& now,
        const SimTime& requestTime, bool& blockedByUnknown) const
{
    std::set<uint32_t> result;
    blockedByUnknown = false;

    // only conflicts caused by the manoeuvre count, not those of ordinary car following
    const auto times = checkTimes(now, plan.lat.end() + 1.0);
    const Geo origin = mPlanner->vehicleGeo();
    const auto moving = mPlanner->footprints(plan, times, mLength, mWidth);
    const auto keeping = mPlanner->footprints(keep, times, mLength, mWidth);
    auto causedBy = [&](const std::function<Body(double)>& other) {
        for (std::size_t i = 0; i < times.size(); ++i) {
            const Body b = other(times[i]);
            if (mcm::conflicts(moving[i], b, mConflictRules) && !mcm::conflicts(keeping[i], b, mConflictRules)) {
                return true;
            }
        }
        return false;
    };

    std::vector<Enu> known; // current positions of vehicles known from MCMs
    for (const auto& entry : mNeighbours) {
        const Neighbour& n = entry.second;
        if (mcm::length(mcm::toEnu(origin, mcm::referenceOf(*n.msg))) > mRequestRange) {
            continue; // too far away to be affected within the horizon
        }
        const McPoints reference = mcm::referencePoints(*n.msg);
        known.push_back(neighbourBody(n, reference, origin, now.dbl()).front);
        if (causedBy([&](double t) { return neighbourBody(n, reference, origin, t); })) {
            result.insert(entry.first);
            continue;
        }
        // an earlier request of another vehicle has precedence (TR 103 578 clause 5.2.3),
        // ties are broken by the station id
        const bool earlier = n.requestSeen < requestTime || (n.requestSeen == requestTime && entry.first < stationId());
        if (n.msg->getRequestedId() != 0 && earlier) {
            const McPoints requested = mcm::requestedPoints(*n.msg);
            if (causedBy([&](double t) { return neighbourBody(n, requested, origin, t); })) {
                result.insert(entry.first);
            }
        }
    }

#ifdef WITH_ENVMOD
    // vehicles without MCM, perceived by the own sensors, extrapolated at constant velocity
    if (mEnvironmentModel) {
        for (const auto& tracked : mEnvironmentModel->allObjects()) {
            auto object = tracked.first.lock();
            auto vehicle = dynamic_cast<const TraCIEnvironmentModelObject*>(object.get());
            if (!vehicle) {
                continue;
            }
            const auto& data = vehicle->getVehicleData();
            const Enu position = mcm::toEnu(origin, geoOf(data));
            const bool hasMcm = std::any_of(known.begin(), known.end(),
                    [&position](const Enu& p) { return mcm::length(p - position) < 3.0; });
            if (hasMcm) {
                continue;
            }
            const double heading = headingDegrees(data);
            const double speed = data.speed().value();
            const double length = vehicle->getLength().value();
            const double width = vehicle->getWidth().value();
            if (causedBy([&](double t) {
                    Body b;
                    b.front = position + ((t - now.dbl()) * speed) * mcm::headingVector(heading);
                    b.heading = heading;
                    b.speed = speed;
                    b.length = length;
                    b.width = width;
                    return b;
                })) {
                blockedByUnknown = true;
                break;
            }
        }
    }
#endif

    return result;
}

bool BaseMcService::affectedBy(const Neighbour& n, const McPlan& own, const SimTime& now) const
{
    const McPoints requested = mcm::requestedPoints(*n.msg);
    if (requested.empty()) {
        return false;
    }
    const McPoints reference = mcm::referencePoints(*n.msg);
    const auto times = checkTimes(now, n.generated.dbl() + mcm::horizonOf(requested));
    const Geo origin = mPlanner->vehicleGeo();
    const auto mine = mPlanner->footprints(own, times, mLength, mWidth);
    for (std::size_t i = 0; i < times.size(); ++i) {
        // the requester is the reference body: conflicts are judged along its heading
        const Body moving = neighbourBody(n, requested, origin, times[i]);
        const Body keeping = neighbourBody(n, reference, origin, times[i]);
        // the yielder judges the request with a margin on top of the rules of the requester: both
        // know the other only from its last MCM (age, 250 ms points), and the smallest sufficient
        // adaptation would otherwise leave the requester's own check at the edge (hysteresis);
        // conflicts of ordinary car following are excluded with the rules of the requester
        if (mcm::conflicts(moving, mine[i], mYieldRules) && !mcm::conflicts(keeping, mine[i], mConflictRules)) {
            return true;
        }
    }
    return false;
}

// --- reference trajectory and transmission ----------------------------------

McPlan BaseMcService::keepLanePlan(const SimTime& now) const
{
    const auto lane = mPlanner->laneState();
    const double horizon = mNumPoints * mPointSpacing;
    McPlan plan = mPlanner->planLane(lane, now.dbl(), std::max(mPlanner->vehicleSpeed(), 10.0) * horizon + 30.0);
    plan.type = MC_KEEP_LANE;
    return plan;
}

void BaseMcService::setReference(const McPlan& plan, McManeuverType type, const SimTime& now, uint16_t id)
{
    if (id == 0) {
        id = nextTrajectoryId();
    }
    mReference = plan;
    mReferenceType = type;
    mReferenceId = id;
    mSendNow = true;
}

uint16_t BaseMcService::nextTrajectoryId()
{
    // ids identify trajectories of one sender, 0 means "none"
    if (++mTrajectoryCounter == 0) {
        ++mTrajectoryCounter;
    }
    return mTrajectoryCounter;
}

void BaseMcService::refreshReference(const SimTime& now)
{
    const double speed = mPlanner->vehicleSpeed();
    switch (mState) {
        case State::Idle:
        case State::Requesting:
            // lane keeping at the current speed, with the path of the current lane
            mReference = keepLanePlan(now);
            break;
        case State::Committed:
        case State::Executing:
            mPlanner->anchor(mManeuver.plan, now.dbl(), speed);
            mReference = mManeuver.plan;
            break;
        case State::Yielding: {
            McPlan plan = keepLanePlan(now);
            plan.type = MC_YIELD;
            const double dv = mYield.targetSpeed - speed;
            if (!mYield.holding && mYield.deceleration != 0.0 && std::abs(dv) > 0.05) {
                plan.lon.accel = dv > 0.0 ? std::abs(mYield.deceleration) : -std::abs(mYield.deceleration);
                plan.lon.accelDuration = std::abs(dv) / std::abs(mYield.deceleration);
                plan.lon.maxSpeed = dv > 0.0 ? mYield.targetSpeed : 1e9;
            }
            mReference = plan;
            break;
        }
    }
    if (mState == State::Requesting) {
        mPlanner->anchor(mManeuver.plan, now.dbl(), speed);
    }
}

bool BaseMcService::deviatesFromReference(const SimTime& now) const
{
    if (mReference.empty()) {
        return true;
    }
    const double t = now.dbl();
    const auto actual = mPlanner->project(mReference, mPlanner->vehiclePosition());
    const double plannedS = mReference.s0 + mReference.lon.distance(t - mReference.t0);
    const double plannedSpeed = mReference.lon.speed(t - mReference.t0);
    return std::abs(actual.s - plannedS) > mLongitudinalDeviation ||
        std::abs(actual.lateral - mReference.lat.offset(t)) > mLateralDeviation ||
        std::abs(mPlanner->vehicleSpeed() - plannedSpeed) > mSpeedDeviation;
}

void BaseMcService::transmit(const SimTime& now)
{
    if (now < mSilentUntil) {
        return;
    }
    if (mLastSent != SimTime::ZERO && now - mLastSent < mMinInterval) {
        return;
    }

    bool send = mSendNow || mLastSent == SimTime::ZERO || now - mLastSent >= mMaxInterval;
    send = send || (mState == State::Requesting && now - mLastSent >= mRequestRepeatInterval);
    send = send || deviatesFromReference(now);
    if (!send) {
        return;
    }

    refreshReference(now);
    auto msg = createMessage(now);
    beforeSend(*msg);
    sendMcm(*msg);
    mSendNow = false;
    mLastSent = now;
}

std::unique_ptr<McMessage> BaseMcService::createMessage(const SimTime& now)
{
    const auto& vdp = *mVehicleDataProvider;
    std::unique_ptr<McMessage> msg { new McMessage("MCM") };
    msg->setStationId(vdp.getStationId());
    msg->setGenerationDeltaTime(countTaiMilliseconds(mTimer->getTimeFor(vdp.updated())));
    msg->setLatitude(mcm::toTenthMicrodegree(vdp.latitude().value()));
    msg->setLongitude(mcm::toTenthMicrodegree(vdp.longitude().value()));
    msg->setHeading(static_cast<uint16_t>(std::lround(headingDegrees(vdp) * 10.0)) % 3600);
    msg->setSpeed(static_cast<uint16_t>(std::lround(std::max(0.0, vdp.speed().value()) * 100.0)));
    msg->setVehicleLength(static_cast<uint16_t>(std::lround(mLength * 10.0)));
    msg->setVehicleWidth(static_cast<uint16_t>(std::lround(mWidth * 10.0)));

    msg->setReferenceId(mReferenceId);
    msg->setReferenceType(mReferenceType);
    mcm::setReferencePoints(*msg, mPlanner->sample(mReference, now.dbl(), mNumPoints, mPointSpacing));

    if (mState == State::Requesting) {
        msg->setRequestedId(mManeuver.id);
        msg->setRequestedType(mManeuver.type);
        mcm::setRequestedPoints(*msg, mPlanner->sample(mManeuver.plan, now.dbl(), mNumPoints, mPointSpacing));
    } else {
        msg->setRequestedId(0);
        msg->setRequestedType(MC_KEEP_LANE);
    }

    std::vector<McResponse> responses;
    for (const auto& entry : mResponses) {
        if (entry.second.until > now) {
            responses.push_back(entry.second.response);
        }
    }
    msg->setResponsesArraySize(responses.size());
    for (unsigned i = 0; i < responses.size(); ++i) {
        msg->setResponses(i, responses[i]);
    }
    return msg;
}

void BaseMcService::sendMcm(McMessage& msg)
{
    using namespace vanetza;
    btp::DataRequestB req;
    req.destination_port = host_cast<BaseMcService::port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
    req.gn.traffic_class.tc_id(mTrafficClass);
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    McMessage* packet = msg.dup();
    const unsigned requested = packet->getRequestedArraySize();
    packet->setByteLength(mHeaderBytes + mPointBytes * (packet->getReferenceArraySize() + requested) +
        (packet->getRequestedId() != 0 ? mRequestHeaderBytes : 0) + mResponseBytes * packet->getResponsesArraySize());
    emit(scSignalMcmSent, packet);

    ++mSent;
    if (packet->getRequestedId() != 0) {
        ++mSentWithRequest;
    }
    mSentResponses += packet->getResponsesArraySize();

    auto& log = mcm::McLog::instance();
    if (mLogTransmissions && log.isOpen()) {
        auto& out = log.transmissions();
        out << simTime().dbl() << "," << generationTimeOf(*packet).dbl() << "," << packet->getStationId() << ","
            << stateName(static_cast<int>(mState)) << ","
            << packet->getReferenceId() << "," << static_cast<int>(packet->getReferenceType()) << ","
            << packet->getRequestedId() << "," << static_cast<int>(packet->getRequestedType()) << ","
            << packet->getGenerationDeltaTime() << "," << packet->getLatitude() << "," << packet->getLongitude() << ","
            << packet->getHeading() << "," << packet->getSpeed() << ","
            << packet->getVehicleLength() << "," << packet->getVehicleWidth() << "," << packet->getByteLength() << ",";
        writePoints(out, mcm::referencePoints(*packet));
        out << ",";
        writePoints(out, mcm::requestedPoints(*packet));
        out << ",";
        for (unsigned i = 0; i < packet->getResponsesArraySize(); ++i) {
            const McResponse& r = packet->getResponses(i);
            out << (i ? ";" : "") << r.stationId << ":" << r.trajectoryId << ":" << r.accept << ":" << static_cast<int>(r.reason);
        }
        out << "\n";
    }

    if (mVisualizeTrajectory) {
        // blue keep lane, green lane change, cyan yield; requested trajectories orange
        libsumo::TraCIColor color(30, 70, 130);
        if (packet->getReferenceType() == MC_YIELD) {
            color = libsumo::TraCIColor(0, 170, 210);
        } else if (packet->getReferenceType() != MC_KEEP_LANE) {
            color = libsumo::TraCIColor(140, 190, 30);
        }
        drawTrajectory("ref", mcm::referencePoints(*packet), *packet, color, packet->getReferenceType() == MC_KEEP_LANE ? 0.25 : 0.4);
        if (requested > 0) {
            drawTrajectory("req", mcm::requestedPoints(*packet), *packet, libsumo::TraCIColor(240, 150, 0), 0.4);
        } else {
            eraseTrajectory("req");
        }
    }

    request(req, packet);
}

void BaseMcService::controlLateral(const SimTime& now)
{
    // closed loop on the lateral offset from the origin lane: SUMO moves the vehicle by
    // the commanded distance within the next step (sublane model, limited by maxSpeedLat)
    const double target = mManeuver.plan.lat.offset(now.dbl() + mLateralLookahead);
    const double actual = mPlanner->project(mManeuver.plan, mPlanner->vehiclePosition()).lateral;
    const double delta = target - actual;
    if (std::abs(delta) > 0.005) {
        mVehicleController->getTraCI()->vehicle.changeSublane(vehicleId(), delta);
    }
}

void BaseMcService::holdSpeed(double speed)
{
    mVehicleController->getTraCI()->vehicle.setSpeed(vehicleId(), speed);
}

void BaseMcService::releaseSpeed()
{
    if (mVehicleGone) {
        return;
    }
    mVehicleController->getTraCI()->vehicle.setSpeed(vehicleId(), -1.0);
}

// --- reception ----------------------------------------------------------------

void BaseMcService::indicate(const vanetza::btp::DataIndication& ind, cPacket* packet)
{
    Enter_Method("indicate");
    std::unique_ptr<McMessage> msg { check_and_cast<McMessage*>(packet) };
    ++mReceived;
    // p_drop: additional loss on top of the channel, e.g. to lose answers and aborts
    if (mDropProbability > 0.0 && uniform(0.0, 1.0) < mDropProbability) {
        ++mDropped;
        return;
    }

    // the detector consumes MCMs through this signal, after the simulated loss
    emit(scSignalMcmReceived, msg.get());

    const SimTime now = simTime();
    const uint32_t sender = msg->getStationId();
    if (sender == stationId()) {
        return;
    }

    // explicit answers to the own request
    if (mState == State::Requesting) {
        for (unsigned i = 0; i < msg->getResponsesArraySize(); ++i) {
            const McResponse& r = msg->getResponses(i);
            if (r.stationId == stationId() && r.trajectoryId == mManeuver.id) {
                (r.accept ? mManeuver.acks : mManeuver.nacks).insert(sender);
            }
        }
    }

    Neighbour& n = mNeighbours[sender];
    const uint16_t requestId = msg->getRequestedId();
    if (requestId != 0 && (!n.msg || n.msg->getRequestedId() != requestId)) {
        n.requestSeen = now;
    }
    n.generated = generationTimeOf(*msg);
    n.received = now;
    n.msg = std::move(msg);

    if (requestId != 0 && !mVehicleGone) {
        handleRequest(n, now);
    }
}

void BaseMcService::expireNeighbours(const SimTime& now)
{
    for (auto it = mNeighbours.begin(); it != mNeighbours.end();) {
        if (now - it->second.received > mNeighbourTimeout) {
            it = mNeighbours.erase(it);
        } else {
            ++it;
        }
    }
    for (auto it = mEvaluatedRequests.begin(); it != mEvaluatedRequests.end();) {
        auto found = mNeighbours.find(it->first);
        if (found == mNeighbours.end() || found->second.msg->getRequestedId() != it->second) {
            it = mEvaluatedRequests.erase(it);
        } else {
            ++it;
        }
    }
    for (auto it = mResponses.begin(); it != mResponses.end();) {
        if (it->second.until <= now) {
            it = mResponses.erase(it);
        } else {
            ++it;
        }
    }
}

SimTime BaseMcService::generationTimeOf(const McMessage& msg) const
{
    return mTimer->getTimeFor(mTimer->reconstructMilliseconds(msg.getGenerationDeltaTime()));
}

// --- visualisation ------------------------------------------------------------

void BaseMcService::receiveSignal(cComponent*, simsignal_t, const SimTime&, cObject*)
{
    // only subscribed to traci.close
    mTraciClosed = true;
}

void BaseMcService::drawTrajectory(const std::string& suffix, const McPoints& trajectory, const McMessage& msg,
        const libsumo::TraCIColor& color, double lineWidth)
{
    if (mTraciClosed || trajectory.empty()) {
        return;
    }

    const double headWidth = lineWidth > 0.3 ? 2.4 : 1.6;
    const double headLength = 1.5 * headWidth;

    // The net uses a transverse Mercator projection centred on it, so grid north
    // equals true north up to a negligible convergence and the east/north offsets
    // can be added to the converted reference position directly.
    auto& api = *mVehicleController->getTraCI();
    const Geo ref = mcm::referenceOf(msg);
    const libsumo::TraCIPosition origin = api.convert2D(traci::TraCIGeoPosition { ref.longitude, ref.latitude });
    std::vector<Enu> points { Enu {} };
    for (const McTrajectoryPoint& p : trajectory) {
        points.push_back(Enu { p.deltaEast * 0.01, p.deltaNorth * 0.01 });
    }

    // arrow head along the last segment, the line ends at its base
    const Enu tip = points.back();
    Enu direction = tip - points[points.size() - 2];
    const double segment = mcm::length(direction);
    direction = segment > 1e-3 ? (1.0 / segment) * direction : mcm::headingVector(mcm::headingOf(msg));
    const Enu base = tip - headLength * direction;
    const Enu side = Enu { direction.north, -direction.east };
    points.back() = base;

    auto toSumo = [&origin](const Enu& e) {
        libsumo::TraCIPosition pos;
        pos.x = origin.x + e.east;
        pos.y = origin.y + e.north;
        return pos;
    };
    libsumo::TraCIPositionVector line, head;
    for (const Enu& e : points) {
        line.value.push_back(toSumo(e));
    }
    head.value = { toSumo(tip), toSumo(base + 0.5 * headWidth * side), toSumo(base - 0.5 * headWidth * side) };

    const std::string id = "mcm." + vehicleId() + "." + suffix;
    if (mDrawn.count(suffix)) {
        api.polygon.setShape(id, line);
        api.polygon.setColor(id, color);
        api.polygon.setShape(id + ".head", head);
        api.polygon.setColor(id + ".head", color);
    } else {
        api.polygon.add(id, line, color, false, "mcm", 100);
        api.polygon.add(id + ".head", head, color, true, "mcm", 100);
        mDrawn.insert(suffix);
    }
    api.polygon.setLineWidth(id, lineWidth);
}

void BaseMcService::eraseTrajectory(const std::string& suffix)
{
    if (!mDrawn.count(suffix) || mTraciClosed) {
        return;
    }
    auto& api = *mVehicleController->getTraCI();
    const std::string id = "mcm." + vehicleId() + "." + suffix;
    api.polygon.remove(id, 100);
    api.polygon.remove(id + ".head", 100);
    mDrawn.erase(suffix);
}

} // namespace artery
