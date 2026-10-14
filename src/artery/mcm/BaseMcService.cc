#include "artery/mcm/BaseMcService.h"
#include "artery/mcm/McGeometry.h"
#include "artery/mcm/McLog.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/application/Timer.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/checkandcast.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/common/byte_order.hpp>
#include <vanetza/geonet/interface.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace artery
{

using namespace omnetpp;
using mcm::Enu;
using mcm::Geo;

Define_Module(BaseMcService)

namespace
{

const simsignal_t scSignalMcmSent = cComponent::registerSignal("McmSent");
const simsignal_t scSignalMcmReceived = cComponent::registerSignal("McmReceived");

// lane change actions of SUMO's lane change model (libsumo::LCA_*)
constexpr int LCA_LEFT = 1 << 1;
constexpr int LCA_RIGHT = 1 << 2;
constexpr int LCA_STRATEGIC = 1 << 3;
constexpr int LCA_BLOCKED_BY_LEFT_LEADER = 1 << 9;
constexpr int LCA_BLOCKED_BY_RIGHT_LEADER = 1 << 11;

constexpr double pi = 3.14159265358979323846;

double headingDegrees(const VehicleDataProvider& vdp)
{
    double deg = vdp.heading().value() * 180.0 / pi;
    deg = std::fmod(deg, 360.0);
    return deg < 0.0 ? deg + 360.0 : deg;
}

double speedOf(const VehicleDataProvider& vdp)
{
    return vdp.speed().value();
}

Geo geoOf(const VehicleDataProvider& vdp)
{
    return Geo { vdp.latitude().value(), vdp.longitude().value() };
}

} // namespace

void BaseMcService::initialize()
{
    ItsG5Service::initialize();
    mVehicleController = &getFacilities().get_mutable<traci::VehicleController>();
    mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
    mTimer = &getFacilities().get_const<Timer>();
    mLocalDynamicMap = &getFacilities().get_const<LocalDynamicMap>();
    mPlanner.reset(new McTrajectoryPlanner(*mVehicleController->getTraCI(), mVehicleController->getVehicleId()));

    mMinInterval = par("minInterval");
    mMaxInterval = par("maxInterval");
    mHeadingDelta = par("headingDelta");
    mPositionDelta = par("positionDelta");
    mSpeedDelta = par("speedDelta");
    mNumPoints = par("numPoints");
    mPointSpacing = par("pointSpacing");
    mLeadTime = par("leadTime");
    mRequestTimeout = par("requestTimeout");
    mRequestRepeatInterval = par("requestRepeatInterval");
    mExecutionRepeatInterval = par("executionRepeatInterval");
    mManeuverCooldown = par("maneuverCooldown");
    mStartTolerance = par("startTolerance");
    mLaneChangeDuration = par("laneChangeDuration");
    mWishReasons = par("wishReasons");
    mPartnerSearchDistance = par("partnerSearchDistance");
    mMaxAcceptDeceleration = par("maxAcceptDeceleration");
    mAcceptMinGap = par("acceptMinGap");
    mAcceptHeadway = par("acceptHeadway");
    mAssumedVehicleLength = par("assumedVehicleLength");
    mDropProbability = par("dropProbability");
    mHeaderBytes = par("headerBytes");
    mPointBytes = par("pointBytes");
    mTrafficClass = par("trafficClass");
    mLogTransmissions = par("logTransmissions");

    if (mRequestTimeout > mLeadTime) {
        throw cRuntimeError("requestTimeout must not exceed leadTime (thesis Section 4.1)");
    }
    if (mLeadTime + SimTime(mLaneChangeDuration) > SimTime(mNumPoints * mPointSpacing)) {
        throw cRuntimeError("lead time and lane change duration have to fit into the trajectory horizon");
    }

    // lane changes are commanded by this service only
    mVehicleController->getTraCI()->vehicle.setLaneChangeMode(vehicleId(), par("laneChangeMode").intValue());

    // give the LDM time to fill before the first maneuver
    mNextManeuverAllowed = simTime() + mManeuverCooldown;

    const std::string prefix = par("logPrefix").stdstringValue();
    if (!prefix.empty()) {
        mcm::McLog::instance().open(prefix);
    }
}

void BaseMcService::finish()
{
    if (mManeuver.id != 0) {
        // the vehicle may already have left SUMO, so do not touch it any more
        mVehicleGone = true;
        finishManeuver("unfinished", simTime());
    }
    recordScalar("mcmSentIntent", mSentByPhase[MC_INTENT]);
    recordScalar("mcmSentRequest", mSentByPhase[MC_REQUEST]);
    recordScalar("mcmSentAccept", mSentByPhase[MC_ACCEPT]);
    recordScalar("mcmSentExecute", mSentByPhase[MC_EXECUTE]);
    recordScalar("mcmSentCancel", mSentCancel);
    recordScalar("mcmReceived", mReceived);
    recordScalar("mcmDropped", mDropped);
    recordScalar("requestsDeclinedBusy", mDeclinedBusy);
    recordScalar("requestsDeclinedLate", mDeclinedLate);
    recordScalar("requestsDeclinedDeceleration", mDeclinedDeceleration);
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
        mcm::McLog::instance().stations() << stationId() << "," << vehicleId() << "," << getNedTypeName() << "\n";
        mStationLogged = true;
    }

    switch (mState) {
        case State::Idle:
            if (now >= mNextManeuverAllowed && checkLaneChangeWish(now)) {
                break;
            }
            checkIntentTriggers(now);
            break;
        case State::Requesting:
            if (now - mManeuver.announced >= mRequestTimeout) {
                sendCancel(mManeuver);
                finishManeuver("cancelled_timeout", now);
                checkIntentTriggers(now);
            } else if (now - mLastSent >= mRequestRepeatInterval) {
                // repetitions carry the same trajectory with the same generation time
                sendMcm(*mManeuver.announcement);
            }
            break;
        case State::Accepted:
            if (now >= mManeuver.lateralStart) {
                execute(now);
            } else {
                resend(now);
            }
            break;
        case State::Executing:
            checkExecution(now);
            break;
        case State::Partner:
            updatePartner(now);
            break;
        case State::Silent:
            if (now >= mSilentUntil) {
                mState = State::Idle;
                mForceIntent = true;
                checkIntentTriggers(now);
            }
            break;
    }
}

void BaseMcService::checkIntentTriggers(const SimTime& now)
{
    if (mLastSent != SimTime::ZERO && now - mLastSent < mMinInterval) {
        return;
    }

    const auto& vdp = *mVehicleDataProvider;
    bool send = mForceIntent || now - mLastIntent >= mMaxInterval;
    send = send || distance(mLastIntentPosition, vdp.position()).value() > mPositionDelta;
    send = send || std::abs(mcm::wrapAngle(headingDegrees(vdp) - mLastIntentHeading)) > mHeadingDelta;
    send = send || std::abs(speedOf(vdp) - mLastIntentSpeed) > mSpeedDelta;
    if (send) {
        sendIntent(now);
    }
}

void BaseMcService::sendIntent(const SimTime& now)
{
    // consecutive keep lane announcements form one maneuver, a new one starts after every lane change
    if (mManeuverCounter == 0 || mForceIntent) {
        ++mManeuverCounter;
    }

    auto msg = createMessage(MC_INTENT, MC_KEEP_LANE, mManeuverCounter);
    const auto lane = mPlanner->laneState();
    McLongitudinalProfile lon;
    lon.v0 = speedOf(*mVehicleDataProvider);
    McLateralProfile lat;
    lat.d0 = lane.lateralPosition;
    mPlanner->plan(*msg, lane, lon, lat, mNumPoints, mPointSpacing);
    sendMcm(*msg);

    mLastIntent = now;
    mLastIntentPosition = mVehicleDataProvider->position();
    mLastIntentHeading = headingDegrees(*mVehicleDataProvider);
    mLastIntentSpeed = lon.v0;
    mForceIntent = false;
}

bool BaseMcService::checkLaneChangeWish(const SimTime& now)
{
    const auto lane = mPlanner->laneState();
    if (lane.internal) {
        return false;
    }

    auto& vehicle = mVehicleController->getTraCI()->vehicle;
    int chosen = 0;
    int chosenState = 0;
    for (int direction : { +1, -1 }) {
        // first: state computed by the lane change model, regardless of the lane change mode
        const int state = vehicle.getLaneChangeState(vehicleId(), direction).first;
        const int want = direction > 0 ? LCA_LEFT : LCA_RIGHT;
        const int blockedByLeader = direction > 0 ? LCA_BLOCKED_BY_LEFT_LEADER : LCA_BLOCKED_BY_RIGHT_LEADER;
        if (!(state & want) || !(state & mWishReasons) || (state & blockedByLeader)) {
            continue; // a blocking leader cannot be resolved by negotiating with the follower
        }
        if (mPlanner->neighbourLane(lane, direction).empty()) {
            continue;
        }
        if (!chosen || ((state & LCA_STRATEGIC) && !(chosenState & LCA_STRATEGIC))) {
            chosen = direction;
            chosenState = state;
        }
    }

    if (!chosen) {
        return false;
    }

    McManeuverType type = chosen > 0 ? MC_LANE_CHANGE_LEFT : MC_LANE_CHANGE_RIGHT;
    if ((chosenState & LCA_STRATEGIC) && mPlanner->laneEnds(lane)) {
        type = MC_MERGE;
    }
    startLaneChange(chosen, type, now);
    return true;
}

void BaseMcService::startLaneChange(int direction, McManeuverType type, const SimTime& now)
{
    const auto lane = mPlanner->laneState();
    const std::string target = mPlanner->neighbourLane(lane, direction);
    const double width = 0.5 * (mPlanner->laneWidth(lane.laneId) + mPlanner->laneWidth(target));

    Maneuver m;
    m.id = ++mManeuverCounter;
    m.type = type;
    m.direction = direction;
    m.partnerId = findPartner(direction, width);
    m.announced = now;
    m.lateralStart = now + mLeadTime;
    m.end = m.lateralStart + mLaneChangeDuration;

    // without a vehicle behind the gap there is nobody to negotiate with
    auto msg = createMessage(m.partnerId ? MC_REQUEST : MC_EXECUTE, type, m.id);
    msg->setPartnerId(m.partnerId);
    McLongitudinalProfile lon;
    lon.v0 = speedOf(*mVehicleDataProvider);
    McLateralProfile lat;
    lat.d0 = lane.lateralPosition;
    lat.delta = direction * width - lane.lateralPosition;
    lat.start = mLeadTime.dbl();
    lat.duration = mLaneChangeDuration;
    mPlanner->plan(*msg, lane, lon, lat, mNumPoints, mPointSpacing);
    m.announcement = std::move(msg);

    mManeuver = std::move(m);
    mState = mManeuver.partnerId ? State::Requesting : State::Accepted;
    sendMcm(*mManeuver.announcement);

    // hold the announced constant speed (thesis Section 5.2)
    mVehicleController->getTraCI()->vehicle.setSpeed(vehicleId(), lon.v0);
}

uint32_t BaseMcService::findPartner(int direction, double laneWidth) const
{
    // nearest vehicle in the target lane behind the own front, known from received CAMs
    const Geo own = geoOf(*mVehicleDataProvider);
    const double heading = headingDegrees(*mVehicleDataProvider);
    const Enu forward = mcm::headingVector(heading);
    const Enu left = mcm::leftVector(heading);
    const SimTime now = simTime();

    uint32_t partner = 0;
    double nearest = -mPartnerSearchDistance;
    for (const auto& entry : mLocalDynamicMap->allEntries()) {
        const auto& cam = entry.second.cam();
        const auto& params = cam->cam.camParameters;
        const auto& ref = params.basicContainer.referencePosition;
        Enu relative = mcm::toEnu(own, Geo { ref.latitude * 1e-7, ref.longitude * 1e-7 });

        // extrapolate the CAM position to now
        if (params.highFrequencyContainer.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency) {
            const auto& hfc = params.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
            const SimTime generated = mTimer->getTimeFor(mTimer->reconstructMilliseconds(cam->cam.generationDeltaTime));
            const double age = std::max(0.0, (now - generated).dbl());
            const double speed = hfc.speed.speedValue * 0.01;
            relative = relative + (speed * age) * mcm::headingVector(hfc.heading.headingValue * 0.1);
        }

        const double lon = mcm::dot(relative, forward);
        const double lat = direction * mcm::dot(relative, left);
        if (lat > 0.5 * laneWidth && lat < 1.5 * laneWidth && lon <= 0.0 && lon > nearest) {
            nearest = lon;
            partner = entry.first;
        }
    }
    return partner;
}

void BaseMcService::execute(const SimTime& now)
{
    switch (decideExecution(mManeuver)) {
        case Execution::Drive: {
            const auto lane = mPlanner->laneState();
            if (lane.internal || mPlanner->neighbourLane(lane, mManeuver.direction).empty()) {
                sendCancel(mManeuver);
                finishManeuver("failed_cancel", now);
                return;
            }
            auto& vehicle = mVehicleController->getTraCI()->vehicle;
            const auto position = vehicle.getPosition(vehicleId());
            mManeuver.commandPosition = Enu { position.x, position.y };
            mManeuver.commandHeading = vehicle.getAngle(vehicleId());
            vehicle.changeLaneRelative(vehicleId(), mManeuver.direction, 0.2);
            mManeuver.announcement->setPhase(MC_EXECUTE);
            mState = State::Executing;
            sendMcm(*mManeuver.announcement);
            break;
        }
        case Execution::AbortWithCancel:
            sendCancel(mManeuver);
            finishManeuver("aborted_cancel", now);
            break;
        case Execution::AbortSilent: {
            // no message at all until the announcement has expired
            const SimTime expiry = mManeuver.announced + mcm::horizonOf(*mManeuver.announcement);
            finishManeuver("aborted_silent", now);
            mSilentUntil = expiry;
            mState = State::Silent;
            break;
        }
    }
}

void BaseMcService::checkExecution(const SimTime& now)
{
    if (!mManeuver.started) {
        // lateral displacement since the command, SUMO coordinates (x east, y north)
        auto& vehicle = mVehicleController->getTraCI()->vehicle;
        const auto position = vehicle.getPosition(vehicleId());
        const Enu moved = Enu { position.x, position.y } - mManeuver.commandPosition;
        const double lateral = mManeuver.direction * mcm::dot(moved, mcm::leftVector(mManeuver.commandHeading));
        if (lateral > 0.05) {
            mManeuver.started = true;
        } else if (now - mManeuver.lateralStart > mStartTolerance) {
            sendCancel(mManeuver);
            finishManeuver("failed_cancel", now);
            return;
        } else if (!mPlanner->laneState().internal) {
            vehicle.changeLaneRelative(vehicleId(), mManeuver.direction, 0.2);
        }
    }

    if (now >= mManeuver.end) {
        finishManeuver("executed", now);
        checkIntentTriggers(now);
    } else {
        resend(now);
    }
}

void BaseMcService::resend(const SimTime& now)
{
    if (now - mLastSent >= mExecutionRepeatInterval) {
        sendMcm(*mManeuver.announcement);
    }
}

void BaseMcService::indicate(const vanetza::btp::DataIndication& ind, cPacket* packet)
{
    Enter_Method("indicate");
    std::unique_ptr<McMessage> msg { check_and_cast<McMessage*>(packet) };
    ++mReceived;
    if (isDropped(*msg)) {
        ++mDropped;
        return;
    }

    // the detector consumes MCMs through this signal, after the simulated loss
    emit(scSignalMcmReceived, msg.get());

    const SimTime now = simTime();
    if (msg->getCancel()) {
        handleCancel(*msg);
    } else if (msg->getPhase() == MC_REQUEST) {
        handleRequest(*msg, now);
    } else if (msg->getPhase() == MC_ACCEPT) {
        handleAccept(*msg);
    }
}

bool BaseMcService::isDropped(const McMessage& msg)
{
    const bool droppable = msg.getCancel() || msg.getPhase() == MC_ACCEPT;
    return droppable && mDropProbability > 0.0 && uniform(0.0, 1.0) < mDropProbability;
}

void BaseMcService::handleRequest(const McMessage& msg, const SimTime& now)
{
    if (msg.getPartnerId() != stationId()) {
        return;
    }

    // repetition of a request already accepted: our accept may have been lost
    if (mState == State::Partner && mManeuver.partnerId == msg.getStationId() &&
            mManeuver.partnerManeuverId == msg.getManeuverId()) {
        if (now - mLastSent >= mRequestRepeatInterval) {
            sendMcm(*mManeuver.announcement);
        }
        return;
    }

    if (mState != State::Idle) {
        ++mDeclinedBusy;
        return; // busy with an own maneuver: no answer, the requester times out
    }

    // start of the lateral movement in the announced trajectory
    const double heading = mcm::headingOf(msg);
    const Enu forward = mcm::headingVector(heading);
    const Enu left = mcm::leftVector(heading);
    double lateralStart = -1.0;
    double previousTime = 0.0;
    for (unsigned i = 0; i < msg.getPointsArraySize(); ++i) {
        const McTrajectoryPoint& point = msg.getPoints(i);
        const Enu p { point.deltaEast * 0.01, point.deltaNorth * 0.01 };
        if (std::abs(mcm::dot(p, left)) > 0.2) {
            lateralStart = previousTime;
            break;
        }
        previousTime = point.timeOffset * 1e-3;
    }
    if (lateralStart < 0.0) {
        return;
    }

    const SimTime generated = generationTimeOf(msg);
    const double tau = lateralStart - (now - generated).dbl();
    if (tau < 0.3) {
        ++mDeclinedLate;
        return; // too late to open a gap
    }

    // current gap between own front and the requester's rear, from the requester's announcement
    const auto requester = mcm::sampleTrajectory(msg, std::min(lateralStart, (now - generated).dbl()));
    const Enu own = mcm::toEnu(mcm::referenceOf(msg), geoOf(*mVehicleDataProvider));
    const double gap = mcm::dot(requester.position - own, forward) - mAssumedVehicleLength;
    const double v = speedOf(*mVehicleDataProvider);
    const double vReq = requester.speed;

    // decelerate constantly until the speed of the requester is reached, so that the gap
    // does not fall below the safe gap: (v - vReq)^2 / (2 a) <= gap - minGap - vReq * headway
    double decel = 0.0;
    double decelDuration = 0.0;
    double vt = v;
    if (v > vReq) {
        const double margin = gap - mAcceptMinGap - vReq * mAcceptHeadway;
        decel = margin > 0.0 ? (v - vReq) * (v - vReq) / (2.0 * margin) : std::numeric_limits<double>::infinity();
        decelDuration = (v - vReq) / decel;
        vt = vReq;
    } else if (gap + (vReq - v) * tau < mAcceptMinGap + v * mAcceptHeadway) {
        decel = std::numeric_limits<double>::infinity(); // too close even without closing in
    }
    if (decel > mMaxAcceptDeceleration) {
        ++mDeclinedDeceleration;
        return; // no answer: the requester times out and cancels
    }

    Maneuver m;
    m.id = ++mManeuverCounter;
    m.type = MC_KEEP_LANE;
    m.role = "partner";
    m.partnerId = msg.getStationId();
    m.partnerManeuverId = msg.getManeuverId();
    m.announced = now;
    m.lateralStart = now + tau;
    m.end = std::min(m.lateralStart + mLaneChangeDuration + 0.5, now + mNumPoints * mPointSpacing);
    m.acceptReceived = true;
    m.holdSpeed = vt;
    m.holdFrom = now + decelDuration;

    auto accept = createMessage(MC_ACCEPT, MC_KEEP_LANE, m.id);
    accept->setPartnerId(m.partnerId);
    accept->setPartnerManeuverId(m.partnerManeuverId);
    const auto lane = mPlanner->laneState();
    McLongitudinalProfile lon;
    lon.v0 = v;
    lon.accel = -decel;
    lon.accelDuration = decelDuration;
    McLateralProfile lat;
    lat.d0 = lane.lateralPosition;
    mPlanner->plan(*accept, lane, lon, lat, mNumPoints, mPointSpacing);
    m.announcement = std::move(accept);

    mManeuver = std::move(m);
    mState = State::Partner;
    sendMcm(*mManeuver.announcement);

    auto& vehicle = mVehicleController->getTraCI()->vehicle;
    if (decelDuration > 0.1) {
        vehicle.slowDown(vehicleId(), vt, decelDuration);
    } else {
        vehicle.setSpeed(vehicleId(), vt);
        mManeuver.holding = true;
    }
}

void BaseMcService::handleAccept(const McMessage& msg)
{
    if (mState == State::Requesting && msg.getStationId() == mManeuver.partnerId &&
            msg.getPartnerId() == stationId() && msg.getPartnerManeuverId() == mManeuver.id) {
        mManeuver.acceptReceived = true;
        mState = State::Accepted;
    }
}

void BaseMcService::handleCancel(const McMessage& msg)
{
    if (mState == State::Partner && msg.getStationId() == mManeuver.partnerId &&
            msg.getCancelledManeuverId() == mManeuver.partnerManeuverId) {
        finishManeuver("partner_released", simTime());
    }
}

void BaseMcService::updatePartner(const SimTime& now)
{
    if (!mManeuver.holding && now >= mManeuver.holdFrom) {
        mVehicleController->getTraCI()->vehicle.setSpeed(vehicleId(), mManeuver.holdSpeed);
        mManeuver.holding = true;
    }

    if (now >= mManeuver.end) {
        finishManeuver("partner_done", now);
        checkIntentTriggers(now);
    } else {
        resend(now);
    }
}

void BaseMcService::finishManeuver(const char* outcome, const SimTime& now)
{
    auto& log = mcm::McLog::instance();
    if (log.isOpen()) {
        log.maneuvers() << stationId() << "," << vehicleId() << "," << mManeuver.id << ","
            << mManeuver.role << "," << static_cast<int>(mManeuver.type) << ","
            << mManeuver.partnerId << "," << mManeuver.partnerManeuverId << ","
            << mManeuver.announced.dbl() << "," << mManeuver.lateralStart.dbl() << "," << now.dbl() << ","
            << outcome << "," << mManeuver.acceptReceived << "," << mManeuver.cancelSent << "\n";
    }

    releaseSpeed();
    mManeuver = Maneuver();
    mState = State::Idle;
    mForceIntent = true;
    mNextManeuverAllowed = now + mManeuverCooldown;
}

void BaseMcService::releaseSpeed()
{
    if (mVehicleGone) {
        return;
    }
    mVehicleController->getTraCI()->vehicle.setSpeed(vehicleId(), -1.0);
}

std::unique_ptr<McMessage> BaseMcService::createMessage(McPhase phase, McManeuverType type, uint16_t maneuverId) const
{
    const auto& vdp = *mVehicleDataProvider;
    std::unique_ptr<McMessage> msg { new McMessage("MCM") };
    msg->setStationId(vdp.getStationId());
    msg->setGenerationDeltaTime(countTaiMilliseconds(mTimer->getTimeFor(vdp.updated())));
    msg->setLatitude(mcm::toTenthMicrodegree(vdp.latitude().value()));
    msg->setLongitude(mcm::toTenthMicrodegree(vdp.longitude().value()));
    msg->setHeading(static_cast<uint16_t>(std::lround(headingDegrees(vdp) * 10.0)) % 3600);
    msg->setManeuverType(type);
    msg->setPhase(phase);
    msg->setManeuverId(maneuverId);
    return msg;
}

void BaseMcService::sendCancel(Maneuver& m)
{
    auto msg = createMessage(m.announcement ? static_cast<McPhase>(m.announcement->getPhase()) : MC_INTENT, m.type, m.id);
    msg->setPartnerId(m.partnerId);
    msg->setCancel(true);
    msg->setCancelledManeuverId(m.id);
    sendMcm(*msg);
    m.cancelSent = true;
}

void BaseMcService::sendMcm(const McMessage& msg)
{
    using namespace vanetza;
    btp::DataRequestB req;
    req.destination_port = host_cast<BaseMcService::port_type>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
    req.gn.traffic_class.tc_id(mTrafficClass);
    req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    McMessage* packet = msg.dup();
    packet->setByteLength(mHeaderBytes + mPointBytes * packet->getPointsArraySize());
    emit(scSignalMcmSent, packet);

    if (packet->getCancel()) {
        ++mSentCancel;
    } else if (packet->getPhase() < 4) {
        ++mSentByPhase[packet->getPhase()];
    }

    auto& log = mcm::McLog::instance();
    if (mLogTransmissions && log.isOpen()) {
        auto& out = log.transmissions();
        out << simTime().dbl() << "," << generationTimeOf(*packet).dbl() << "," << packet->getStationId() << "," << packet->getManeuverId() << ","
            << static_cast<int>(packet->getPhase()) << "," << static_cast<int>(packet->getManeuverType()) << ","
            << packet->getPartnerId() << "," << packet->getPartnerManeuverId() << ","
            << packet->getCancel() << "," << packet->getCancelledManeuverId() << ","
            << packet->getGenerationDeltaTime() << "," << packet->getLatitude() << "," << packet->getLongitude() << ","
            << packet->getHeading() << "," << packet->getByteLength() << ",";
        for (unsigned i = 0; i < packet->getPointsArraySize(); ++i) {
            const McTrajectoryPoint& p = packet->getPoints(i);
            out << (i ? ";" : "") << p.timeOffset << ":" << p.deltaEast << ":" << p.deltaNorth
                << ":" << p.speed << ":" << p.heading;
        }
        out << "\n";
    }

    request(req, packet);
    mLastSent = simTime();
}

SimTime BaseMcService::generationTimeOf(const McMessage& msg) const
{
    return mTimer->getTimeFor(mTimer->reconstructMilliseconds(msg.getGenerationDeltaTime()));
}

} // namespace artery
