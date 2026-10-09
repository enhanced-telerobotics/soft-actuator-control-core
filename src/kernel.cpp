// Extracted from Soft_gripper/ControlPCC.cpp. The numerical update order,
// fixed per-call steps, contact logic, and equal-pressure perturbations are
// deliberately retained for legacy replay parity. This file performs no I/O.
#include "detail/kernel.hpp"
#include <algorithm>
#include <cmath>

namespace soft_actuator::detail {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kPositionServoDeadbandMm = 0.6;
constexpr double kPositionServoMaxPressureStepKPa = 0.25;
constexpr double kPositionServoProportionalGain = 0.01;
constexpr double kPositionServoContactProportionalGain = 0.10;
constexpr double kPositionServoSlowRadiusMm = 3.0;
constexpr bool kPositionServoSensorLowPassEnabled = false;
constexpr double kPositionServoSensorLowPassAlpha = 0.01;
constexpr double kPositionServoLoadPositionResidualOnMm = 3.0;
constexpr double kPositionServoLoadPositionResidualOffMm = 1.5;
constexpr double kPositionServoLoadLengthResidualOnMm = 3.0;
constexpr double kPositionServoLoadLengthResidualOffMm = 1.5;
constexpr double kPositionServoFreeTrimLimitMm = 10.0;
constexpr double kPositionServoContactTrimLimitMm = 1.0;
constexpr double kPositionServoContactTrimRateMmPerSec = 3.0;
constexpr double kPositionServoLoadMaxPressureStepKPa = 0.20;
constexpr double kPositionServoFeedforwardToleranceMm = 0.1;
constexpr double kPositionServoStallCheckIntervalSec = 0.25;
constexpr double kPositionServoMinErrorImprovementMm = 0.10;
constexpr double kPositionServoFirstFeedbackDtSec = 0.02;
constexpr double kPositionServoMinFeedbackDtSec = 0.005;
constexpr double kPositionServoMaxFeedbackDtSec = 0.08;
constexpr double kPositionServoContactDetectionMaxTargetSpeedMmPerSec = 3.0;
constexpr double kPositionServoHardContactResidualMm = 6.0;
constexpr unsigned int kPositionServoContactEnterFrames = 4;
constexpr unsigned int kPositionServoContactExitFrames = 8;
constexpr unsigned int kPositionServoRecoverFrames = 5;

Vec3 clampVector(const Vec3& v, double minValue, double maxValue)
{
    return Vec3(
        clamp(v.x(), minValue, maxValue),
        clamp(v.y(), minValue, maxValue),
        clamp(v.z(), minValue, maxValue));
}

Vec3 rateLimitVector(
    const Vec3& target,
    const Vec3& current,
    double maxStep)
{
    Vec3 limited;
    for (int i = 0; i < 3; ++i) {
        const double delta = clamp(target(i) - current(i), -maxStep, maxStep);
        limited(i) = current(i) + delta;
    }
    return limited;
}

Vec3 clampVectorLength(const Vec3& v, double maxLength)
{
    if (maxLength < 0.0) {
        return Vec3(0.0, 0.0, 0.0);
    }

    Vec3 limited = v;
    if (limited.length() > maxLength && limited.length() > 1e-12) {
        limited.normalize();
        limited *= maxLength;
    }
    return limited;
}

Vec3 slewVectorToward(
    const Vec3& current,
    const Vec3& target,
    double maxStep)
{
    if (!(maxStep > 0.0) || !std::isfinite(maxStep)) {
        return current;
    }

    const Vec3 delta = target - current;
    const double distance = delta.length();
    if (!std::isfinite(distance) || distance <= maxStep || distance <= 1e-12) {
        return target;
    }
    return current + delta * (maxStep / distance);
}

bool isContactFamily(PositionServoContactState state)
{
    return state == PositionServoContactState::CONTACT_SUSPECT ||
        state == PositionServoContactState::CONTACT ||
        state == PositionServoContactState::RELEASE;
}
} // namespace

Kernel::Kernel(const PressureLengthModel& model, double chamberRadiusMm)
    : d(chamberRadiusMm),
      h_0(model.neutralLengthMm),
      estLength(h_0, h_0, h_0),
      currPressure(0.0, 0.0, 0.1),
    m_positionServoStateInitialized(false),
    m_positionServoDesiredLengthMm(h_0, h_0, h_0),
    m_positionServoDesiredPressureKPa(0.0, 0.0, 0.0),
    m_positionServoActualErrorPccMm(0.0, 0.0, 0.0),
    m_positionServoActualErrorNormMm(0.0),
    m_positionServoFilterInitialized(false),
    m_positionServoFilteredSensorPosPccMm(0.0, 0.0, h_0),
    m_positionServoFilteredSensorLengthMm(h_0, h_0, h_0),
    m_positionServoFeedforwardPressureKPa(0.0, 0.0, 0.0),
    m_positionServoFeedforwardLengthMm(h_0, h_0, h_0),
    m_positionServoFreeTrimLengthMm(0.0, 0.0, 0.0),
    m_positionServoContactTrimLengthMm(0.0, 0.0, 0.0),
    m_positionServoFeedbackTrimLengthMm(0.0, 0.0, 0.0),
    m_positionServoPositionResidualPccMm(0.0, 0.0, 0.0),
    m_positionServoLengthResidualMm(0.0, 0.0, 0.0),
    m_positionServoPositionResidualNormMm(0.0),
    m_positionServoLengthResidualNormMm(0.0),
    m_positionServoLastStallCheckTime(0.0),
    m_positionServoLastErrorNormMm(0.0),
    m_positionServoLastPositionResidualNormMm(0.0),
    m_positionServoLastLengthResidualNormMm(0.0),
    m_positionServoStallMonitorInitialized(false),
    m_positionServoNoProgressLikely(false),
    m_positionServoContactState(PositionServoContactState::FREE),
    m_positionServoHaveFeedbackFrame(false),
    m_positionServoLastFeedbackFrameSeq(0),
    m_positionServoLastFeedbackReceiveTimeSec(0.0),
    m_positionServoFeedbackDtSec(0.0),
    m_positionServoLastFeedbackTargetPccMm(0.0, 0.0, h_0),
    m_positionServoTargetSpeedMmPerSec(0.0),
    m_positionServoContactOnFrames(0),
    m_positionServoContactOffFrames(0),
    m_positionServoRecoverFrames(0),
    m_positionServoLoadLikely(false),
    m_positionServoFreezeActive(false),
    m_pressureLengthModel(model)
{
}

Vec3 Kernel::calcForwardPressurePos(Vec3& x)
{
    Vec3 result;
    for (int i = 0; i < 3; ++i)
    {
        double P = x(i);  // kPa

        result(i) = pressure_length_model::lengthForPressureKPa(
            m_pressureLengthModel,
            P); // mm
    }
    return result;
}

//----------------------------------------------------------
// Length to pressure
//----------------------------------------------------------
Vec3 Kernel::calcForwardPressurePosInverse(Vec3& x)
{
    Vec3 result;
    for (int i = 0; i < 3; ++i)
    {
        double L = x(i);  // mm
        double P = 0.0;
        if (!pressure_length_model::pressureForLengthMm(
            m_pressureLengthModel,
            L,
            P)) {
            P = 0.0;
        }
        result(i) = P; // kPa
    }
    return result;
}

//----------------------------------------------------------
// PCC: Kappa 
//----------------------------------------------------------
double Kernel::calcKappa(Vec3& length)
{
    double sumSq = length(0) * length(0) + length(1) * length(1) + length(2) * length(2);
    double crossTerm = length(0) * length(1) + length(1) * length(2) + length(0) * length(2);
    return 2.0 * std::sqrt(sumSq - crossTerm) / (d * (length(0) + length(1) + length(2)));
}

//----------------------------------------------------------
// PCC: Phi
//----------------------------------------------------------
double Kernel::calcPhi(Vec3& length)
{
    double numerator = std::sqrt(3.0) * (length(1) + length(2) - 2.0 * length(0));
    double denominator = 3.0 * (length(1) - length(2));
    return std::atan2(numerator, denominator);
}

//----------------------------------------------------------
// PCC: Theta
//----------------------------------------------------------
double Kernel::calcTheta(Vec3& length)
{
    double sumSq = length(0) * length(0) + length(1) * length(1) + length(2) * length(2);
    double crossTerm = length(0) * length(1) + length(1) * length(2) + length(0) * length(2);
    return 2.0 * std::sqrt(sumSq - crossTerm) / (d * 3.0);
}

//----------------------------------------------------------
// Forward Kinematic PCC
//----------------------------------------------------------
Vec3 Kernel::forwardKinematic(Vec3& pressure, bool pressure_flag)
{
    Vec3 length = pressure;
    if (pressure_flag)
    {
        length = calcForwardPressurePos(pressure);
    }

    double phi = calcPhi(length);
    double theta = calcTheta(length);
    double kappa = calcKappa(length);

    double x_out = std::cos(phi) * (1 - std::cos(theta)) / kappa;
    double y_out = std::sin(phi) * (1 - std::cos(theta)) / kappa;
    double z_out = std::sin(theta) / kappa;

    return Vec3(x_out, y_out, z_out);
}

Vec3 Kernel::pressureToLengthMm(Vec3 pressure)
{
    return calcForwardPressurePos(pressure);
}

void Kernel::reset(const Vec3& pressure)
{
    currPressure = pressure;
    estLength = calcForwardPressurePos(currPressure);
    m_positionServoStateInitialized = true;
    m_positionServoDesiredLengthMm = estLength;
    m_positionServoDesiredPressureKPa = currPressure;
    m_positionServoActualErrorPccMm.zero();
    m_positionServoActualErrorNormMm = 0.0;
    m_positionServoFilterInitialized = false;
    m_positionServoFilteredSensorPosPccMm.set(0.0, 0.0, h_0);
    m_positionServoFilteredSensorLengthMm.set(h_0, h_0, h_0);
    m_positionServoFeedforwardPressureKPa = currPressure;
    m_positionServoFeedforwardLengthMm = estLength;
    m_positionServoFreeTrimLengthMm.zero();
    m_positionServoContactTrimLengthMm.zero();
    m_positionServoFeedbackTrimLengthMm.zero();
    m_positionServoPositionResidualPccMm.zero();
    m_positionServoLengthResidualMm.zero();
    m_positionServoPositionResidualNormMm = 0.0;
    m_positionServoLengthResidualNormMm = 0.0;
    m_positionServoLastStallCheckTime = 0.0;
    m_positionServoLastErrorNormMm = 0.0;
    m_positionServoLastPositionResidualNormMm = 0.0;
    m_positionServoLastLengthResidualNormMm = 0.0;
    m_positionServoStallMonitorInitialized = false;
    m_positionServoNoProgressLikely = false;
    m_positionServoContactState = PositionServoContactState::FREE;
    m_positionServoHaveFeedbackFrame = false;
    m_positionServoLastFeedbackFrameSeq = 0;
    m_positionServoLastFeedbackReceiveTimeSec = 0.0;
    m_positionServoFeedbackDtSec = 0.0;
    m_positionServoLastFeedbackTargetPccMm.set(0.0, 0.0, h_0);
    m_positionServoTargetSpeedMmPerSec = 0.0;
    m_positionServoContactOnFrames = 0;
    m_positionServoContactOffFrames = 0;
    m_positionServoRecoverFrames = 0;
    m_positionServoLoadLikely = false;
    m_positionServoFreezeActive = false;
}

Mat3 Kernel::calcJacobianAnalytical(Vec3& length)
{
    double q1 = length(0);
    double q2 = length(1);
    double q3 = length(2);

    double k = calcKappa(length);    // kappa(q)
    double ph = calcPhi(length);     // phi(q)
    double l = (q1 + q2 + q3) / 3.0;

    //----------------------J_h Matrix (3x3)----------------------
    double kl = k * l;
    double cos_ph = std::cos(ph);
    double sin_ph = std::sin(ph);
    double cos_kl = std::cos(kl);
    double sin_kl = std::sin(kl);

    double J_h11 = (cos_ph * (kl * sin_kl + cos_kl - 1.0)) / (k * k);
    double J_h12 = -(sin_ph * (1.0 - cos_kl)) / k;
    double J_h13 = cos_ph * sin_kl;

    double J_h21 = (sin_ph * (kl * sin_kl + cos_kl - 1.0)) / (k * k);
    double J_h22 = (cos_ph * (1.0 - cos_kl)) / k;
    double J_h23 = sin_ph * sin_kl;

    double J_h31 = (l * cos_kl) / k - (sin_kl / (k * k));
    double J_h32 = 0.0;
    double J_h33 = cos_kl;

    //----------------------J_g Matrix (3x3)----------------------
    double q_sum = (q1 + q2 + q3);
    double q_sqrt = std::sqrt(q1 * q1 + q2 * q2 + q3 * q3 - q1 * q2 - q2 * q3 - q1 * q3);

    double J_g11 = (3.0 * (q1 * q2 + q1 * q3 - q2 * q2 - q3 * q3)) /
        ((double)d * q_sum * q_sum * q_sqrt);
    double J_g12 = -(3.0 * (q1 * q1 - q1 * q2 - q2 * q3 + q3 * q3)) /
        ((double)d * q_sum * q_sum * q_sqrt);
    double J_g13 = -(3.0 * (q1 * q1 - q1 * q3 - q2 * q3 + q2 * q2)) /
        ((double)d * q_sum * q_sum * q_sqrt);

    double J_g21 = (std::sqrt(3.0) * (q3 - q2)) / (2.0 * q_sqrt * q_sqrt);
    double J_g22 = (std::sqrt(3.0) * (q1 - q3)) / (2.0 * q_sqrt * q_sqrt);
    double J_g23 = (std::sqrt(3.0) * (q2 - q1)) / (2.0 * q_sqrt * q_sqrt);

    double J_g31 = 1.0 / 3.0;
    double J_g32 = 1.0 / 3.0;
    double J_g33 = 1.0 / 3.0;

    //--------------------------------------------------------------
    // Mat3::set(row0col0, row0col1, row0col2, 
    //                row1col0, row1col1, row1col2,
    //                row2col0, row2col1, row2col2)
    //--------------------------------------------------------------
    Mat3 J_h, J_g;
    J_h.set(
        J_h11, J_h12, J_h13,
        J_h21, J_h22, J_h23,
        J_h31, J_h32, J_h33
    );

    J_g.set(
        J_g11, J_g12, J_g13,
        J_g21, J_g22, J_g23,
        J_g31, J_g32, J_g33
    );

    //----------------------J_result Matrix (3x3) = J_h * J_g-----------
    Mat3 J_result = J_h * J_g;
    return J_result;
}


Vec3 Kernel::computeCurrArcFromPccPose(
    const Vec3& posPccMm,
    const Vec3& tipAxisPcc,
    double radiusMm)
{
    const double eps = 1e-6;

    const double xPos = posPccMm.x();
    const double yPos = posPccMm.y();
    const double zPos = posPccMm.z();

    Vec3 axis = tipAxisPcc;
    if (axis.length() < eps) {
        axis.set(0.0, 0.0, 1.0);
    }
    else {
        axis.normalize();
    }

    const double tz = clamp(axis.z(), -1.0, 1.0);
    [[maybe_unused]] double kappa;
    double phi;
    double centerLength = zPos;
    const double xSqySq = xPos * xPos + yPos * yPos;
    const double rXY = std::sqrt(xSqySq);
    const bool nearVertical = (xSqySq < eps);
    double theta = nearVertical ? std::acos(tz) : 2.0 * std::atan2(rXY, zPos);

    if (std::fabs(theta) < eps)
    {
        kappa = 0.0;
        phi = 0.0;
        theta = 0.0;
        centerLength = zPos;
    }
    else
    {
        // The corrected tip position defines the PCC arc plane and bend magnitude.
        // Marker orientation is kept only as the near-vertical fallback.
        phi = nearVertical ? std::atan2(axis.y(), axis.x()) : std::atan2(yPos, xPos);

        double r_xy = rXY / (1.0 - std::cos(theta));
        double r_z = zPos / std::sin(theta);

        double rVal = (std::fabs(std::sin(theta)) > eps) ? r_z : r_xy;
        kappa = (std::fabs(rVal) < eps) ? 0.0 : (1.0 / rVal);
        centerLength = theta * rVal;
    }

    // Chamber phase is matched to calcPhi()/forwardKinematic().
    static const double anglesDeg[3] = { 90.0, 210.0, 330.0 };

    double l[3];
    for (int i = 0; i < 3; i++)
    {
        double angle_rad = anglesDeg[i] * kPi / 180.0;
        double delta_phi = angle_rad - phi;
        l[i] = centerLength - theta * radiusMm * std::cos(delta_phi);
    }

    bool anyNaN = (std::isnan(l[0]) || std::isnan(l[1]) || std::isnan(l[2]));

    if (anyNaN || nearVertical)
    {
        l[0] = zPos;
        l[1] = zPos;
        l[2] = zPos;
    }

    return Vec3(l[0], l[1], l[2]);
}

KernelResult Kernel::openStep(Vec3& pressure,
    const Vec3& pos)
{
    // Give a small disturbe to aviod singularity 
    if ((pressure(0) == pressure(1)) && (pressure(1) == pressure(2)))
    {
        pressure(2) += 0.00001;
    }

    Vec3 current_pos = forwardKinematic(pressure, true);
    Vec3 current_length = calcForwardPressurePos(pressure);

    //Pick a method to get Jacobian: Numerical or Analytical 
    //Mat3 J = calcJacobianNumerical(current_length);
    Mat3 J = calcJacobianAnalytical(current_length);

    Mat3 J_inverse = J;
    J_inverse.invert();

    // calculate error
    Vec3 error = pos - current_pos;
    double errLen = error.length();

    // target velocity 
    Vec3 target_vel(0, 0, 0);
    if (errLen > 1e-12)
    {
        target_vel = (error / errLen) * MULTIPLIER;
    }

    // length changing rate
    Vec3 l_dot = J_inverse * target_vel;
    Vec3 new_length = current_length + l_dot * delta_t;

    // new position  
    Vec3 new_pos = forwardKinematic(new_length, false);
    // new pressure
    Vec3 new_pressure = calcForwardPressurePosInverse(new_length);

    const bool numericValid = isFiniteVector(current_pos) &&
        isFiniteVector(current_length) && isFiniteVector(new_pos) &&
        isFiniteVector(new_pressure);
    return { new_pos, new_pressure, new_pressure, false, numericValid };
}

KernelResult Kernel::closedStep(
    const Vec3& targetPos,
    const KernelFeedback& sample,
    double nowSec,
    double pressureAbsLimitKPa,
    const pressure_length_model::PressureEnvelope* pressureEnvelope,
	bool contactInferenceEnabled)
{
    const bool profilePressureLimitEnabled =
        std::isfinite(pressureAbsLimitKPa) && pressureAbsLimitKPa > 0.0;
    double effectiveMinPressureKPa = profilePressureLimitEnabled
        ? std::max(min_neg_pressure, -pressureAbsLimitKPa)
        : min_neg_pressure;
    double effectiveMaxPressureKPa = profilePressureLimitEnabled
        ? std::min(max_pos_pressure, pressureAbsLimitKPa)
        : max_pos_pressure;
    if (pressureEnvelope &&
        pressure_length_model::isValid(*pressureEnvelope)) {
        effectiveMinPressureKPa = (std::max)(
            effectiveMinPressureKPa,
            pressureEnvelope->minimumKPa);
        effectiveMaxPressureKPa = (std::min)(
            effectiveMaxPressureKPa,
            pressureEnvelope->maximumKPa);
    }
    currPressure = clampVector(
        currPressure,
        effectiveMinPressureKPa,
        effectiveMaxPressureKPa);

    if (!m_positionServoStateInitialized) {
        reset(currPressure);
    }

    Vec3 rawSensorPos = sample.position;
    const Vec3 fallbackSensorPos = m_positionServoFilterInitialized
        ? m_positionServoFilteredSensorPosPccMm
        : rawSensorPos;
    if (!isFiniteVector(targetPos)) {
        return { fallbackSensorPos, currPressure, currPressure, false, false };
    }

    // A repeated MT snapshot must never change the filter, residual detector,
    // contact state, or feedback trim.  This is the key time-base separation:
    // actuator feedforward still runs below, while measurement feedback does
    // not get integrated at the haptics-loop frequency.
    const bool newFeedbackFrame = sample.frameSeq != 0 &&
        (!m_positionServoHaveFeedbackFrame ||
            sample.frameSeq != m_positionServoLastFeedbackFrameSeq);

    if (newFeedbackFrame) {
        if (!isFiniteVector(rawSensorPos)) {
            return { fallbackSensorPos, currPressure, currPressure, false, false };
        }

        const Vec3 sensorLength = computeCurrArcFromPccPose(sample.position, sample.tip_axis, d);
        if (!isFiniteVector(sensorLength)) {
            return { rawSensorPos, currPressure, currPressure, false, false };
        }

        double feedbackTime = sample.receiveTimeSec;
        if (!std::isfinite(feedbackTime) || feedbackTime <= 0.0) {
            feedbackTime = nowSec;
        }
        double feedbackDt = kPositionServoFirstFeedbackDtSec;
        if (m_positionServoHaveFeedbackFrame) {
            const double rawDt =
                feedbackTime - m_positionServoLastFeedbackReceiveTimeSec;
            if (std::isfinite(rawDt) && rawDt > 0.0) {
                feedbackDt = clamp(
                    rawDt,
                    kPositionServoMinFeedbackDtSec,
                    kPositionServoMaxFeedbackDtSec);
            }
        }
        m_positionServoFeedbackDtSec = feedbackDt;
        if (m_positionServoHaveFeedbackFrame && feedbackDt > 0.0) {
            m_positionServoTargetSpeedMmPerSec =
                (targetPos - m_positionServoLastFeedbackTargetPccMm).length() /
                feedbackDt;
        }
        else {
            m_positionServoTargetSpeedMmPerSec = 0.0;
        }
        m_positionServoLastFeedbackTargetPccMm = targetPos;

        if (!m_positionServoFilterInitialized) {
            m_positionServoFilteredSensorPosPccMm = rawSensorPos;
            m_positionServoFilteredSensorLengthMm = sensorLength;
            m_positionServoFilterInitialized = true;
        }
        else if (kPositionServoSensorLowPassEnabled) {
            // The filter is intentionally updated at the tracker sample rate,
            // never at the haptics-loop rate.
            m_positionServoFilteredSensorPosPccMm =
                m_positionServoFilteredSensorPosPccMm *
                (1.0 - kPositionServoSensorLowPassAlpha) +
                rawSensorPos * kPositionServoSensorLowPassAlpha;
            m_positionServoFilteredSensorLengthMm =
                m_positionServoFilteredSensorLengthMm *
                (1.0 - kPositionServoSensorLowPassAlpha) +
                sensorLength * kPositionServoSensorLowPassAlpha;
        }
        else {
            m_positionServoFilteredSensorPosPccMm = rawSensorPos;
            m_positionServoFilteredSensorLengthMm = sensorLength;
        }

        const Vec3 controlSensorPos =
            m_positionServoFilteredSensorPosPccMm;
        const Vec3 controlSensorLength =
            m_positionServoFilteredSensorLengthMm;
        const Vec3 error = targetPos - controlSensorPos;
        const double errLen = error.length();
        m_positionServoActualErrorPccMm = error;
        m_positionServoActualErrorNormMm = errLen;

        // Use the last pressure successfully written by the host when it is
        // available.  This is still an estimate of chamber pressure, but it is
        // materially better than comparing an MT frame to an unsent request.
        Vec3 modelPressure = clampVector(
            sample.appliedPressureValid ? sample.appliedPressureKPa : currPressure,
            effectiveMinPressureKPa,
            effectiveMaxPressureKPa);
        Vec3 currentCommandLength = calcForwardPressurePos(modelPressure);
        if (!isFiniteVector(currentCommandLength)) {
            return { rawSensorPos, currPressure, currPressure, false, false };
        }

        Vec3 pressureForModel = modelPressure;
        if (std::fabs(pressureForModel.x() - pressureForModel.y()) < 1e-12 &&
            std::fabs(pressureForModel.y() - pressureForModel.z()) < 1e-12) {
            pressureForModel(2) += 0.00001;
        }
        const Vec3 pressureModelTip =
            forwardKinematic(pressureForModel, true);
        if (!isFiniteVector(pressureModelTip)) {
            return { rawSensorPos, currPressure, currPressure, false, false };
        }

        // Formal residuals used for force-sensor-free contact inference:
        // r_x = measured tip - model tip(applied serial pressure),
        // r_l = sensor-equivalent length - model length(applied serial pressure).
        // Both must agree before a contact transition can start.
        m_positionServoPositionResidualPccMm =
            controlSensorPos - pressureModelTip;
        m_positionServoLengthResidualMm =
            controlSensorLength - currentCommandLength;
        m_positionServoPositionResidualNormMm =
            m_positionServoPositionResidualPccMm.length();
        m_positionServoLengthResidualNormMm =
            m_positionServoLengthResidualMm.length();

        // Do not classify expected pneumatic/transient lag as contact.  The
        // residual must persist while tracking error fails to improve over a
        // real-time window, rather than merely being large for one MT frame.
        if (!m_positionServoStallMonitorInitialized) {
            m_positionServoLastStallCheckTime = feedbackTime;
            m_positionServoLastErrorNormMm = errLen;
            m_positionServoLastPositionResidualNormMm =
                m_positionServoPositionResidualNormMm;
            m_positionServoLastLengthResidualNormMm =
                m_positionServoLengthResidualNormMm;
            m_positionServoStallMonitorInitialized = true;
            m_positionServoNoProgressLikely = false;
        }
        else if (feedbackTime - m_positionServoLastStallCheckTime >=
            kPositionServoStallCheckIntervalSec) {
            const double errorImprovementMm =
                m_positionServoLastErrorNormMm - errLen;
            m_positionServoNoProgressLikely =
                errLen >= kPositionServoDeadbandMm &&
                errorImprovementMm < kPositionServoMinErrorImprovementMm;
            m_positionServoLastStallCheckTime = feedbackTime;
            m_positionServoLastErrorNormMm = errLen;
            m_positionServoLastPositionResidualNormMm =
                m_positionServoPositionResidualNormMm;
            m_positionServoLastLengthResidualNormMm =
                m_positionServoLengthResidualNormMm;
        }
        if (errLen < kPositionServoDeadbandMm) {
            m_positionServoNoProgressLikely = false;
        }

        const bool residualHigh =
            m_positionServoPositionResidualNormMm >
                kPositionServoLoadPositionResidualOnMm &&
            m_positionServoLengthResidualNormMm >
                kPositionServoLoadLengthResidualOnMm;
        const bool hardResidualHigh =
            m_positionServoPositionResidualNormMm >
                kPositionServoHardContactResidualMm &&
            m_positionServoLengthResidualNormMm >
                kPositionServoHardContactResidualMm;
        // The former implementation used OR here, so one noisy residual could
        // immediately clear a loaded state.  Release now requires both
        // independent residuals to be below their hysteretic thresholds.
        const bool residualLow =
            m_positionServoPositionResidualNormMm <
                kPositionServoLoadPositionResidualOffMm &&
            m_positionServoLengthResidualNormMm <
                kPositionServoLoadLengthResidualOffMm;
        // With no force sensor, a fast target sweep is indistinguishable from
        // pneumatic phase lag using a static PCC residual alone.  Normal
        // contact entry therefore waits for a quasi-static target; a much
        // larger residual still takes the conservative path immediately.
        const bool targetQuasiStatic =
            m_positionServoTargetSpeedMmPerSec <=
            kPositionServoContactDetectionMaxTargetSpeedMmPerSec;
        const bool contactEvidence = contactInferenceEnabled &&
			(hardResidualHigh ||
            (residualHigh && m_positionServoNoProgressLikely &&
				targetQuasiStatic));

        Vec3 jacobianLength = controlSensorLength;
        if (std::fabs(jacobianLength.x() - jacobianLength.y()) < 1e-9 &&
            std::fabs(jacobianLength.y() - jacobianLength.z()) < 1e-9) {
            jacobianLength(2) += 0.00001;
        }
        Mat3 J = calcJacobianAnalytical(jacobianLength);
        Mat3 JInv = J;
        JInv.invert();

        auto resolvedLengthForTipCommand =
            [&JInv](const Vec3& tipCommand) -> Vec3
        {
            const Vec3 lengthCommand = JInv * tipCommand;
            return isFiniteVector(lengthCommand)
                ? lengthCommand
                : Vec3(0.0, 0.0, 0.0);
        };

        auto updateContactTrim = [&]()
        {
            // Contact correction is a bounded P target, not an integrator.
            // The slew limiter keeps entry, release, and trim-limit changes
            // bumpless even if the free-space integral had accumulated.
            const Vec3 targetTrim = clampVectorLength(
                resolvedLengthForTipCommand(
                    error * kPositionServoContactProportionalGain),
                kPositionServoContactTrimLimitMm);
            m_positionServoContactTrimLengthMm = slewVectorToward(
                m_positionServoContactTrimLengthMm,
                targetTrim,
                kPositionServoContactTrimRateMmPerSec * feedbackDt);
            m_positionServoFeedbackTrimLengthMm =
                m_positionServoContactTrimLengthMm;
        };

        switch (m_positionServoContactState) {
        case PositionServoContactState::FREE:
            if (contactEvidence) {
                m_positionServoContactState =
                    PositionServoContactState::CONTACT_SUSPECT;
                m_positionServoContactOnFrames = 1;
                m_positionServoContactOffFrames = 0;
            }
            else if (errLen >= kPositionServoDeadbandMm) {
                Vec3 targetVel =
                    error * kPositionServoProportionalGain;
                const double slowScale = clamp(
                    errLen / kPositionServoSlowRadiusMm,
                    0.0,
                    1.0);
                targetVel *= slowScale;
                if (targetVel.length() > MULTIPLIER) {
                    targetVel.normalize();
                    targetVel *= MULTIPLIER;
                }
                m_positionServoFreeTrimLengthMm +=
                    resolvedLengthForTipCommand(targetVel) * feedbackDt;
                m_positionServoFreeTrimLengthMm = clampVectorLength(
                    m_positionServoFreeTrimLengthMm,
                    kPositionServoFreeTrimLimitMm);
                m_positionServoFeedbackTrimLengthMm =
                    m_positionServoFreeTrimLengthMm;
                m_positionServoContactTrimLengthMm =
                    m_positionServoFreeTrimLengthMm;
            }
            break;

        case PositionServoContactState::CONTACT_SUSPECT:
            // Freeze only the integral while the evidence is confirmed.  A
            // single transient frame returns directly to FREE with no jump.
            if (contactEvidence) {
                ++m_positionServoContactOnFrames;
                if (m_positionServoContactOnFrames >=
                    kPositionServoContactEnterFrames) {
                    m_positionServoContactState =
                        PositionServoContactState::CONTACT;
                    m_positionServoContactTrimLengthMm =
                        m_positionServoFeedbackTrimLengthMm;
                    m_positionServoContactOffFrames = 0;
                }
            }
            else {
                m_positionServoContactState =
                    PositionServoContactState::FREE;
                m_positionServoContactOnFrames = 0;
                m_positionServoFreeTrimLengthMm =
                    m_positionServoFeedbackTrimLengthMm;
                m_positionServoContactTrimLengthMm =
                    m_positionServoFeedbackTrimLengthMm;
            }
            break;

        case PositionServoContactState::CONTACT:
            updateContactTrim();
            if (residualLow) {
                m_positionServoContactState =
                    PositionServoContactState::RELEASE;
                m_positionServoContactOffFrames = 1;
            }
            break;

        case PositionServoContactState::RELEASE:
            updateContactTrim();
            if (residualHigh) {
                m_positionServoContactState =
                    PositionServoContactState::CONTACT;
                m_positionServoContactOffFrames = 0;
            }
            else if (residualLow) {
                ++m_positionServoContactOffFrames;
                if (m_positionServoContactOffFrames >=
                    kPositionServoContactExitFrames) {
                    m_positionServoContactState =
                        PositionServoContactState::RECOVER;
                    m_positionServoRecoverFrames = 0;
                    m_positionServoFreeTrimLengthMm =
                        m_positionServoFeedbackTrimLengthMm;
                    m_positionServoContactTrimLengthMm =
                        m_positionServoFeedbackTrimLengthMm;
                }
            }
            else {
                m_positionServoContactOffFrames = 0;
            }
            break;

        case PositionServoContactState::RECOVER:
            // Transfer the current trim before re-enabling integration.  This
            // makes contact release bumpless rather than resetting to zero.
            m_positionServoFreeTrimLengthMm =
                m_positionServoFeedbackTrimLengthMm;
            m_positionServoContactTrimLengthMm =
                m_positionServoFeedbackTrimLengthMm;
            if (residualHigh) {
                m_positionServoContactState =
                    PositionServoContactState::CONTACT;
                m_positionServoContactOffFrames = 0;
                m_positionServoRecoverFrames = 0;
            }
            else if (residualLow) {
                ++m_positionServoRecoverFrames;
                if (m_positionServoRecoverFrames >=
                    kPositionServoRecoverFrames) {
                    m_positionServoContactState =
                        PositionServoContactState::FREE;
                    m_positionServoContactOnFrames = 0;
                    m_positionServoContactOffFrames = 0;
                }
            }
            else {
                m_positionServoRecoverFrames = 0;
            }
            break;
        }

        m_positionServoLoadLikely =
            isContactFamily(m_positionServoContactState);
        m_positionServoFreezeActive =
            isContactFamily(m_positionServoContactState);
        m_positionServoHaveFeedbackFrame = true;
        m_positionServoLastFeedbackFrameSeq = sample.frameSeq;
        m_positionServoLastFeedbackReceiveTimeSec = feedbackTime;
    }

    Vec3 feedforwardPressureWork = m_positionServoFeedforwardPressureKPa;
    Vec3 feedforwardTarget = targetPos;
    const KernelResult feedforwardResult = openStep(
        feedforwardPressureWork,
        feedforwardTarget);
    Vec3 feedforwardModelPos = feedforwardResult.position;
    const Vec3 unconstrainedFeedforwardPressure =
        feedforwardResult.pressure;
    bool pressureLimited = false;
    for (int i = 0; i < 3; ++i) {
        if (unconstrainedFeedforwardPressure(i) < effectiveMinPressureKPa ||
            unconstrainedFeedforwardPressure(i) > effectiveMaxPressureKPa) {
            pressureLimited = true;
        }
    }
    Vec3 feedforwardNextPressure =
        clampVector(
            unconstrainedFeedforwardPressure,
            effectiveMinPressureKPa,
            effectiveMaxPressureKPa);
    if (!isFiniteVector(feedforwardModelPos) ||
        !isFiniteVector(feedforwardNextPressure)) {
        return { fallbackSensorPos, currPressure, currPressure, false, false };
    }

    const double feedforwardError = (feedforwardModelPos - targetPos).length();
    if (feedforwardError >= kPositionServoFeedforwardToleranceMm) {
        m_positionServoFeedforwardPressureKPa = feedforwardNextPressure;
    }
    m_positionServoFeedforwardPressureKPa =
        clampVector(
            m_positionServoFeedforwardPressureKPa,
            effectiveMinPressureKPa,
            effectiveMaxPressureKPa);
    m_positionServoFeedforwardLengthMm =
        calcForwardPressurePos(m_positionServoFeedforwardPressureKPa);
    if (!isFiniteVector(m_positionServoFeedforwardLengthMm)) {
        return { fallbackSensorPos, currPressure, currPressure, false, false };
    }

    Vec3 desiredLength =
        m_positionServoFeedforwardLengthMm + m_positionServoFeedbackTrimLengthMm;
    if (!isFiniteVector(desiredLength)) {
        return { fallbackSensorPos, currPressure, currPressure, false, false };
    }
	// Preserve the controller's complete pre-limit request for realtime safety
	// reporting.  The previous value exposed only feedforward pressure and could
	// hide a feedback trim that drove the final request outside the run envelope.
	bool numericValid = true;
	Vec3 unconstrainedDesiredPressure =
		calcForwardPressurePosInverse(desiredLength);
	if (!isFiniteVector(unconstrainedDesiredPressure)) {
		unconstrainedDesiredPressure = unconstrainedFeedforwardPressure;
		pressureLimited = true;
		numericValid = false;
	}

    Vec3 minPressureVec(
        effectiveMinPressureKPa,
        effectiveMinPressureKPa,
        effectiveMinPressureKPa);
    Vec3 maxPressureVec(
        effectiveMaxPressureKPa,
        effectiveMaxPressureKPa,
        effectiveMaxPressureKPa);
    Vec3 minLength = calcForwardPressurePos(minPressureVec);
    Vec3 maxLength = calcForwardPressurePos(maxPressureVec);
    for (int i = 0; i < 3; ++i) {
        if (desiredLength(i) < minLength(i) || desiredLength(i) > maxLength(i)) {
            pressureLimited = true;
        }
        desiredLength(i) = clamp(desiredLength(i), minLength(i), maxLength(i));
    }
    m_positionServoDesiredLengthMm = desiredLength;

    Vec3 desiredPressure = calcForwardPressurePosInverse(desiredLength);
    if (!isFiniteVector(desiredPressure)) {
        return { fallbackSensorPos, currPressure, currPressure, false, false };
    }

    desiredPressure =
        clampVector(
            desiredPressure,
            effectiveMinPressureKPa,
            effectiveMaxPressureKPa);
    const double maxPressureStep = isContactFamily(
        m_positionServoContactState)
        ? kPositionServoLoadMaxPressureStepKPa
        : kPositionServoMaxPressureStepKPa;
    desiredPressure =
        rateLimitVector(desiredPressure, currPressure, maxPressureStep);
    desiredPressure =
        clampVector(
            desiredPressure,
            effectiveMinPressureKPa,
            effectiveMaxPressureKPa);

    currPressure = desiredPressure;
    m_positionServoDesiredPressureKPa = desiredPressure;
    estLength = calcForwardPressurePos(desiredPressure);

    Vec3 newModelPos = forwardKinematic(estLength, false);
    if (!isFiniteVector(newModelPos)) {
        newModelPos = fallbackSensorPos;
		numericValid = false;
    }

    return {
        newModelPos,
        desiredPressure,
		unconstrainedDesiredPressure,
        pressureLimited,
		numericValid };
}

} // namespace soft_actuator::detail
