#pragma once

#include "math.hpp"
#include "soft_actuator_core/model.hpp"
#include <cstdint>

namespace soft_actuator::detail {

enum class PositionServoContactState { FREE, CONTACT_SUSPECT, CONTACT, RELEASE, RECOVER };

struct KernelFeedback {
    Vec3 position{};
    Vec3 tip_axis{0.0, 0.0, 1.0};
    std::uint64_t frameSeq = 0;
    double receiveTimeSec = 0.0;
    Vec3 appliedPressureKPa{}; // Last accepted host write, never pressure sensing.
    bool appliedPressureValid = false;
};

struct KernelResult {
    Vec3 position{};
    Vec3 pressure{};
    Vec3 unconstrainedPressure{};
    bool pressureLimited = false;
    bool numericValid = true;
};

// Internal stateful numerical kernel. This header is not an installed API.
// Its deliberately retained legacy names support direct regression auditing.
class Kernel {
public:
    explicit Kernel(const PressureLengthModel& model, double chamberRadiusMm = 12.0);
    void reset(const Vec3& pressure);
    // An exactly equal input triple receives the original +1e-5 third-channel
    // perturbation through this reference, just as ControlPCC did.
    KernelResult openStep(Vec3& pressure, const Vec3& target);
    KernelResult closedStep(const Vec3& target, const KernelFeedback& feedback,
        double nowSec, double pressureAbsLimitKPa = 0.0,
        const pressure_length_model::PressureEnvelope* envelope = nullptr,
        bool contactInferenceEnabled = true);
    Vec3 pressureToLengthMm(Vec3 pressure);
    Vec3 forwardKinematic(Vec3& pressureOrLength, bool pressureFlag);
    Vec3 computeCurrArcFromPccPose(const Vec3& position, const Vec3& tipAxis, double radiusMm);
    void setCurrentPressure(const Vec3& pressure) { currPressure = pressure; }
    const Vec3& currentPressure() const { return currPressure; }

    const double d;
    const double h_0;
    // Estimated actuator lengths and current pressure.
    Vec3 estLength;
    Vec3 currPressure;

    // Position-servo state and debug values.
    bool m_positionServoStateInitialized;
    Vec3 m_positionServoDesiredLengthMm;
    Vec3 m_positionServoDesiredPressureKPa;
    Vec3 m_positionServoActualErrorPccMm;
    double m_positionServoActualErrorNormMm;
    bool m_positionServoFilterInitialized;
    Vec3 m_positionServoFilteredSensorPosPccMm;
    Vec3 m_positionServoFilteredSensorLengthMm;
    Vec3 m_positionServoFeedforwardPressureKPa;
    Vec3 m_positionServoFeedforwardLengthMm;
    // The free trim is integrated only in FREE.  The contact trim is a
    // bounded proportional target used in CONTACT/RELEASE.  The legacy
    // feedback-trim member below is always the trim actually applied.
    Vec3 m_positionServoFreeTrimLengthMm;
    Vec3 m_positionServoContactTrimLengthMm;
    Vec3 m_positionServoFeedbackTrimLengthMm;
    Vec3 m_positionServoPositionResidualPccMm;
    Vec3 m_positionServoLengthResidualMm;
    double m_positionServoPositionResidualNormMm;
    double m_positionServoLengthResidualNormMm;
    double m_positionServoLastStallCheckTime;
    double m_positionServoLastErrorNormMm;
    double m_positionServoLastPositionResidualNormMm;
    double m_positionServoLastLengthResidualNormMm;
    bool m_positionServoStallMonitorInitialized;
    bool m_positionServoNoProgressLikely;
    PositionServoContactState m_positionServoContactState;
    bool m_positionServoHaveFeedbackFrame;
    std::uint64_t m_positionServoLastFeedbackFrameSeq;
    double m_positionServoLastFeedbackReceiveTimeSec;
    double m_positionServoFeedbackDtSec;
    Vec3 m_positionServoLastFeedbackTargetPccMm;
    double m_positionServoTargetSpeedMmPerSec;
    unsigned int m_positionServoContactOnFrames;
    unsigned int m_positionServoContactOffFrames;
    unsigned int m_positionServoRecoverFrames;
    // In contact-family states this means the FREE integrator is frozen; it
    // does not mean pressure output or bounded contact feedback is frozen.
    bool m_positionServoLoadLikely;
    bool m_positionServoFreezeActive;

private:
    PressureLengthModel m_pressureLengthModel;
    const double delta_t = 0.04;
    const double MULTIPLIER = 0.05;
    const double min_neg_pressure = -40.0;
    const double max_pos_pressure = 50.0;
    Vec3 calcForwardPressurePos(Vec3& pressure);
    Vec3 calcForwardPressurePosInverse(Vec3& length);
    double calcKappa(Vec3& length);
    double calcPhi(Vec3& length);
    double calcTheta(Vec3& length);
    Mat3 calcJacobianAnalytical(Vec3& length);
};

} // namespace soft_actuator::detail