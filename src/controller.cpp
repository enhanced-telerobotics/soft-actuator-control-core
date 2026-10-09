#include "soft_actuator_core/controller.hpp"
#include "detail/kernel.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace soft_actuator {
namespace {
using detail::Vec3;
bool finite(const Triple& v) noexcept {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
Vec3 bounded(Vec3 pressure, const StepInput& input) noexcept {
    if (std::isfinite(input.pressure_abs_limit_kpa) && input.pressure_abs_limit_kpa > 0.0)
        for (unsigned i = 0; i < 3; ++i)
            pressure(i) = detail::clamp(pressure(i), -input.pressure_abs_limit_kpa,
                                      input.pressure_abs_limit_kpa);
    if (input.pressure_envelope && pressure_length_model::isValid(*input.pressure_envelope))
        for (unsigned i = 0; i < 3; ++i)
            pressure(i) = detail::clamp(pressure(i), input.pressure_envelope->minimumKPa,
                                      input.pressure_envelope->maximumKPa);
    return pressure;
}
bool outsideEnvelope(const Vec3& p, const StepInput& input) noexcept {
    return input.pressure_envelope &&
        !pressure_length_model::classifyPressureEnvelope(p.data, *input.pressure_envelope).withinEnvelope();
}
bool validEnvelope(const StepInput& input) noexcept {
    if (!input.pressure_envelope) return true;
    const auto& envelope = *input.pressure_envelope;
    if (!pressure_length_model::isValid(envelope)) return false;
    double minimum = std::max(-40.0, envelope.minimumKPa);
    double maximum = std::min(50.0, envelope.maximumKPa);
    if (std::isfinite(input.pressure_abs_limit_kpa) && input.pressure_abs_limit_kpa > 0.0) {
        minimum = std::max(minimum, -input.pressure_abs_limit_kpa);
        maximum = std::min(maximum, input.pressure_abs_limit_kpa);
    }
    return minimum <= maximum;
}
}

struct Controller::Impl {
    detail::Kernel kernel;
    ControllerOptions options;
    Mode mode = Mode::OpenLoop;
    Vec3 pressure{};
    bool calibration_valid = false;
    std::uint64_t calibration_revision = 0;
    bool reset_pending = false;
    bool fallback_active = false;
    unsigned recovery_frames = 0;
    std::uint64_t last_observed_frame = 0;
    FeedbackReason latched_reason = FeedbackReason::None;

    Impl(const PressureLengthModel& model, ControllerOptions value)
        : kernel(model, value.chamber_radius_mm), options(value) {}

    void resetGate() noexcept {
        fallback_active = false;
        recovery_frames = 0;
        last_observed_frame = 0;
        latched_reason = FeedbackReason::None;
    }

    void diagnostics(StepResult& result) const noexcept {
        result.contact_state = static_cast<ContactState>(kernel.m_positionServoContactState);
        result.feedback_trim_mm = kernel.m_positionServoFeedbackTrimLengthMm.data;
        result.feedforward_pressure_kpa = kernel.m_positionServoFeedforwardPressureKPa.data;
        result.feedback_dt_s = kernel.m_positionServoFeedbackDtSec;
    }

    StepResult openStep(const StepInput& input) noexcept {
        StepResult result;
        result.requested_mode = mode;
        pressure = bounded(pressure, input);
        const auto value = kernel.openStep(pressure, Vec3(input.target_pcc_mm));
        const Vec3 error = value.position - Vec3(input.target_pcc_mm);
        const double norm = error.length();
        result.numeric_valid = value.numericValid && finite(input.target_pcc_mm) &&
            detail::isFiniteVector(value.position) && detail::isFiniteVector(value.pressure) &&
            detail::isFiniteVector(pressure) && std::isfinite(norm);
        result.requested_pressure_kpa = (norm >= input.open_loop_tolerance_mm
            ? value.pressure : pressure).data;
        result.pressure_limited = outsideEnvelope(Vec3(result.requested_pressure_kpa), input);
        result.predicted_tip_pcc_mm = value.position.data;
        // Model error in open loop; measured_error_valid stays false.
        result.position_error_mm = (Vec3(input.target_pcc_mm) - value.position).data;
        result.position_error_norm_mm = norm;
        if (!result.numeric_valid) {
            pressure.zero(); // The legacy application's m_initPressure is zero.
            kernel.setCurrentPressure(pressure);
            result.pressure_limited = true;
        } else {
            if (norm >= input.open_loop_tolerance_mm) pressure = value.pressure;
            pressure = bounded(pressure, input);
        }
        result.command_pressure_kpa = pressure.data;
        diagnostics(result);
        return result;
    }
};

Controller::Controller(const PressureLengthModel& model, ControllerOptions options) {
    std::string error;
    if (!validateModel(model, &error)) throw std::invalid_argument(error);
    if (!std::isfinite(options.chamber_radius_mm) || options.chamber_radius_mm <= 0.0 ||
        !std::isfinite(options.feedback_max_age_s) || options.feedback_max_age_s <= 0.0 ||
        !std::isfinite(options.written_command_max_age_s) || options.written_command_max_age_s <= 0.0)
        throw std::invalid_argument("Controller geometry and timeout options must be positive and finite");
    impl_ = std::make_unique<Impl>(model, options);
    impl_->kernel.reset(Vec3{});
}
Controller::~Controller() = default;
Controller::Controller(Controller&&) noexcept = default;
Controller& Controller::operator=(Controller&&) noexcept = default;

void Controller::reset(const ResetState& state) {
    if (!finite(state.pressure_kpa)) throw std::invalid_argument("Reset pressure must be finite");
    for (double p : state.pressure_kpa)
        if (p < -40.0 || p > 50.0) throw std::invalid_argument("Reset pressure outside [-40, 50] kPa");
    auto& s = *impl_;
    s.pressure = Vec3(state.pressure_kpa);
    s.calibration_valid = state.calibration_valid;
    s.calibration_revision = state.calibration_revision;
    s.kernel.reset(s.pressure);
    s.resetGate();
    s.reset_pending = s.mode == Mode::PositionServo;
}

bool Controller::requestMode(Mode mode) noexcept {
    auto& s = *impl_;
    if (mode != Mode::OpenLoop && mode != Mode::PositionServo) return false;
    if (s.mode == mode) return true;
    if (mode == Mode::PositionServo && !s.calibration_valid) return false;
    s.mode = mode;
    s.reset_pending = mode == Mode::PositionServo;
    s.resetGate();
    return true;
}
Mode Controller::requestedMode() const noexcept { return impl_->mode; }

StepResult Controller::step(const StepInput& input) noexcept {
    auto& s = *impl_;
    // Invalid call metadata is rejected without allowing NaNs to reach output.
    if (!std::isfinite(input.now_s) || input.now_s < 0.0 ||
        !std::isfinite(input.open_loop_tolerance_mm) || input.open_loop_tolerance_mm < 0.0 ||
        !validEnvelope(input)) {
        StepResult result;
        result.requested_mode = s.mode;
        result.effective_mode = EffectiveMode::InvalidInput;
        result.numeric_valid = false;
        result.pressure_limited = true;
        result.command_pressure_kpa = s.pressure.data;
        s.diagnostics(result);
        return result;
    }
    if (input.feedback) {
        const auto& f = *input.feedback;
        if (f.calibration_valid && (!s.calibration_valid ||
            f.calibration_revision != s.calibration_revision)) {
            s.reset_pending = true;
        }
        s.calibration_valid = f.calibration_valid;
        s.calibration_revision = f.calibration_revision;
    }
    if (s.mode == Mode::OpenLoop) return s.openStep(input);

    FeedbackReason reason = FeedbackReason::None;
    if (!input.feedback) reason = FeedbackReason::Missing;
    else {
        const auto& f = *input.feedback;
        if (!f.calibration_valid) reason = FeedbackReason::CalibrationInvalid;
        else if (!f.valid || f.frame_sequence == 0 || !finite(f.position_pcc_mm) ||
                 !finite(f.tip_axis_pcc)) reason = FeedbackReason::InvalidFrame;
        else if (!std::isfinite(f.received_at_s) || f.received_at_s <= 0.0 ||
                 input.now_s < f.received_at_s ||
                 input.now_s - f.received_at_s > s.options.feedback_max_age_s)
            reason = FeedbackReason::Stale;
    }
    const bool available = reason == FeedbackReason::None;
    const auto seq = input.feedback ? input.feedback->frame_sequence : 0;
    const bool new_observation = seq != 0 && seq != s.last_observed_frame;
    if (new_observation) s.last_observed_frame = seq;
    bool recovered = false;
    if (!available) {
        // The old main program passes hardFailure=true: immediate fallback.
        s.fallback_active = true;
        s.recovery_frames = 0;
        s.latched_reason = reason;
    } else if (s.fallback_active) {
        if (new_observation && s.recovery_frames < 3) ++s.recovery_frames;
        if (s.recovery_frames >= 3) {
            s.fallback_active = false;
            s.recovery_frames = 0;
            s.latched_reason = FeedbackReason::None;
            recovered = true;
        }
    }
    if (!available || s.fallback_active) {
        auto result = s.openStep(input);
        result.effective_mode = EffectiveMode::OpenLoopFallback;
        result.feedback_reason = s.latched_reason;
        result.feedback_available = available;
        return result;
    }

    StepResult result;
    result.requested_mode = s.mode;
    result.effective_mode = EffectiveMode::PositionServo;
    result.feedback_available = true;
    result.servo_reset = s.reset_pending || recovered;
    s.reset_pending = false;
    s.pressure = bounded(s.pressure, input);
    if (result.servo_reset) s.kernel.reset(s.pressure);
    s.kernel.setCurrentPressure(s.pressure);
    const auto& f = *input.feedback;
    detail::KernelFeedback feedback;
    feedback.position = Vec3(f.position_pcc_mm);
    feedback.tip_axis = Vec3(f.tip_axis_pcc);
    feedback.frameSeq = f.frame_sequence;
    feedback.receiveTimeSec = f.received_at_s;
    if (input.last_written_command) {
        const auto& written = *input.last_written_command;
        const double age = input.now_s - written.written_at_s;
        if (std::isfinite(age) && age >= 0.0 &&
            age <= s.options.written_command_max_age_s && finite(written.pressure_kpa)) {
            feedback.appliedPressureKPa = Vec3(written.pressure_kpa);
            feedback.appliedPressureValid = true;
        }
    }
    result.new_feedback_frame = !s.kernel.m_positionServoHaveFeedbackFrame ||
        f.frame_sequence != s.kernel.m_positionServoLastFeedbackFrameSeq;
    const auto value = s.kernel.closedStep(Vec3(input.target_pcc_mm), feedback, input.now_s,
        input.pressure_abs_limit_kpa,
        input.pressure_envelope ? &*input.pressure_envelope : nullptr,
        input.contact_inference_enabled);
    result.numeric_valid = value.numericValid && finite(input.target_pcc_mm) &&
        detail::isFiniteVector(value.position) && detail::isFiniteVector(value.pressure) &&
        detail::isFiniteVector(value.unconstrainedPressure);
    result.requested_pressure_kpa = value.unconstrainedPressure.data;
    result.predicted_tip_pcc_mm = value.position.data;
    result.pressure_limited = value.pressureLimited || outsideEnvelope(value.unconstrainedPressure, input);
    if (!result.numeric_valid) {
        s.pressure.zero();
        result.pressure_limited = true;
    } else {
        s.pressure = bounded(value.pressure, input);
    }
    s.kernel.setCurrentPressure(s.pressure);
    result.command_pressure_kpa = s.pressure.data;
    result.position_error_mm = s.kernel.m_positionServoActualErrorPccMm.data;
    result.position_error_norm_mm = s.kernel.m_positionServoActualErrorNormMm;
    result.measured_error_valid = result.numeric_valid;
    s.diagnostics(result);
    return result;
}

const char* toString(EffectiveMode value) noexcept {
    switch (value) {
    case EffectiveMode::OpenLoop: return "open_loop";
    case EffectiveMode::PositionServo: return "position_servo";
    case EffectiveMode::OpenLoopFallback: return "open_loop_fallback";
    case EffectiveMode::InvalidInput: return "invalid_input";
    }
    return "unknown";
}
const char* toString(FeedbackReason value) noexcept {
    switch (value) {
    case FeedbackReason::None: return "none";
    case FeedbackReason::Missing: return "missing";
    case FeedbackReason::Stale: return "stale";
    case FeedbackReason::CalibrationInvalid: return "calibration_invalid";
    case FeedbackReason::InvalidFrame: return "invalid_frame";
    }
    return "unknown";
}
const char* toString(ContactState value) noexcept {
    switch (value) {
    case ContactState::Free: return "free";
    case ContactState::ContactSuspect: return "contact_suspect";
    case ContactState::Contact: return "contact";
    case ContactState::Release: return "release";
    case ContactState::Recover: return "recover";
    }
    return "unknown";
}
} // namespace soft_actuator
