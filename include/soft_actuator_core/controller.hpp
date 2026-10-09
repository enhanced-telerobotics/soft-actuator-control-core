#pragma once

#include "soft_actuator_core/model.hpp"
#include <array>
#include <cstdint>
#include <memory>
#include <optional>

namespace soft_actuator {

using Triple = std::array<double, 3>;
using PressureEnvelope = pressure_length_model::PressureEnvelope;

enum class Mode { OpenLoop, PositionServo };
enum class EffectiveMode { OpenLoop, PositionServo, OpenLoopFallback, InvalidInput };
enum class FeedbackReason { None, Missing, Stale, CalibrationInvalid, InvalidFrame };
enum class ContactState { Free, ContactSuspect, Contact, Release, Recover };

struct TipFeedback {
    Triple position_pcc_mm{};
    Triple tip_axis_pcc{0.0, 0.0, 1.0};
    std::uint64_t frame_sequence = 0;
    double received_at_s = 0.0;
    bool valid = true;
    bool calibration_valid = true;
    std::uint64_t calibration_revision = 0;
};

// A successful host/driver write, NOT a measured chamber pressure or device ACK.
struct WrittenCommand {
    Triple pressure_kpa{};
    std::uint64_t sequence = 0;
    double written_at_s = 0.0;
};

struct ControllerOptions {
    double chamber_radius_mm = 12.0;
    double feedback_max_age_s = 0.10;
    double written_command_max_age_s = 0.15;
};

struct ResetState {
    Triple pressure_kpa{};
    bool calibration_valid = false;
    std::uint64_t calibration_revision = 0;
};

struct StepInput {
    Triple target_pcc_mm{};
    double now_s = 0.0; // Caller-owned, monotonic clock; same domain as input times.
    std::optional<TipFeedback> feedback;
    std::optional<WrittenCommand> last_written_command;
    double open_loop_tolerance_mm = 0.05;
    double pressure_abs_limit_kpa = 0.0; // Zero disables the symmetric profile limit.
    std::optional<PressureEnvelope> pressure_envelope;
    bool contact_inference_enabled = true;
};

struct StepResult {
    Mode requested_mode = Mode::OpenLoop;
    EffectiveMode effective_mode = EffectiveMode::OpenLoop;
    FeedbackReason feedback_reason = FeedbackReason::None;
    Triple requested_pressure_kpa{};
    Triple command_pressure_kpa{};
    Triple predicted_tip_pcc_mm{};
    Triple position_error_mm{};
    double position_error_norm_mm = 0.0;
    bool measured_error_valid = false;
    bool numeric_valid = true;
    bool pressure_limited = false;
    bool feedback_available = false;
    bool new_feedback_frame = false;
    bool servo_reset = false;
    ContactState contact_state = ContactState::Free;
    Triple feedback_trim_mm{};
    Triple feedforward_pressure_kpa{};
    double feedback_dt_s = 0.0;
};

// Single-thread-owned, independent state per physical actuator. step() has no I/O.
class Controller {
public:
    explicit Controller(const PressureLengthModel& model,
                        ControllerOptions options = {});
    ~Controller();
    Controller(Controller&&) noexcept;
    Controller& operator=(Controller&&) noexcept;
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    void reset(const ResetState& state); // Resets computation only; never writes hardware.
    bool requestMode(Mode mode) noexcept; // False if closed-loop calibration is unavailable.
    Mode requestedMode() const noexcept;
    StepResult step(const StepInput& input) noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

const char* toString(EffectiveMode value) noexcept;
const char* toString(FeedbackReason value) noexcept;
const char* toString(ContactState value) noexcept;

} // namespace soft_actuator
