#pragma once

#include <array>
#include <optional>

namespace soft_actuator {

struct OutputOptions {
    bool forceZero = false;
    double profileAbsLimitKPa = 0.0; // Non-positive/non-finite disables this clamp.
    bool studyGuardEnabled = false;
    // Enabled intervals must be finite, ordered and inside [-40, 50] kPa.
    // Invalid intervals return the last valid write or zero, validInput=false.
    double studyMinKPa = -40.0;
    double studyMaxKPa = 45.0;
};

struct OutputDecision {
    std::array<double, 3> pressureKPa{};
    unsigned hardLowerMask = 0;
    unsigned hardUpperMask = 0;
    unsigned studyLowerMask = 0;
    unsigned studyUpperMask = 0;
    bool hardHeld = false;
    bool studyHeld = false;
    bool validInput = true;
    bool forcedZero = false;
};

// Transport-independent counterpart of the legacy writer policy. Call once per
// intended write, using the last successfully written pressure, not a merely
// requested value. evaluate() never changes caller state or performs I/O.
// Non-finite/out-of-hard-range history is treated as absent and reported through
// validInput=false. Explicit forceZero bypasses input validation and outputs zero.
class PressureOutputPolicy {
public:
    static constexpr double hardMinKPa = -40.0;
    static constexpr double hardMaxKPa = 50.0;
    static constexpr double maxDeltaPerWriteKPa = 20.0;

    static OutputDecision evaluate(
        const std::array<double, 3>& requestKPa,
        const std::optional<std::array<double, 3>>& lastWrittenKPa,
        const OutputOptions& options = {}) noexcept;
};

} // namespace soft_actuator
