#include "soft_actuator_core/output_policy.hpp"

#include <algorithm>
#include <cmath>

namespace soft_actuator {
namespace {
bool finite(const std::array<double, 3>& values) noexcept {
    return std::isfinite(values[0]) && std::isfinite(values[1]) &&
        std::isfinite(values[2]);
}
bool validHistory(const std::array<double, 3>& values) noexcept {
    if (!finite(values)) return false;
    for (double value : values)
        if (value < PressureOutputPolicy::hardMinKPa ||
            value > PressureOutputPolicy::hardMaxKPa) return false;
    return true;
}
}

OutputDecision PressureOutputPolicy::evaluate(
    const std::array<double, 3>& requestKPa,
    const std::optional<std::array<double, 3>>& lastWrittenKPa,
    const OutputOptions& options) noexcept {
    OutputDecision result;
    if (options.forceZero) {
        result.forcedZero = true;
        return result; // Zero pressure bypasses slew, limits and stale input.
    }

    const bool requestFinite = finite(requestKPa);
    const bool haveLast = lastWrittenKPa && validHistory(*lastWrittenKPa);
    const std::array<double, 3> last = haveLast
        ? *lastWrittenKPa : std::array<double, 3>{};
    result.validInput = requestFinite && (!lastWrittenKPa || haveLast);

    // Preserve whole-frame hard hold, including valid endpoint commands.
    if (!requestFinite) {
        result.pressureKPa = last;
    } else {
        for (unsigned i = 0; i < 3; ++i) {
            if (requestKPa[i] < hardMinKPa) result.hardLowerMask |= 1u << i;
            if (requestKPa[i] > hardMaxKPa) result.hardUpperMask |= 1u << i;
        }
        if ((result.hardLowerMask || result.hardUpperMask) && haveLast) {
            result.pressureKPa = last;
            result.hardHeld = true;
        } else {
            for (unsigned i = 0; i < 3; ++i)
                result.pressureKPa[i] = std::clamp(requestKPa[i], hardMinKPa, hardMaxKPa);
        }
    }

    // These four stages retain their original order: hard guard, slew,
    // profile clamp, then inclusive study guard on the value actually sent.
    if (haveLast) {
        for (unsigned i = 0; i < 3; ++i) {
            const double delta = result.pressureKPa[i] - last[i];
            if (delta > maxDeltaPerWriteKPa)
                result.pressureKPa[i] = last[i] + maxDeltaPerWriteKPa;
            else if (delta < -maxDeltaPerWriteKPa)
                result.pressureKPa[i] = last[i] - maxDeltaPerWriteKPa;
            // Preserve the original requested value when already within slew.
        }
    }
    if (std::isfinite(options.profileAbsLimitKPa) && options.profileAbsLimitKPa > 0.0) {
        for (double& pressure : result.pressureKPa)
            pressure = std::clamp(pressure, -options.profileAbsLimitKPa,
                options.profileAbsLimitKPa);
    }
    if (options.studyGuardEnabled) {
        if (!std::isfinite(options.studyMinKPa) ||
            !std::isfinite(options.studyMaxKPa) ||
            !(options.studyMinKPa < options.studyMaxKPa) ||
            options.studyMinKPa < hardMinKPa ||
            options.studyMaxKPa > hardMaxKPa) {
            result.validInput = false;
            result.pressureKPa = last;
            result.studyHeld = haveLast;
            return result;
        }
        for (unsigned i = 0; i < 3; ++i) {
            if (result.pressureKPa[i] <= options.studyMinKPa)
                result.studyLowerMask |= 1u << i;
            if (result.pressureKPa[i] >= options.studyMaxKPa)
                result.studyUpperMask |= 1u << i;
        }
        if ((result.studyLowerMask || result.studyUpperMask) && haveLast) {
            result.pressureKPa = last;
            result.studyHeld = true;
        } else {
            for (double& pressure : result.pressureKPa)
                pressure = std::clamp(pressure, options.studyMinKPa, options.studyMaxKPa);
        }
    }
    return result;
}

} // namespace soft_actuator
