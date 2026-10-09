#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace soft_actuator {

enum class NegativePressureModel
{
    Quadratic,
    PiecewiseLinear
};

struct PressureLengthModel
{
    double neutralLengthMm = 0.0;
    double positiveSlopeMmPerKPa = 0.0;

    // Legacy negative-pressure model: L = a2*P^2 + a1*P + a0.
    double negativeQuadraticA2 = 0.0;
    double negativeQuadraticA1 = 0.0;
    double negativeQuadraticA0 = 0.0;

    NegativePressureModel negativeModel = NegativePressureModel::Quadratic;

    // Continuous piecewise-linear negative-pressure model.  The near branch
    // is selected for breakpoint <= P < 0; the far branch for P < breakpoint.
    double negativeBreakpointKPa = 0.0;
    double negativeNearSlopeMmPerKPa = 0.0;
    double negativeNearInterceptMm = 0.0;
    double negativeFarSlopeMmPerKPa = 0.0;
    double negativeFarInterceptMm = 0.0;

    // User-study actuation settings travel with the physical actuator model.
    // Defaults preserve compatibility with older profile files that predate
    // these keys; current profiles write all four values explicitly.
    double exp2TargetRadiusMm = 5.6;
    double exp2NeutralPressureKPa = -13.0;
    double studyPressureMinKPa = -40.0;
    double studyPressureMaxKPa = 45.0;
};

namespace pressure_length_model {

using Triple = std::array<double, 3>;

// A deliberately asymmetric pressure interval. Positive and vacuum limits
// are independent because neither the regulator nor the calibrated actuator
// model has a symmetric working range.
struct PressureEnvelope
{
    double minimumKPa = 0.0;
    double maximumKPa = 0.0;
};

struct PressureEnvelopeState
{
    unsigned lowerMask = 0;
    unsigned upperMask = 0;

    bool withinEnvelope() const
    {
        return lowerMask == 0 && upperMask == 0;
    }
};

// Key-8 Z path testing keeps endpoint planning inside the runtime safety
// envelope so closed-loop feedback still has pressure authority at either end.
// With the current -40/+45 kPa profile, endpoint search uses -30 kPa for -Z
// and +38 kPa for +Z, while runtime may still use -38/+43 kPa.
struct ZPathPressureEnvelopes
{
    PressureEnvelope runtime{};
    PressureEnvelope positiveEndpointSearch{};
    PressureEnvelope negativeEndpointSearch{};
};

struct PccTargetPressureEstimate
{
    bool valid = false;
    Triple pccTargetMm{};
    Triple chamberLengthMm{};
    Triple pressureKPa{};
    Triple reconstructedPccMm{};
    double reconstructionResidualMm = 0.0;
    PressureEnvelope envelope{};
    PressureEnvelopeState envelopeState{};

    bool withinEnvelope() const
    {
        return valid && envelopeState.withinEnvelope();
    }
};

inline bool isValid(const PressureEnvelope& envelope)
{
    return std::isfinite(envelope.minimumKPa) &&
        std::isfinite(envelope.maximumKPa) &&
        envelope.minimumKPa < envelope.maximumKPa;
}

inline PressureEnvelope studyPressureEnvelope(
    const PressureLengthModel& model)
{
    return PressureEnvelope{
        model.studyPressureMinKPa,
        model.studyPressureMaxKPa
    };
}

inline bool makeZPathPressureEnvelopes(
    const PressureLengthModel& model,
    double standardReserveKPa,
    double positiveEndpointSearchUpperReserveKPa,
    double negativeEndpointSearchLowerReserveKPa,
    ZPathPressureEnvelopes& envelopes)
{
    if (!std::isfinite(model.studyPressureMinKPa) ||
        !std::isfinite(model.studyPressureMaxKPa) ||
        !std::isfinite(standardReserveKPa) ||
        !std::isfinite(positiveEndpointSearchUpperReserveKPa) ||
        !std::isfinite(negativeEndpointSearchLowerReserveKPa) ||
        standardReserveKPa < 0.0 ||
        positiveEndpointSearchUpperReserveKPa < standardReserveKPa ||
        negativeEndpointSearchLowerReserveKPa < standardReserveKPa) {
        return false;
    }

    envelopes.runtime = PressureEnvelope{
        model.studyPressureMinKPa + standardReserveKPa,
        model.studyPressureMaxKPa - standardReserveKPa
    };
    envelopes.positiveEndpointSearch = PressureEnvelope{
        model.studyPressureMinKPa + standardReserveKPa,
        model.studyPressureMaxKPa - positiveEndpointSearchUpperReserveKPa
    };
    // The margin is added to a negative lower limit so it moves toward zero.
    // For example, -40 kPa plus the current 10 kPa reserve is -30 kPa.
    envelopes.negativeEndpointSearch = PressureEnvelope{
        model.studyPressureMinKPa + negativeEndpointSearchLowerReserveKPa,
        model.studyPressureMaxKPa - standardReserveKPa
    };
    return isValid(envelopes.runtime) &&
        isValid(envelopes.positiveEndpointSearch) &&
        isValid(envelopes.negativeEndpointSearch);
}

inline PressureEnvelopeState classifyPressureEnvelope(
    const Triple& pressureKPa,
    const PressureEnvelope& envelope)
{
    PressureEnvelopeState state{};
    if (!isValid(envelope)) {
        state.lowerMask = 0x7u;
        state.upperMask = 0x7u;
        return state;
    }

    for (std::size_t chamber = 0; chamber < pressureKPa.size(); ++chamber) {
        const unsigned bit = 1u << static_cast<unsigned>(chamber);
        if (!std::isfinite(pressureKPa[chamber])) {
            state.lowerMask |= bit;
            state.upperMask |= bit;
        }
        else if (pressureKPa[chamber] < envelope.minimumKPa) {
            state.lowerMask |= bit;
        }
        else if (pressureKPa[chamber] > envelope.maximumKPa) {
            state.upperMask |= bit;
        }
    }
    return state;
}

inline const char* negativeModelName(NegativePressureModel model)
{
    return model == NegativePressureModel::PiecewiseLinear
        ? "piecewise_linear"
        : "quadratic";
}

inline double negativeLengthForPressureKPa(
    const PressureLengthModel& model,
    double pressureKPa)
{
    if (model.negativeModel == NegativePressureModel::PiecewiseLinear) {
        return pressureKPa >= model.negativeBreakpointKPa
            ? model.negativeNearSlopeMmPerKPa * pressureKPa +
                model.negativeNearInterceptMm
            : model.negativeFarSlopeMmPerKPa * pressureKPa +
                model.negativeFarInterceptMm;
    }
    return model.negativeQuadraticA2 * pressureKPa * pressureKPa +
        model.negativeQuadraticA1 * pressureKPa +
        model.negativeQuadraticA0;
}

inline double lengthForPressureKPa(
    const PressureLengthModel& model,
    double pressureKPa)
{
    return pressureKPa < 0.0
        ? negativeLengthForPressureKPa(model, pressureKPa)
        : model.positiveSlopeMmPerKPa * pressureKPa + model.neutralLengthMm;
}

inline bool pressureForLengthMm(
    const PressureLengthModel& model,
    double lengthMm,
    double& pressureKPa)
{
    if (!std::isfinite(lengthMm) ||
        !(model.neutralLengthMm > 0.0) ||
        !(model.positiveSlopeMmPerKPa > 0.0)) {
        return false;
    }

    if (lengthMm >= model.neutralLengthMm) {
        pressureKPa =
            (lengthMm - model.neutralLengthMm) /
            model.positiveSlopeMmPerKPa;
        return std::isfinite(pressureKPa);
    }

    if (model.negativeModel == NegativePressureModel::PiecewiseLinear) {
        if (!(model.negativeBreakpointKPa < 0.0) ||
            !(model.negativeNearSlopeMmPerKPa > 0.0) ||
            !(model.negativeFarSlopeMmPerKPa > 0.0)) {
            return false;
        }
        const double breakpointLengthMm =
            model.negativeNearSlopeMmPerKPa * model.negativeBreakpointKPa +
            model.negativeNearInterceptMm;
        pressureKPa = lengthMm >= breakpointLengthMm
            ? (lengthMm - model.negativeNearInterceptMm) /
                model.negativeNearSlopeMmPerKPa
            : (lengthMm - model.negativeFarInterceptMm) /
                model.negativeFarSlopeMmPerKPa;
        return std::isfinite(pressureKPa);
    }

    const double a2 = model.negativeQuadraticA2;
    const double a1 = model.negativeQuadraticA1;
    const double c = model.negativeQuadraticA0 - lengthMm;
    if (std::fabs(a2) < 1e-12) {
        return false;
    }
    double discriminant = a1 * a1 - 4.0 * a2 * c;
    if (!std::isfinite(discriminant)) {
        return false;
    }
    discriminant = discriminant < 0.0 ? 0.0 : discriminant;
    const double sqrtDiscriminant = std::sqrt(discriminant);
    const double denominator = 2.0 * a2;
    const double first = (-a1 + sqrtDiscriminant) / denominator;
    const double second = (-a1 - sqrtDiscriminant) / denominator;
    pressureKPa = first < 0.0 ? first : second;
    return std::isfinite(pressureKPa);
}

inline bool reconstructPccFromChamberLengthsMm(
    const Triple& chamberLengthMm,
    double chamberRadiusMm,
    Triple& reconstructedPccMm)
{
    const double q1 = chamberLengthMm[0];
    const double q2 = chamberLengthMm[1];
    const double q3 = chamberLengthMm[2];
    if (!std::isfinite(q1) || !std::isfinite(q2) || !std::isfinite(q3) ||
        !(q1 > 0.0) || !(q2 > 0.0) || !(q3 > 0.0) ||
        !std::isfinite(chamberRadiusMm) || !(chamberRadiusMm > 0.0)) {
        return false;
    }

    const double qSum = q1 + q2 + q3;
    double rootSquared =
        q1 * q1 + q2 * q2 + q3 * q3 - q1 * q2 - q2 * q3 - q1 * q3;
    if (!std::isfinite(rootSquared) || !(qSum > 0.0)) {
        return false;
    }
    if (rootSquared < 0.0 && rootSquared > -1e-10) {
        rootSquared = 0.0;
    }
    if (rootSquared < 0.0) {
        return false;
    }

    const double root = std::sqrt(rootSquared);
    const double centerLengthMm = qSum / 3.0;
    constexpr double kStraightEpsilon = 1e-10;
    if (root < kStraightEpsilon) {
        reconstructedPccMm = Triple{ 0.0, 0.0, centerLengthMm };
        return true;
    }

    const double theta = 2.0 * root / (3.0 * chamberRadiusMm);
    const double kappa = 2.0 * root / (chamberRadiusMm * qSum);
    const double phi = std::atan2(
        std::sqrt(3.0) * (q2 + q3 - 2.0 * q1),
        3.0 * (q2 - q3));
    if (!std::isfinite(theta) || !std::isfinite(kappa) ||
        !std::isfinite(phi) || std::fabs(kappa) < kStraightEpsilon) {
        return false;
    }

    reconstructedPccMm = Triple{
        std::cos(phi) * (1.0 - std::cos(theta)) / kappa,
        std::sin(phi) * (1.0 - std::cos(theta)) / kappa,
        std::sin(theta) / kappa
    };
    return std::isfinite(reconstructedPccMm[0]) &&
        std::isfinite(reconstructedPccMm[1]) &&
        std::isfinite(reconstructedPccMm[2]);
}

// Pure position-only inverse for the same constant-curvature geometry used by
// ControlPCC::computeCurrArcFromPccPose(). The target is expressed in the PCC
// frame in millimeters. For a non-vertical target, its XYZ position uniquely
// determines bend plane, bend angle, and the three chamber lengths; no marker
// orientation is required. The returned pressures use the supplied actuator
// profile's production length-to-pressure inverse and are never clamped.
inline PccTargetPressureEstimate estimatePressureForPccTargetMm(
    const PressureLengthModel& model,
    const Triple& pccTargetMm,
    double chamberRadiusMm,
    const PressureEnvelope& envelope)
{
    PccTargetPressureEstimate estimate{};
    estimate.pccTargetMm = pccTargetMm;
    estimate.envelope = envelope;

    const double xMm = pccTargetMm[0];
    const double yMm = pccTargetMm[1];
    const double zMm = pccTargetMm[2];
    if (!std::isfinite(xMm) || !std::isfinite(yMm) ||
        !std::isfinite(zMm) || !(zMm > 0.0) ||
        !std::isfinite(chamberRadiusMm) || !(chamberRadiusMm > 0.0) ||
        !isValid(envelope)) {
        return estimate;
    }

    constexpr double kNearVerticalSquaredMm = 1e-6;
    constexpr double kGeometryEpsilon = 1e-9;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kChamberAngleDeg[3] = { 90.0, 210.0, 330.0 };

    const double lateralSquaredMm = xMm * xMm + yMm * yMm;
    if (lateralSquaredMm < kNearVerticalSquaredMm) {
        estimate.chamberLengthMm = Triple{ zMm, zMm, zMm };
    }
    else {
        const double lateralMm = std::sqrt(lateralSquaredMm);
        const double theta = 2.0 * std::atan2(lateralMm, zMm);
        const double sinTheta = std::sin(theta);
        const double oneMinusCosTheta = 1.0 - std::cos(theta);
        double bendRadiusMm = 0.0;
        if (std::fabs(sinTheta) > kGeometryEpsilon) {
            bendRadiusMm = zMm / sinTheta;
        }
        else if (std::fabs(oneMinusCosTheta) > kGeometryEpsilon) {
            bendRadiusMm = lateralMm / oneMinusCosTheta;
        }
        if (!std::isfinite(theta) || !std::isfinite(bendRadiusMm) ||
            std::fabs(bendRadiusMm) < kGeometryEpsilon) {
            return estimate;
        }

        const double centerLengthMm = theta * bendRadiusMm;
        const double phi = std::atan2(yMm, xMm);
        for (std::size_t chamber = 0;
            chamber < estimate.chamberLengthMm.size();
            ++chamber) {
            const double chamberAngleRad =
                kChamberAngleDeg[chamber] * kPi / 180.0;
            estimate.chamberLengthMm[chamber] = centerLengthMm -
                theta * chamberRadiusMm *
                    std::cos(chamberAngleRad - phi);
        }
    }

    for (std::size_t chamber = 0;
        chamber < estimate.chamberLengthMm.size();
        ++chamber) {
        const double lengthMm = estimate.chamberLengthMm[chamber];
        if (!std::isfinite(lengthMm) || !(lengthMm > 0.0) ||
            !pressureForLengthMm(
                model,
                lengthMm,
                estimate.pressureKPa[chamber])) {
            return estimate;
        }
    }

    estimate.envelopeState = classifyPressureEnvelope(
        estimate.pressureKPa,
        envelope);
    if (!reconstructPccFromChamberLengthsMm(
        estimate.chamberLengthMm,
        chamberRadiusMm,
        estimate.reconstructedPccMm)) {
        return estimate;
    }
    const double errorX = estimate.reconstructedPccMm[0] - pccTargetMm[0];
    const double errorY = estimate.reconstructedPccMm[1] - pccTargetMm[1];
    const double errorZ = estimate.reconstructedPccMm[2] - pccTargetMm[2];
    estimate.reconstructionResidualMm = std::sqrt(
        errorX * errorX + errorY * errorY + errorZ * errorZ);
    if (!std::isfinite(estimate.reconstructionResidualMm)) {
        return estimate;
    }
    estimate.valid = true;
    return estimate;
}

inline bool isSane(const PressureLengthModel& model)
{
    if (!(model.neutralLengthMm > 0.0) ||
        !(model.positiveSlopeMmPerKPa > 0.0) ||
        !(model.exp2TargetRadiusMm > 0.0) ||
        !std::isfinite(model.exp2NeutralPressureKPa) ||
        !std::isfinite(model.studyPressureMinKPa) ||
        !std::isfinite(model.studyPressureMaxKPa) ||
        !(model.studyPressureMinKPa < model.studyPressureMaxKPa) ||
        !(model.exp2NeutralPressureKPa > model.studyPressureMinKPa) ||
        !(model.exp2NeutralPressureKPa < model.studyPressureMaxKPa)) {
        return false;
    }
    if (model.negativeModel == NegativePressureModel::Quadratic) {
        return std::fabs(model.negativeQuadraticA2) >= 1e-12 &&
            std::isfinite(model.negativeQuadraticA1) &&
            std::isfinite(model.negativeQuadraticA0);
    }
    if (!(model.negativeBreakpointKPa < 0.0) ||
        !(model.negativeNearSlopeMmPerKPa > 0.0) ||
        !(model.negativeFarSlopeMmPerKPa > 0.0) ||
        std::fabs(model.negativeNearInterceptMm - model.neutralLengthMm) > 1e-6) {
        return false;
    }
    const double nearAtBreakpoint =
        model.negativeNearSlopeMmPerKPa * model.negativeBreakpointKPa +
        model.negativeNearInterceptMm;
    const double farAtBreakpoint =
        model.negativeFarSlopeMmPerKPa * model.negativeBreakpointKPa +
        model.negativeFarInterceptMm;
    return std::isfinite(nearAtBreakpoint) &&
        std::isfinite(farAtBreakpoint) &&
        std::fabs(nearAtBreakpoint - farAtBreakpoint) <= 1e-6;
}

} // namespace pressure_length_model

// Explicit configuration-time loading; no current-directory search or fallback.
PressureLengthModel loadModel(const std::string& path);
// Configuration-time validation. Optional error text describes a rejected model.
bool validateModel(const PressureLengthModel& model, std::string* error = nullptr);

} // namespace soft_actuator
