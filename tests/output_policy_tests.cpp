#include "soft_actuator_core/model.hpp"
#include "soft_actuator_core/output_policy.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace soft_actuator;
using Triple = std::array<double, 3>;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void equal(const Triple& actual, const Triple& expected, const char* message) {
    require(actual == expected, message);
}

struct TemporaryProfile {
    std::filesystem::path path;
    explicit TemporaryProfile(const std::string& text) {
        static unsigned sequence = 0;
        path = std::filesystem::temp_directory_path() /
            ("soft_actuator_model_test_" + std::to_string(
                std::chrono::high_resolution_clock::now().time_since_epoch().count()) +
                "_" + std::to_string(++sequence) + ".txt");
        std::ofstream output(path);
        output << text;
        if (!output) throw std::runtime_error("Cannot create temporary model fixture");
    }
    ~TemporaryProfile() {
        std::error_code error;
        std::filesystem::remove(path, error);
    }
};

void rejectsProfile(const std::string& text) {
    const TemporaryProfile profile(text);
    bool rejected = false;
    try { (void)loadModel(profile.path.string()); }
    catch (const std::runtime_error&) { rejected = true; }
    require(rejected, "Malformed profile was accepted");
}

void outputPolicyTests() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    equal(PressureOutputPolicy::evaluate({30, -30, 10}, Triple{0, 0, 0}).pressureKPa,
        {20, -20, 10}, "Slew must limit each write to 20 kPa");
    equal(PressureOutputPolicy::evaluate({-40, 50, 0}, std::nullopt).pressureKPa,
        {-40, 50, 0}, "Hard endpoints must be valid first writes");
    auto decision = PressureOutputPolicy::evaluate({51, 2, 3}, Triple{1, 2, 3});
    equal(decision.pressureKPa, {1, 2, 3}, "One hard violation must hold all channels");
    require(decision.hardHeld && decision.hardUpperMask == 1 && !decision.hardLowerMask,
        "Hard hold diagnostics must identify the violating chamber");
    decision = PressureOutputPolicy::evaluate({90, -90, 0}, std::nullopt);
    equal(decision.pressureKPa, {50, -40, 0}, "First command clamps without prior write");
    require(!decision.hardHeld && decision.hardUpperMask == 1 && decision.hardLowerMask == 2,
        "First clamp diagnostics are incorrect");
    decision = PressureOutputPolicy::evaluate({nan, 9, 8}, Triple{2, 3, 4});
    equal(decision.pressureKPa, {2, 3, 4}, "Non-finite request must use last whole frame");
    require(!decision.validInput, "Non-finite request must be reported");
    equal(PressureOutputPolicy::evaluate({nan, 9, 8}, std::nullopt).pressureKPa,
        {0, 0, 0}, "No history plus invalid request must return zero pressure");
    decision = PressureOutputPolicy::evaluate({1, 2, 3}, Triple{nan, 0, 0});
    equal(decision.pressureKPa, {1, 2, 3}, "Invalid prior pressure must not propagate NaN");
    require(!decision.validInput, "Invalid prior pressure must be reported");
    decision = PressureOutputPolicy::evaluate({1, 2, 3}, Triple{51, 0, 0});
    equal(decision.pressureKPa, {1, 2, 3}, "Overpressure history must be treated as absent");
    require(!decision.validInput, "Out-of-hard-range history must be reported");
    decision = PressureOutputPolicy::evaluate({1, 2, 3}, Triple{-41, 0, 0});
    equal(decision.pressureKPa, {1, 2, 3}, "Excess vacuum history must be treated as absent");
    require(!decision.validInput, "Under-range history must be reported");

    OutputOptions options;
    options.profileAbsLimitKPa = 5;
    equal(PressureOutputPolicy::evaluate({30, 30, 30}, Triple{30, 30, 30}, options).pressureKPa,
        {5, 5, 5}, "Profile clamp must follow slew and immediately enforce a new limit");
    options = {};
    options.studyGuardEnabled = true;
    decision = PressureOutputPolicy::evaluate({50, 50, 50}, Triple{0, 0, 0}, options);
    equal(decision.pressureKPa, {20, 20, 20}, "Study guard must inspect post-slew pressure");
    require(!decision.studyHeld && !decision.studyUpperMask,
        "Distant request must not trigger study hold before slew");
    decision = PressureOutputPolicy::evaluate({45, 2, 3}, Triple{30, 4, 5}, options);
    equal(decision.pressureKPa, {30, 4, 5}, "Study endpoint must hold the entire previous write");
    require(decision.studyHeld && decision.studyUpperMask == 1,
        "Study endpoint equality must trigger its inclusive limit");
    decision = PressureOutputPolicy::evaluate({-40, 0, 0}, Triple{-30, 4, 5}, options);
    require(decision.studyHeld && decision.studyLowerMask == 1,
        "Study lower endpoint equality must trigger its inclusive limit");
    options.studyMinKPa = 1;
    options.studyMaxKPa = 1;
    decision = PressureOutputPolicy::evaluate({2, 2, 2}, Triple{3, 4, 5}, options);
    equal(decision.pressureKPa, {3, 4, 5}, "Invalid study interval must hold prior output");
    require(!decision.validInput && decision.studyHeld, "Invalid interval must be diagnosed");
    options.studyMinKPa = 60;
    options.studyMaxKPa = 70;
    decision = PressureOutputPolicy::evaluate({0, 0, 0}, std::nullopt, options);
    equal(decision.pressureKPa, {0, 0, 0}, "Invalid study interval must never raise zero above hard maximum");
    require(!decision.validInput && !decision.studyHeld, "Out-of-range study interval must be diagnosed");
    options.studyMinKPa = -50;
    options.studyMaxKPa = -45;
    decision = PressureOutputPolicy::evaluate({0, 0, 0}, Triple{3, 4, 5}, options);
    equal(decision.pressureKPa, {3, 4, 5}, "Invalid study interval must return last valid write");
    require(!decision.validInput && decision.studyHeld, "Under-range study interval must be diagnosed");
    options.forceZero = true;
    decision = PressureOutputPolicy::evaluate({nan, 50, 50}, Triple{50, 50, 50}, options);
    equal(decision.pressureKPa, {0, 0, 0}, "Explicit zero must bypass slew and guard holds");
    require(decision.forcedZero, "Forced-zero status is missing");
}

void modelTests(const std::string& profilePath) {
    const auto model = loadModel(profilePath);
    require(validateModel(model), "Bundled profile is invalid");
    require(model.neutralLengthMm == 28.6 &&
        model.negativeModel == NegativePressureModel::PiecewiseLinear,
        "Bundled profile identity changed");
    for (double pressure : {-40.0, -19.0, -18.999, -4.0, 0.0, 25.0, 50.0}) {
        const double length = pressure_length_model::lengthForPressureKPa(model, pressure);
        double reconstructed = 0;
        require(pressure_length_model::pressureForLengthMm(model, length, reconstructed),
            "Pressure-length inversion failed");
        require(std::fabs(reconstructed - pressure) < 1e-8,
            "Pressure-length round trip changed");
    }
    auto invalid = model;
    invalid.neutralLengthMm = std::numeric_limits<double>::infinity();
    std::string error;
    require(!validateModel(invalid, &error) && !error.empty(), "Infinite neutral length accepted");
    invalid = model;
    invalid.negativeFarInterceptMm += 1;
    require(!validateModel(invalid), "Discontinuous piecewise model accepted");
    invalid = model;
    invalid.studyPressureMaxKPa = 51;
    require(!validateModel(invalid), "Out-of-regulator study range accepted");

    std::ifstream file(profilePath);
    const std::string validText((std::istreambuf_iterator<char>(file)), {});
    rejectsProfile(validText + "\nneutralLengthMm=29\n");
    rejectsProfile(validText + "\nunrecognized=1\n");
    rejectsProfile("neutralLengthMm=28.6\npositiveSlopeMmPerKPa=0.15\n");
    rejectsProfile("neutralLengthMm=28.6junk\n");
    rejectsProfile("neutralLengthMm=nan\n");
    rejectsProfile(validText + "\nnot-a-key-value-line\n");
    bool rejected = false;
    try { (void)loadModel(""); }
    catch (const std::runtime_error&) { rejected = true; }
    require(rejected, "Empty explicit path must never load a fallback");

    const TemporaryProfile quadratic(
        "neutralLengthMm=28.6\npositiveSlopeMmPerKPa=0.15\n"
        "negativeModel=quadratic\nnegativeQuadraticA2=0.001\n"
        "negativeQuadraticA1=0.2\nnegativeQuadraticA0=28.6\n");
    require(loadModel(quadratic.path.string()).negativeModel == NegativePressureModel::Quadratic,
        "Legacy quadratic profile loading failed");
}
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "Usage: output_policy_tests <actuatorprofile_4.txt>");
        outputPolicyTests();
        modelTests(argv[1]);
        std::cout << "Output policy and explicit model loading checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
