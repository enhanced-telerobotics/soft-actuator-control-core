#include "soft_actuator_core/model.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace soft_actuator {
namespace {
std::string trim(const std::string& value) {
    const auto first = std::find_if_not(value.begin(), value.end(),
        [](unsigned char c) { return std::isspace(c); });
    const auto last = std::find_if_not(value.rbegin(), value.rend(),
        [](unsigned char c) { return std::isspace(c); }).base();
    return first < last ? std::string(first, last) : std::string{};
}
double number(const std::string& key, const std::string& text) {
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    double value{};
    if (!(stream >> value) || !std::isfinite(value))
        throw std::runtime_error("Invalid finite numeric value for " + key);
    stream >> std::ws;
    if (!stream.eof()) throw std::runtime_error("Trailing text in " + key);
    return value;
}
}

bool validateModel(const PressureLengthModel& model, std::string* error) {
    if (error) error->clear();
    const auto reject = [error](const char* text) {
        if (error) *error = text;
        return false;
    };
    const double values[] = {
        model.neutralLengthMm, model.positiveSlopeMmPerKPa,
        model.negativeQuadraticA2, model.negativeQuadraticA1, model.negativeQuadraticA0,
        model.negativeBreakpointKPa, model.negativeNearSlopeMmPerKPa,
        model.negativeNearInterceptMm, model.negativeFarSlopeMmPerKPa,
        model.negativeFarInterceptMm, model.exp2TargetRadiusMm,
        model.exp2NeutralPressureKPa, model.studyPressureMinKPa, model.studyPressureMaxKPa
    };
    for (double value : values)
        if (!std::isfinite(value)) return reject("Model contains non-finite coefficients");
    if (model.negativeModel != NegativePressureModel::Quadratic &&
        model.negativeModel != NegativePressureModel::PiecewiseLinear)
        return reject("Unsupported negative pressure model");
    if (!pressure_length_model::isSane(model))
        return reject("Model failed pressure-length/continuity/study-envelope checks");
    if (model.studyPressureMinKPa < -40.0 || model.studyPressureMaxKPa > 50.0)
        return reject("Study envelope exceeds the legacy regulator range [-40, 50] kPa");
    return true;
}

PressureLengthModel loadModel(const std::string& path) {
    if (path.empty()) throw std::runtime_error("A model file path is required");
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open model: " + path);
    PressureLengthModel model;
    using Member = double PressureLengthModel::*;
    const std::map<std::string, Member> fields = {
        {"neutralLengthMm", &PressureLengthModel::neutralLengthMm},
        {"positiveSlopeMmPerKPa", &PressureLengthModel::positiveSlopeMmPerKPa},
        {"negativeQuadraticA2", &PressureLengthModel::negativeQuadraticA2},
        {"negativeQuadraticA1", &PressureLengthModel::negativeQuadraticA1},
        {"negativeQuadraticA0", &PressureLengthModel::negativeQuadraticA0},
        {"negativeBreakpointKPa", &PressureLengthModel::negativeBreakpointKPa},
        {"negativeNearSlopeMmPerKPa", &PressureLengthModel::negativeNearSlopeMmPerKPa},
        {"negativeNearInterceptMm", &PressureLengthModel::negativeNearInterceptMm},
        {"negativeFarSlopeMmPerKPa", &PressureLengthModel::negativeFarSlopeMmPerKPa},
        {"negativeFarInterceptMm", &PressureLengthModel::negativeFarInterceptMm},
        {"exp2TargetRadiusMm", &PressureLengthModel::exp2TargetRadiusMm},
        {"exp2NeutralPressureKPa", &PressureLengthModel::exp2NeutralPressureKPa},
        {"studyPressureMinKPa", &PressureLengthModel::studyPressureMinKPa},
        {"studyPressureMaxKPa", &PressureLengthModel::studyPressureMaxKPa}
    };
    const std::set<std::string> metadata = {
        "profile_schema_version", "profile_id", "positivePressureMinKPa",
        "positivePressureMaxKPa", "negativePressureMinKPa", "negativePressureMaxKPa"
    };
    std::set<std::string> seen;
    std::string line;
    unsigned lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        line = trim(line.substr(0, line.find('#')));
        if (line.empty()) continue;
        const auto separator = line.find('=');
        if (separator == std::string::npos)
            throw std::runtime_error("Expected key=value at " + path + ":" + std::to_string(lineNumber));
        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        if (key.empty() || value.empty() || !seen.insert(key).second)
            throw std::runtime_error("Empty or duplicate model field: " + key);
        if (key == "description") continue;
        if (key == "negativeModel") {
            if (value == "quadratic") model.negativeModel = NegativePressureModel::Quadratic;
            else if (value == "piecewise_linear") model.negativeModel = NegativePressureModel::PiecewiseLinear;
            else throw std::runtime_error("Unsupported negativeModel: " + value);
        } else if (const auto field = fields.find(key); field != fields.end()) {
            model.*(field->second) = number(key, value);
        } else if (metadata.count(key)) {
            const double parsed = number(key, value);
            if (key == "profile_schema_version" && parsed != 1.0 && parsed != 2.0)
                throw std::runtime_error("Unsupported profile schema version");
        } else {
            throw std::runtime_error("Unknown model field: " + key);
        }
    }
    if (input.bad()) throw std::runtime_error("Failed reading model: " + path);
    const auto required = [&seen](const char* key) {
        if (!seen.count(key)) throw std::runtime_error(std::string("Missing model field: ") + key);
    };
    required("neutralLengthMm");
    required("positiveSlopeMmPerKPa");
    if (model.negativeModel == NegativePressureModel::PiecewiseLinear) {
        for (const char* key : {"negativeBreakpointKPa", "negativeNearSlopeMmPerKPa",
            "negativeNearInterceptMm", "negativeFarSlopeMmPerKPa", "negativeFarInterceptMm"}) required(key);
    } else {
        for (const char* key : {"negativeQuadraticA2", "negativeQuadraticA1", "negativeQuadraticA0"}) required(key);
    }
    const char* studyKeys[] = {"exp2TargetRadiusMm", "exp2NeutralPressureKPa",
        "studyPressureMinKPa", "studyPressureMaxKPa"};
    unsigned studyCount = 0;
    for (const char* key : studyKeys) studyCount += seen.count(key) ? 1 : 0;
    if (studyCount != 0 && studyCount != 4)
        throw std::runtime_error("Study model settings must be all present or all omitted");
    std::string error;
    if (!validateModel(model, &error)) throw std::runtime_error("Invalid model " + path + ": " + error);
    return model;
}

} // namespace soft_actuator
