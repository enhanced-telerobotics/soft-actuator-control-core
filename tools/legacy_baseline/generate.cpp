// Expected outputs come exclusively from the original ControlPCC implementation.
// This file constructs synthetic input trajectories and serializes old results.
#include "ControlPCC.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

using chai3d::cVector3d;

struct Input {
    cVector3d target{0., 0., 28.6};
    cVector3d pressure{0., 0., 0.};
    PositionServoFeedbackSample feedback{};
    pressure_length_model::PressureEnvelope envelope{-40., 50.};
    bool useEnvelope = false;
    bool contact = true;
    double absLimit = 0.;
    Input() {
        feedback.sensorPNO.pos[2] = 28.6f;
        feedback.sensorPNO.ori[0] = 1.f;
        feedback.frameSeq = 1;
        feedback.receiveTimeSec = 10.;
    }
};

class Writer {
    std::ofstream out;
    std::unique_ptr<ControlPCC> controller;
    std::string scenario;
    cVector3d resetPressure{0.,0.,0.};
    int step = 0;
    bool reset = true;

    void value(double x) {
        out << ',';
        if (std::isnan(x)) out << "nan";
        else if (std::isinf(x)) out << (x < 0 ? "-inf" : "inf");
        else out << x;
    }
    void vector(const cVector3d& v) { value(v.x()); value(v.y()); value(v.z()); }

public:
    explicit Writer(const char* path): out(path, std::ios::binary) {
        if (!out) throw std::runtime_error("Cannot create fixture output");
        out << std::setprecision(17);
        out << "scenario,step,reset,mode"
            ",target_x,target_y,target_z,pressure_in_0,pressure_in_1,pressure_in_2,reset_pressure_0,reset_pressure_1,reset_pressure_2"
            ",sensor_x,sensor_y,sensor_z,sensor_qw,sensor_qx,sensor_qy,sensor_qz,tip_axis_x,tip_axis_y,tip_axis_z"
            ",frame_seq,receive_sec,applied_valid,applied_0,applied_1,applied_2"
            ",envelope_enabled,envelope_min,envelope_max,abs_limit,contact_enabled"
            ",pressure_0,pressure_1,pressure_2,model_x,model_y,model_z"
            ",unconstrained_0,unconstrained_1,unconstrained_2,numeric_valid,pressure_limited"
            ",last_pressure_0,last_pressure_1,last_pressure_2"
            ",pressure_arg_after_0,pressure_arg_after_1,pressure_arg_after_2"
            ",free_trim_0,free_trim_1,free_trim_2,contact_trim_0,contact_trim_1,contact_trim_2"
            ",trim_0,trim_1,trim_2,contact_state,error_x,error_y,error_z,error_norm"
            ",position_residual_x,position_residual_y,position_residual_z,position_residual_norm"
            ",length_residual_0,length_residual_1,length_residual_2,length_residual_norm"
            ",feedforward_pressure_0,feedforward_pressure_1,feedforward_pressure_2"
            ",feedforward_length_0,feedforward_length_1,feedforward_length_2"
            ",last_feedback_seq,feedback_dt,load_likely,freeze_active\n";
    }

    void begin(const std::string& name, const cVector3d& initial = cVector3d(0.,0.,0.)) {
        scenario = name;
        step = 0;
        reset = true;
        resetPressure = initial;
        controller = std::make_unique<ControlPCC>();
        controller->resetPositionServoState(initial);
    }

    cVector3d row(const Input& input, bool closed) {
        auto& c = *controller;
        cVector3d pressureArg = input.pressure;
        cVector3d targetArg = input.target;
        const auto result = closed
            ? c.updateMotionPositionServoRealtime(input.target, input.feedback,
                input.absLimit, input.useEnvelope ? &input.envelope : nullptr, input.contact)
            : c.updateMotionRealtime(pressureArg, targetArg);
        out << scenario << ',' << step++ << ',' << int(reset) << ',' << (closed ? "closed" : "open");
        reset = false;
        vector(input.target); vector(input.pressure); vector(resetPressure);
        for (float x: input.feedback.sensorPNO.pos) value(x);
        for (float x: input.feedback.sensorPNO.ori) value(x);
        chai3d::cQuaternion q;
        q.w = input.feedback.sensorPNO.ori[0]; q.x = input.feedback.sensorPNO.ori[1];
        q.y = input.feedback.sensorPNO.ori[2]; q.z = input.feedback.sensorPNO.ori[3];
        const auto rotation = quaternionToMatrix(q); // ORIGINAL frameTrans.cpp
        vector(cVector3d(rotation(0,2), rotation(1,2), rotation(2,2)));
        out << ',' << input.feedback.frameSeq;
        value(input.feedback.receiveTimeSec);
        out << ',' << int(input.feedback.appliedPressureValid);
        vector(input.feedback.appliedPressureKPa);
        out << ',' << int(input.useEnvelope);
        value(input.envelope.minimumKPa); value(input.envelope.maximumKPa);
        value(input.absLimit); out << ',' << int(input.contact);
        vector(result.pressure); vector(result.position); vector(result.unconstrainedPressure);
        out << ',' << int(result.numericValid) << ',' << int(result.pressureLimited);
        vector(c.getDevicePressure()); vector(pressureArg);
        vector(c.m_positionServoFreeTrimLengthMm);
        vector(c.m_positionServoContactTrimLengthMm);
        vector(c.m_positionServoFeedbackTrimLengthMm);
        out << ',' << int(c.m_positionServoContactState);
        vector(c.m_positionServoActualErrorPccMm); value(c.m_positionServoActualErrorNormMm);
        vector(c.m_positionServoPositionResidualPccMm); value(c.m_positionServoPositionResidualNormMm);
        vector(c.m_positionServoLengthResidualMm); value(c.m_positionServoLengthResidualNormMm);
        vector(c.m_positionServoFeedforwardPressureKPa); vector(c.m_positionServoFeedforwardLengthMm);
        out << ',' << c.m_positionServoLastFeedbackFrameSeq;
        value(c.m_positionServoFeedbackDtSec);
        out << ',' << int(c.m_positionServoLoadLikely) << ',' << int(c.m_positionServoFreezeActive) << '\n';
        if (!out) throw std::runtime_error("Failed writing fixture row");
        return result.pressure;
    }
};

int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "Usage: legacy_baseline OUTPUT.csv\n"; return 2; }
    try {
        Writer writer(argv[1]);
        Input input;
        writer.begin("open_straight");
        for (int i=0; i<12; ++i) input.pressure = writer.row(input, false);

        input = Input{};
        writer.begin("open_xyz_trajectory");
        for (int i=0; i<90; ++i) {
            input.target = cVector3d(3.*std::sin(i*.07), 2.*std::cos(i*.09), 28.6+std::sin(i*.05));
            input.pressure = writer.row(input, false);
        }

        input = Input{};
        input.pressure = cVector3d(1., 1.+1e-10, 1.-1e-10);
        writer.begin("open_near_singular", input.pressure);
        input.target = cVector3d(.001, -.001, 28.75);
        for (int i=0; i<6; ++i) input.pressure = writer.row(input, false);

        input = Input{};
        writer.begin("closed_new_and_repeated_frames");
        for (int i=0; i<75; ++i) {
            input.target = cVector3d(1.+.02*(i/3), -.4, 29.1);
            input.feedback.frameSeq = 1+i/3;
            input.feedback.receiveTimeSec = 10.+.02*(i/3);
            input.feedback.appliedPressureValid = (i/3)%2 == 0;
            input.feedback.appliedPressureKPa = input.pressure;
            input.pressure = writer.row(input, true);
        }

        for (bool enabled: {true,false}) {
            input = Input{};
            input.contact = enabled;
            input.feedback.appliedPressureValid = true;
            writer.begin(enabled ? "closed_contact_release" : "closed_contact_disabled");
            for (int i=0; i<36; ++i) {
                input.feedback.frameSeq = 1+i;
                input.feedback.receiveTimeSec = 20.+.02*i;
                input.feedback.sensorPNO.pos[2] = i<12 ? 19.f : 28.6f;
                input.pressure = writer.row(input, true);
            }
        }

        for (bool positive: {true,false}) {
            input = Input{};
            input.contact = false;
            input.useEnvelope = true;
            input.envelope = {-1., .5};
            input.target = cVector3d(0.,0.,positive ? 60. : 10.);
            writer.begin(positive ? "closed_envelope_positive" : "closed_envelope_negative");
            for (int i=0; i<140; ++i) {
                input.feedback.frameSeq = 1+i;
                input.feedback.receiveTimeSec = 30.+.02*i;
                input.pressure = writer.row(input, true);
            }
        }

        input = Input{};
        input.contact = false;
        input.absLimit = .2;
        input.target = cVector3d(0.,0.,35.);
        writer.begin("closed_absolute_pressure_limit");
        for (int i=0; i<32; ++i) {
            input.feedback.frameSeq = 1+i;
            input.feedback.receiveTimeSec = 40.+.02*i;
            input.pressure = writer.row(input, true);
        }

        input = Input{};
        writer.begin("closed_near_straight_pose");
        input.feedback.sensorPNO.pos[0] = 1e-8f;
        input.feedback.sensorPNO.ori[2] = 1e-10f;
        input.target = cVector3d(.1,0.,28.6);
        writer.row(input,true);

        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();
        for (bool closed: {true,false}) {
            input = Input{};
            writer.begin(closed ? "closed_nonfinite_target" : "open_nonfinite_target");
            input.target.x(nan);
            writer.row(input,closed);
        }
        input = Input{};
        writer.begin("closed_nonfinite_sensor");
        input.feedback.sensorPNO.pos[0] = static_cast<float>(nan);
        writer.row(input,true);
        input = Input{};
        input.pressure = cVector3d(inf, 0., 0.);
        writer.begin("open_nonfinite_pressure");
        writer.row(input,false);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
