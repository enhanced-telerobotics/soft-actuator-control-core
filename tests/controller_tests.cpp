#include "soft_actuator_core/controller.hpp"
#include "soft_actuator_core/output_policy.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace soft_actuator;
namespace {
int failures = 0;
void check(bool condition, const char* name) {
    if (!condition) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
}
bool close(const Triple& a, const Triple& b, double eps = 1e-12) {
    for (unsigned i = 0; i < 3; ++i)
        if (!std::isfinite(a[i]) || !std::isfinite(b[i]) || std::abs(a[i]-b[i]) > eps) return false;
    return true;
}
StepInput sample(double now, std::uint64_t seq) {
    StepInput input;
    input.now_s = now;
    input.target_pcc_mm = {2.0, 1.0, 29.0};
    TipFeedback feedback;
    feedback.position_pcc_mm = {0.4, 0.2, 28.6};
    feedback.frame_sequence = seq;
    feedback.received_at_s = now;
    feedback.calibration_revision = 7;
    input.feedback = feedback;
    return input;
}
void arm(Controller& controller) {
    controller.reset({{0.0, 0.0, 0.0}, true, 7});
    check(controller.requestMode(Mode::PositionServo), "enable after valid calibration");
}
void compare(const StepResult& a, const StepResult& b) {
    check(close(a.command_pressure_kpa,b.command_pressure_kpa), "deterministic command pressure");
    check(close(a.requested_pressure_kpa,b.requested_pressure_kpa), "deterministic raw pressure");
    check(a.effective_mode == b.effective_mode, "deterministic mode");
    check(a.contact_state == b.contact_state, "deterministic contact state");
    check(close(a.feedback_trim_mm,b.feedback_trim_mm), "deterministic trim");
}
void testModeAndFeedback(const PressureLengthModel& model) {
    Controller c(model);
    check(c.requestedMode() == Mode::OpenLoop, "default open loop");
    check(!c.requestMode(Mode::PositionServo), "reject missing calibration");
    arm(c);
    auto input = sample(1.0,1);
    auto first = c.step(input);
    check(first.effective_mode == EffectiveMode::PositionServo && first.servo_reset,
          "first fresh sample starts servo immediately");
    check(first.numeric_valid && first.measured_error_valid, "valid measured error");
    check(c.requestMode(Mode::PositionServo), "idempotent mode request");
    input.now_s = 1.005;
    auto duplicate = c.step(input);
    check(!duplicate.new_feedback_frame && !duplicate.servo_reset, "duplicate frame is not integrated/reset");
    check(close(first.feedback_trim_mm, duplicate.feedback_trim_mm), "duplicate frame preserves trim");
    input.now_s = 1.099;
    check(c.step(input).effective_mode == EffectiveMode::PositionServo, "sample younger than 100ms accepted");
    input.now_s = 1.101;
    auto stale = c.step(input);
    check(stale.effective_mode == EffectiveMode::OpenLoopFallback &&
          stale.feedback_reason == FeedbackReason::Stale, "100ms freshness failure immediate fallback");
    input = sample(1.2,2);
    check(c.step(input).effective_mode == EffectiveMode::OpenLoopFallback, "recovery first distinct sample");
    input.now_s = 1.205;
    check(c.step(input).effective_mode == EffectiveMode::OpenLoopFallback, "duplicate does not count recovery");
    input = sample(1.22,3);
    check(c.step(input).effective_mode == EffectiveMode::OpenLoopFallback, "recovery second sample");
    input = sample(1.24,4);
    auto recovered = c.step(input);
    check(recovered.effective_mode == EffectiveMode::PositionServo && recovered.servo_reset,
          "recovery third distinct sample resets servo");
    input = sample(1.26,5);
    input.feedback->calibration_valid = false;
    auto invalid = c.step(input);
    check(invalid.effective_mode == EffectiveMode::OpenLoopFallback &&
          invalid.feedback_reason == FeedbackReason::CalibrationInvalid, "invalid calibration cannot close loop");
    c.requestMode(Mode::OpenLoop);
    check(!c.requestMode(Mode::PositionServo), "invalidated calibration blocks mode entry");
}
void testIndependentAndReplay(const PressureLengthModel& model) {
    Controller left(model), leftReference(model), right(model), rightReference(model);
    arm(left); arm(leftReference);
    for (unsigned i = 0; i < 100; ++i) {
        auto l = sample(1.0+i*.005, 1+i/4);
        l.feedback->received_at_s = 1.0+(i/4)*.02;
        l.target_pcc_mm = {std::sin(i*.02)*2.0, .5, 28.6};
        auto r = l;
        r.target_pcc_mm = {-1.0, std::cos(i*.02), 29.0};
        r.feedback.reset();
        const auto leftValue = left.step(l);
        const auto rightValue = right.step(r);
        compare(leftValue,leftReference.step(l));
        compare(rightValue,rightReference.step(r));
    }
    arm(left); arm(leftReference);
    compare(left.step(sample(10.0,1)),leftReference.step(sample(10.0,1)));
}
void testWritesAndCalibration(const PressureLengthModel& model) {
    Controller withoutWrite(model), staleWrite(model);
    arm(withoutWrite); arm(staleWrite);
    for (unsigned i=0;i<8;++i) {
        auto input=sample(2.0+i*.02,i+1);
        const auto expected=withoutWrite.step(input);
        input.last_written_command=WrittenCommand{{30.0,30.0,30.0},i+1,input.now_s-.151};
        compare(expected,staleWrite.step(input));
    }
    auto input=sample(3.0,20);
    input.feedback->calibration_revision=8;
    check(withoutWrite.step(input).servo_reset,"new calibration revision resets old trim");
    auto old=input;
    old.now_s+=.005;
    check(!withoutWrite.step(old).servo_reset,"unchanged calibration does not repeat reset");
}
void testInvalidAndLimits(const PressureLengthModel& model) {
    Controller c(model);
    auto input=sample(1,1);
    input.target_pcc_mm[0]=std::numeric_limits<double>::quiet_NaN();
    const auto invalid=c.step(input);
    check(!invalid.numeric_valid && close(invalid.command_pressure_kpa,Triple{}),"invalid target returns zero request and explicit failure");
    input=sample(1,1);
    input.pressure_envelope=PressureEnvelope{-1,1};
    for(unsigned i=0;i<50;++i) {
        input.now_s+=.005;
        const auto result=c.step(input);
        for(double p:result.command_pressure_kpa) check(p>=-1 && p<=1,"asymmetric runtime envelope respected");
    }
    bool threw=false;
    try { c.reset({{51,0,0},true,1}); } catch(const std::invalid_argument&) {threw=true;}
    check(threw,"reset rejects out-of-range known pressure");
    threw=false;
    try { Controller bad(model,ControllerOptions{0,.1,.15}); } catch(const std::invalid_argument&) {threw=true;}
    check(threw,"geometry validation");
    c.reset({{},true,7}); c.requestMode(Mode::PositionServo);
    input=sample(2,1); input.feedback.reset();
    check(c.step(input).feedback_reason == FeedbackReason::Missing,"closed loop missing feedback explicit reason");
    c.reset({{},true,7});
    input=sample(2,1); input.feedback->received_at_s=3;
    check(c.step(input).feedback_reason == FeedbackReason::Stale,"future clock timestamp rejected");
}
void testRejectedCallIsNotAStateChange(const PressureLengthModel& model) {
    Controller test(model), reference(model);
    arm(test); arm(reference);
    auto valid=sample(2,1);
    compare(test.step(valid),reference.step(valid));
    auto invalid=valid;
    invalid.now_s=std::numeric_limits<double>::quiet_NaN();
    invalid.feedback->calibration_valid=false;
    const auto rejected=test.step(invalid);
    check(!rejected.numeric_valid && rejected.effective_mode==EffectiveMode::InvalidInput,
          "invalid call metadata returns explicit rejection");
    invalid.now_s=2.005;
    invalid.pressure_abs_limit_kpa=5;
    invalid.pressure_envelope=PressureEnvelope{10,20};
    check(test.step(invalid).effective_mode==EffectiveMode::InvalidInput,
          "conflicting limits rejected without reversed clamp bounds");
    invalid.pressure_abs_limit_kpa=0;
    invalid.pressure_envelope=PressureEnvelope{60,70};
    check(test.step(invalid).effective_mode==EffectiveMode::InvalidInput,
          "envelope outside physical range rejected");
    invalid.pressure_envelope=PressureEnvelope{10,-10};
    check(test.step(invalid).effective_mode==EffectiveMode::InvalidInput,
          "reversed envelope rejected");
    valid=sample(2.02,2);
    const auto next=test.step(valid);
    compare(next,reference.step(valid));
    check(!next.servo_reset,"rejected call must not invalidate calibration or reset trim");
}
void testIndependentWriteClock(const PressureLengthModel& model) {
    Controller c(model); arm(c);
    std::optional<WrittenCommand> written;
    unsigned writes=0;
    for(unsigned i=0;i<40;++i) {
        auto input=sample(1+i*.005,1+i/4);
        input.feedback->received_at_s=1+(i/4)*.02;
        input.last_written_command=written;
        const auto result=c.step(input);
        if(i%2==0) {
            auto out=PressureOutputPolicy::evaluate(result.command_pressure_kpa,
                written ? std::optional<Triple>(written->pressure_kpa) : std::nullopt);
            written=WrittenCommand{out.pressureKPa,++writes,input.now_s};
        }
        check(result.numeric_valid,"separate sensor/control/write clocks stay numerically valid");
    }
    check(writes==20,"writing advances only at caller output cadence");
}
}
int main(int argc,char** argv) {
    if(argc!=2) return 2;
    try {
        const auto model=loadModel(argv[1]);
        testModeAndFeedback(model); testIndependentAndReplay(model);
        testWritesAndCalibration(model); testInvalidAndLimits(model);
        testRejectedCallIsNotAStateChange(model);
        testIndependentWriteClock(model);
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
    std::cout<<"Controller contract failures: "<<failures<<'\n';
    return failures ? 1:0;
}
