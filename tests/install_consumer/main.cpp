#include <soft_actuator_core/controller.hpp>
#include <soft_actuator_core/output_policy.hpp>
#include <cmath>
#include <iostream>
int main(int argc,char** argv) {
    if(argc!=2) return 2;
    try {
        soft_actuator::Controller c(soft_actuator::loadModel(argv[1]));
        soft_actuator::StepInput input;
        input.now_s=1.; input.target_pcc_mm={1.,.5,28.6};
        const auto result=c.step(input);
        const auto out=soft_actuator::PressureOutputPolicy::evaluate(result.command_pressure_kpa,std::nullopt);
        if(!result.numeric_valid || !out.validInput) return 1;
        for(double p:out.pressureKPa) if(!std::isfinite(p)) return 1;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n'; return 2;}
    return 0;
}
