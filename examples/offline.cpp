#include "soft_actuator_core/controller.hpp"
#include "soft_actuator_core/output_policy.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

// Synthetic input example only: no camera, ROS, board, serial port or wall clock.
int main(int argc, char** argv) {
    using namespace soft_actuator;
    if(argc!=3 || (std::string(argv[2])!="open" && std::string(argv[2])!="closed")) {
        std::cerr<<"Usage: soft_actuator_offline <profile-path> <open|closed>\n";
        return 2;
    }
    try {
        const auto model=loadModel(argv[1]);
        Controller controller(model);
        controller.reset({{},true,1});
        if(std::string(argv[2])=="closed") controller.requestMode(Mode::PositionServo);
        std::optional<WrittenCommand> written;
        TipFeedback measured;
        measured.position_pcc_mm={0,0,model.neutralLengthMm};
        measured.calibration_revision=1;
        std::uint64_t writeSequence=0;
        std::cout<<"time_s,target_x_mm,target_y_mm,target_z_mm,measured_x_mm,measured_y_mm,measured_z_mm,p0_kpa,p1_kpa,p2_kpa,mode,error_mm,numeric_valid,contact,new_frame,write_sequence\n";
        std::cout<<std::setprecision(12);
        for(unsigned i=0;i<400;++i) {
            const double t=1.0+i*.005;
            StepInput input;
            input.now_s=t;
            input.target_pcc_mm={1.5*std::sin(i*.008),.5,model.neutralLengthMm+.3};
            if(i%4==0) {
                // Deliberately imperfect artificial measurement, not a plant model.
                measured.position_pcc_mm={.8*std::sin(i*.008-.15),.2,model.neutralLengthMm};
                measured.frame_sequence=1+i/4;
                measured.received_at_s=t;
            }
            input.feedback=measured;
            input.last_written_command=written;
            const auto result=controller.step(input);
            if(i%2==0) {
                const auto out=PressureOutputPolicy::evaluate(result.command_pressure_kpa,
                    written ? std::optional<Triple>(written->pressure_kpa):std::nullopt);
                // Offline simulated successful write; never touch actual hardware.
                written=WrittenCommand{out.pressureKPa,++writeSequence,t};
            }
            std::cout<<t;
            for(double v:input.target_pcc_mm) std::cout<<','<<v;
            for(double v:measured.position_pcc_mm) std::cout<<','<<v;
            for(double v:result.command_pressure_kpa) std::cout<<','<<v;
            std::cout<<','<<toString(result.effective_mode)<<','<<result.position_error_norm_mm
                <<','<<result.numeric_valid<<','<<toString(result.contact_state)
                <<','<<result.new_feedback_frame<<','<<writeSequence<<'\n';
            if(!result.numeric_valid) return 1;
        }
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n'; return 2;}
}
