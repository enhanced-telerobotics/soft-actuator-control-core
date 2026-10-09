# Soft Actuator Core

Standalone C++17 control library for a three-chamber soft actuator. It takes a target position in the actuator's PCC frame and optional measured tip feedback, then returns three pressure setpoints. Create one `Controller` per actuator. The library has no MT, ROS 2, Sensoray, serial-port or robot dependencies.

## Build and run

Requires a C++17 compiler and CMake 3.16 or later. There are no third-party runtime dependencies.

Windows PowerShell, from this directory:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
.\build\Release\soft_actuator_offline.exe .\profiles\actuatorprofile_4.txt open > open.csv
.\build\Release\soft_actuator_offline.exe .\profiles\actuatorprofile_4.txt closed > closed.csv
cmake --install build --config Release --prefix install
```

Linux:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/soft_actuator_offline profiles/actuatorprofile_4.txt closed > closed.csv
cmake --install build --prefix install
```

The example uses synthetic feedback and simulated successful writes. It demonstrates independent control, feedback and output rates at 200 Hz, 50 Hz and 100 Hz respectively. These rates are illustrative, not hardware recommendations; the example is neither a pneumatic simulation nor a hardware performance test.

Link the installed library from another CMake project:

```cmake
find_package(soft_actuator_core CONFIG REQUIRED)
target_link_libraries(your_program PRIVATE soft_actuator::core)
```

Configure the consuming project with `-DCMAKE_PREFIX_PATH=<library-install-directory>`.

## Minimal usage

```cpp
#include <soft_actuator_core/controller.hpp>
#include <soft_actuator_core/output_policy.hpp>

using namespace soft_actuator;
auto model = loadModel("/explicit/path/actuatorprofile_4.txt");
Controller controller(model);
controller.reset({{0.0, 0.0, 0.0}, true, 1}); // Caller confirms valid tracking calibration.
controller.requestMode(Mode::PositionServo);

StepInput in;
in.now_s = 1.0;
in.target_pcc_mm = {1.0, 0.0, model.neutralLengthMm};
TipFeedback feedback;
feedback.position_pcc_mm = {0.2, 0.0, model.neutralLengthMm};
feedback.frame_sequence = 1;
feedback.received_at_s = 1.0;
feedback.calibration_revision = 1;
in.feedback = feedback;
auto result = controller.step(in);

// Evaluate separately at output time. Update lastWritten only after a successful write.
std::optional<Triple> lastWritten;
auto output = PressureOutputPolicy::evaluate(result.command_pressure_kpa, lastWritten);
// The adapter decides whether to write and calls the driver. This example performs no I/O.
```

Check `result.numeric_valid`, the effective mode and `output.validInput` before writing to hardware. `reset()` changes software state only; it neither writes to the board nor establishes physical zero pressure. Open loop is the default and requires no tip feedback or tracking calibration. A valid pressure-to-length model is still required.

## Input and output conventions

| Field | Convention |
|---|---|
| `target_pcc_mm` | Target XYZ relative to the soft actuator's own base, in mm. |
| `feedback.position_pcc_mm` | Measured tip position in the same PCC frame, in mm. |
| `feedback.tip_axis_pcc` | Tip-axis direction in the same frame. The adapter converts quaternions and marker extrinsics. |
| Time | `now_s`, feedback receive time and write time use the same local monotonic clock, in seconds. Feedback receive time must be positive. |
| `frame_sequence` | Nonzero source measurement frame ID. Preserve the ID and receive time when reusing a sample. |
| `calibration_*` | The adapter validates the zero/reference-frame calibration. A revision change or restored validity resets feedback state. |
| `last_written_command` | Last command successfully handed to the driver, not measured chamber pressure or confirmation of device execution. Its sequence is for record keeping. |
| Pressure | kPa, in fixed chamber order 0/1/2. The channels do not represent independent XYZ directions. |

Switching to `Mode::PositionServo` requires valid tracking calibration; check the return value of `requestMode()`. Establish calibration validity through `reset()` or by passing calibrated feedback during an open-loop step. Repeating the current mode request does not reset the controller. `reset()` preserves the requested mode while clearing recovery counters and correction history.

`StepResult` distinguishes the pressure request before core limits, the command after core limits, the predicted tip position and the position error. Only `measured_error_valid=true` identifies an error from processed tip feedback. Repeated frames retain the error from the last processed measurement. Open-loop error comes from the model.

## Preserved legacy behavior

- Open loop uses the fixed algorithm step **0.04**, not the actual call interval. Closed-loop pressure changes are limited to **0.25 kPa per channel per call**, or **0.20 kPa** in contact-family states. Changing the call rate changes physical behavior.
- Missing position feedback or feedback older than **100 ms** immediately falls back to open-loop computation. Three consecutive valid new frames restore closed loop after a failure. Initial closed-loop entry can proceed immediately with valid feedback.
- A successful-write record older than **150 ms** is excluded from the model residual; the core pressure state is used instead. Write records are not automatically aligned to camera exposure times.
- Feedback correction updates only on new measurement frames. The legacy low-pass filter remains disabled. Contact states and parameters are preserved. Disabling contact inference suppresses new contact evidence; it does not immediately clear an existing contact state.
- An invalid target or numerical result sets `command_pressure_kpa` to zero and reports `numeric_valid = false`. The pre-limit `requested_pressure_kpa` remains a diagnostic value and may be non-finite. This does not force an immediate hardware write of zero. Invalid call time, tolerance or envelope, including an envelope with no intersection with the hard/profile limits, returns `InvalidInput` and preserves the prior state and command. A non-positive or non-finite absolute-pressure limit retains the legacy convention of disabling that optional limit.
- Internal matrix operations preserve the old inverse-failure behavior, equal-pressure perturbation and near-singular paths. No pseudoinverse or parameter retuning was introduced. Some pathological inputs can still produce invalid model values; callers must check validity flags.

Each instance has a single owning thread. Create a new instance to change the model. Reset or update the valid calibration revision after changing the reference calibration or zero. `step()` performs no heap allocation, file access, system-clock access or logging.

## Output protection is separate from the control loop

Call `PressureOutputPolicy::evaluate()` when preparing an actual output, using the last successful write. It applies these stages in order:

1. Hard pressure-range check: **-40 to +50 kPa**.
2. Maximum change of **20 kPa per channel per write**.
3. Optional profile pressure clamp.
4. Optional study guard, with default thresholds of **-40 and +45 kPa**.

With valid output history, a range violation can hold the entire pressure triple. Preserve this behavior rather than replacing it with per-channel clipping; later protection stages still apply. `forceZero=true` returns three zeros and bypasses these limits. The adapter handles watchdogs, board retries, command expiry, pressure-to-voltage conversion and write confirmation. Serial framing and two-decimal transport quantization are outside this library.

The public API additionally rejects invalid output history and study configurations outside the hard range, reporting `validInput=false`. Valid legacy configurations retain their original processing order. Check this flag before writing.

## Regression validation

CTest covers public interfaces, state transitions, independent controller instances, model functions, output protection, open/closed-loop examples, installed-library consumption and numerical replay against the original `ControlPCC.cpp`. Replay inputs include explicit reset pressures, separate from the current step's pressure values.

Portable reference fixtures are included; normal builds do not require the old project. The numerical tolerance is `1e-8 + 1e-7 * max(|actual|, |expected|)`. States and flags must match exactly, including NaN/Inf classification. Source hashes and reproduction commands are in `tests/data/legacy_provenance.json`. Optional reference-generation instructions are in `tools/legacy_baseline/README.md`.

The bundled profile 4 is a calibration example from the previous physical actuator. Calibrate each new actuator before hardware use. A future ROS 2 adapter will handle subscriptions, coordinate/unit conversion, start/stop coordination and Sensoray status exchange. This package contains no ROS nodes.
