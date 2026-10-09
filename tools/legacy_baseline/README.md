# Original-controller baseline generation

This optional Windows tool compiles the **unmodified original** `ControlPCC.cpp`,
`frameTrans.cpp`, and `TimeUtil.cpp` from the old `Soft_gripper` checkout. It uses
the original CHAI3D 3.2.0 mathematical headers and the original Viper
`ViperInterface.h` declaration of `PNODATA`. The two small compatibility headers
only avoid device and scene umbrella includes; they implement no mathematics.
No old file is edited. No camera, serial port, ROS node, scene, or GUI is opened.
The resulting fixtures are portable and their replay does not require CHAI3D,
Viper, or the old checkout.

From PowerShell at the package root (the explicit source invocation also works
when the package is on the lab UNC share and unsigned `.ps1` files cannot be
launched there directly):

```powershell
$generator = (Resolve-Path -LiteralPath '.\tools\legacy_baseline').ProviderPath
& ([scriptblock]::Create((Get-Content -LiteralPath (Join-Path $generator 'generate.ps1') -Raw))) `
    -GeneratorDirectory $generator `
    -OutputDirectory (Join-Path $generator '..\..\tests\data')
```

All paths can be supplied explicitly; the exact executed command, source hashes,
compiler version, profile hash, CSV hash, and scenario counts are recorded in
`tests/data/legacy_provenance.json`. The build is separate from the new library's
CMake project. C++14 is used because the legacy bundled Eigen depends on standard
library binder types removed in C++17. `/fp:precise` is explicit.

Each CSV row contains input and original expected output. A `reset=1` row creates
a new original controller and calls `resetPositionServoState(reset_pressure)` before
stepping. Other rows retain all state within that scenario. Open-loop inputs form
trajectories from the previous original returned pressure; replay uses those
recorded inputs to compare each step. Closed-loop pressure state evolves inside
the kernel. `last_pressure_*` is `getDevicePressure()`, which the original
open-loop single-step API does **not** update. `pressure_arg_after_*` also records
the original API's equal-pressure third-channel perturbation.

Sensor coordinates and quaternions are serialized **after conversion to the
original `float` PNODATA storage**. `tip_axis_*` comes from original
`quaternionToMatrix()` column 2, so new quaternion math is not used to generate
expectations. Times are explicit positive deterministic values; the wall-clock
fallback in the original implementation is not exercised.

The fixtures cover open-loop XYZ motion; repeated and advancing feedback frames;
all five contact states through contact, release, and recovery; contact inference
disabled; positive and negative pressure envelopes; an absolute pressure limit;
straight and near-singular configurations; and nonfinite target, sensor, and
pressure inputs. Numerical tolerance is `1e-8 + 1e-7 * max(abs(actual),abs(expected))`;
state flags are exact. Nonfinite values must match classification. Pathological
cases have separate named scenarios and receive no relaxed tolerance.

`reset_pressure_*` is distinct from `pressure_in_*`: the original open-loop API
accepts an independent pressure argument without setting controller pressure.
In the nonfinite-pressure scenario the state starts at zero while the input
argument contains infinity, so these cannot be inferred from one another.

These preserve old numerical behavior, including limitations. For example,
`open_nonfinite_target` currently returns `numericValid=true` from the original
single-step kernel, and `open_near_singular` can produce a NaN modeled position.
The public wrapper must reject invalid inputs/results independently; a matching
legacy fixture is not a claim that every legacy result is safe to apply.
