param(
    [string]$LegacyRoot = 'C:\Users\ERIE_Lab\Documents\jxs1778\soft_haptics\Soft_gripper',
    [string]$ChaiRoot = 'C:\Users\ERIE_Lab\Documents\jxs1778\chai3d-3.2.0',
    [string]$ViperInclude = 'C:\Users\ERIE_Lab\Documents\jxs1778\ViperPNOGrabberMI\Inc',
    [string]$BuildDirectory = (Join-Path $env:TEMP ('soft-actuator-legacy-baseline-' + [guid]::NewGuid().ToString('N'))),
    [string]$OutputDirectory = (Join-Path $PSScriptRoot '..\..\tests\data'),
    [string]$GeneratorDirectory = $PSScriptRoot,
    [string]$CMake = 'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
)
$ErrorActionPreference = 'Stop'
$LegacyRoot = (Resolve-Path -LiteralPath $LegacyRoot).ProviderPath
$ChaiRoot = (Resolve-Path -LiteralPath $ChaiRoot).ProviderPath
$ViperInclude = (Resolve-Path -LiteralPath $ViperInclude).ProviderPath
$source = Get-Content -LiteralPath (Join-Path $LegacyRoot 'ControlPCC.cpp') -Raw
if ($source -notmatch 'kActivePressureLengthProfileFilename\s*=\s*"actuatorprofile_4.txt"') {
    throw 'This fixture generator requires the original active profile to be actuatorprofile_4.txt.'
}
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$OutputDirectory = (Resolve-Path -LiteralPath $OutputDirectory).ProviderPath
$csv = Join-Path $OutputDirectory 'legacy_profile4.csv'
$profile = Join-Path $OutputDirectory 'legacy_actuatorprofile_4.txt'

& $CMake -S $GeneratorDirectory -B $BuildDirectory -G 'Visual Studio 17 2022' -A x64 "-DLEGACY_ROOT=$LegacyRoot" "-DCHAI_ROOT=$ChaiRoot" "-DVIPER_INCLUDE=$ViperInclude"
if ($LASTEXITCODE -ne 0) { throw 'Legacy baseline configure failed.' }
& $CMake --build $BuildDirectory --config Release
if ($LASTEXITCODE -ne 0) { throw 'Legacy baseline build failed.' }
$executable = Join-Path $BuildDirectory 'Release\legacy_baseline.exe'
Push-Location -LiteralPath $LegacyRoot
try {
    & $executable $csv
    if ($LASTEXITCODE -ne 0) { throw 'Original implementation baseline generation failed.' }
}
finally { Pop-Location }
# This is test input, not a backup or a modification of the original project.
[IO.File]::WriteAllBytes($profile, [IO.File]::ReadAllBytes((Join-Path $LegacyRoot 'actuatorprofile_4.txt')))

$originalPaths = @('ControlPCC.cpp','ControlPCC.hpp','frameTrans.cpp','frameTrans.hpp',
    'TimeUtil.cpp','TimeUtil.hpp','PressureLengthModel.hpp','SerialPressurePolicy.hpp',
    'actuatorprofile_4.txt') | ForEach-Object { Join-Path $LegacyRoot $_ }
$originalPaths += Join-Path $ViperInclude 'ViperInterface.h'
# Record original mathematical and support headers, including the bundled
# Eigen files used by CHAI3D. None are copied into the new portable library.
$originalPaths += Get-ChildItem -LiteralPath (Join-Path $ChaiRoot 'src\math'),(Join-Path $ChaiRoot 'src\system'),(Join-Path $ChaiRoot 'external\Eigen\Eigen') -Recurse -File | ForEach-Object FullName
$hashes = @($originalPaths | Sort-Object -Unique | ForEach-Object {
    $hash = Get-FileHash -LiteralPath $_ -Algorithm SHA256
    [ordered]@{path=$hash.Path; sha256=$hash.Hash.ToLowerInvariant()}
})
$rows = @(Import-Csv -LiteralPath $csv)
$command = "& ([scriptblock]::Create((Get-Content -LiteralPath '$GeneratorDirectory\generate.ps1' -Raw))) -GeneratorDirectory '$GeneratorDirectory' -LegacyRoot '$LegacyRoot' -ChaiRoot '$ChaiRoot' -ViperInclude '$ViperInclude' -BuildDirectory '$BuildDirectory' -OutputDirectory '$OutputDirectory' -CMake '$CMake'"
$manifest = [ordered]@{
    schema_version = 1
    generated_utc = [DateTime]::UtcNow.ToString('o')
    generator_command = $command
    toolchain = 'Visual Studio 2022 x64, C++14, Release, /fp:precise; version recorded in CMake cache below'
    compiler_configuration = @(Get-ChildItem -Path (Join-Path $BuildDirectory 'CMakeFiles\*\CMakeCXXCompiler.cmake') | ForEach-Object {
        Get-Content -LiteralPath $_.FullName | Where-Object { $_ -match '^set\(CMAKE_CXX_COMPILER(_ID|_VERSION)? ' }
    })
    original_translation_units = @('ControlPCC.cpp','frameTrans.cpp','TimeUtil.cpp')
    include_adapters = @('compat/chai3d.h includes original math headers; no math replacement', 'compat/VPcmdIF.h includes original ViperInterface.h and PNODATA')
    source_hashes = $hashes
    csv_sha256 = (Get-FileHash -LiteralPath $csv -Algorithm SHA256).Hash.ToLowerInvariant()
    profile_sha256 = (Get-FileHash -LiteralPath $profile -Algorithm SHA256).Hash.ToLowerInvariant()
    row_count = $rows.Count
    scenarios = @($rows | Group-Object scenario | ForEach-Object { [ordered]@{name=$_.Name;rows=$_.Count} })
    comparison = [ordered]@{absolute_tolerance=1e-8;relative_tolerance=1e-7;state_flags='exact';nonfinite='match NaN classification or infinity sign, never loosen near-singular tolerance'}
    units = [ordered]@{position_and_length='mm';pressure='kPa';time='s';quaternion='wxyz in original PNODATA float storage';tip_axis='original frameTrans.cpp rotation column 2'}
    hardware = 'No camera, ROS node, serial connection, device SDK call, scene or GUI is constructed.'
}
$manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $OutputDirectory 'legacy_provenance.json') -Encoding utf8
Write-Output "Generated $($rows.Count) legacy rows in $OutputDirectory"
