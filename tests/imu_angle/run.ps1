param(
    [string]$GccPath = 'D:/STM32/Clion/CLion 2026.1.3/bin/mingw/bin/gcc.exe',
    [ValidateSet('All','Measurement','Geometry')][string]$Stage = 'All'
)
$ErrorActionPreference = 'Stop'
$GccPath = (Resolve-Path -LiteralPath $GccPath).ProviderPath
$env:PATH = (Split-Path -Parent $GccPath) + ';' + $env:PATH
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$outputDir = Join-Path $projectRoot 'build/imu-angle-host-tests'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$testExe = Join-Path $outputDir 'test_imu_angle.exe'
if ($Stage -ne 'Geometry') {
& $GccPath -std=c11 -Wall -Wextra -Werror -I (Join-Path $projectRoot 'Bsp/Algorithm') -I $PSScriptRoot -I (Join-Path $projectRoot 'tests/usart') -I (Join-Path $projectRoot 'device/BMI270') -I (Join-Path $projectRoot 'Bsp/usart') -I (Join-Path $projectRoot 'device/motor') (Join-Path $PSScriptRoot 'test_imu_angle.c') (Join-Path $projectRoot 'Bsp/Algorithm/gravity_estimator.c') -o $testExe -lm
if ($LASTEXITCODE -ne 0) { throw "IMU angle host build failed (exit $LASTEXITCODE)" }
& $testExe
if ($LASTEXITCODE -ne 0) { throw "IMU angle host tests failed (exit $LASTEXITCODE)" }
}
$geometryExe = Join-Path $outputDir 'test_gravity_geometry.exe'
& $GccPath -std=c11 -Wall -Wextra -Werror -I (Join-Path $projectRoot 'tests/can') -I (Join-Path $projectRoot 'Bsp/Algorithm') -I (Join-Path $projectRoot 'Task') -I (Join-Path $projectRoot 'device/motor') (Join-Path $PSScriptRoot 'test_gravity_geometry.c') (Join-Path $projectRoot 'Bsp/Algorithm/gravity_estimator.c') (Join-Path $projectRoot 'Bsp/Algorithm/gimbal_level.c') -o $geometryExe -lm
if ($LASTEXITCODE -ne 0) { throw 'Gravity/geometry host build failed' }
& $geometryExe $Stage.ToLowerInvariant()
if ($LASTEXITCODE -ne 0) { throw 'Gravity/geometry host validation failed' }

