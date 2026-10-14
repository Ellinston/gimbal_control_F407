param(
    [string]$GccPath = 'D:/STM32/Clion/CLion 2026.1.3/bin/mingw/bin/gcc.exe'
)
$ErrorActionPreference = 'Stop'
$GccPath = (Resolve-Path -LiteralPath $GccPath).ProviderPath
$env:PATH = (Split-Path -Parent $GccPath) + ';' + $env:PATH
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$outputDir = Join-Path $projectRoot 'build/imu-angle-host-tests'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$testExe = Join-Path $outputDir 'test_imu_angle.exe'
& $GccPath -std=c11 -Wall -Wextra -Werror -I $PSScriptRoot -I (Join-Path $projectRoot 'tests/usart') -I (Join-Path $projectRoot 'device/BMI270') -I (Join-Path $projectRoot 'Bsp/usart') -I (Join-Path $projectRoot 'device/motor') (Join-Path $PSScriptRoot 'test_imu_angle.c') -o $testExe -lm
if ($LASTEXITCODE -ne 0) { throw "IMU angle host build failed (exit $LASTEXITCODE)" }
& $testExe
if ($LASTEXITCODE -ne 0) { throw "IMU angle host tests failed (exit $LASTEXITCODE)" }

