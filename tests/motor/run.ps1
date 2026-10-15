param([string]$GccPath = 'D:/STM32/Clion/CLion 2026.1.3/bin/mingw/bin/gcc.exe')
$ErrorActionPreference = 'Stop'
$GccPath = (Resolve-Path -LiteralPath $GccPath).ProviderPath
$env:PATH = (Split-Path -Parent $GccPath) + ';' + $env:PATH
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$outputDir = Join-Path $projectRoot 'build/motor-host-tests'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$testExe = Join-Path $outputDir 'test_motor.exe'
$dualIncludes = @('-I', (Join-Path $projectRoot 'tests/can'), '-I', (Join-Path $projectRoot 'Bsp/can'), '-I', (Join-Path $projectRoot 'Bsp/usart'), '-I', (Join-Path $projectRoot 'Bsp/Algorithm'), '-I', (Join-Path $projectRoot 'device/motor'), '-I', (Join-Path $projectRoot 'Task'))
$dualSources = @((Join-Path $projectRoot 'device/motor/motor.c'), (Join-Path $projectRoot 'Bsp/Algorithm/pid.c'), (Join-Path $projectRoot 'Bsp/Algorithm/velocity_feedforward.c'), (Join-Path $projectRoot 'Task/motor_test.c'))
$dualSources += @((Join-Path $projectRoot 'Bsp/Algorithm/gimbal_level.c'))
$dualSources += @((Join-Path $projectRoot 'Task/usart_test.c'), (Join-Path $PSScriptRoot 'mock_usart_task.c'))
$pidTestExe = Join-Path $outputDir 'test_pid.exe'
& $GccPath -std=c11 -Wall -Wextra -Werror -I (Join-Path $projectRoot 'Bsp/Algorithm') (Join-Path $PSScriptRoot 'test_pid.c') (Join-Path $projectRoot 'Bsp/Algorithm/pid.c') -lm -o $pidTestExe
if ($LASTEXITCODE -ne 0) { throw "PID host build failed (exit $LASTEXITCODE)" }
& $pidTestExe
if ($LASTEXITCODE -ne 0) { throw "PID host checks failed (exit $LASTEXITCODE)" }
& $GccPath -std=c11 -Wall -Wextra -Werror @dualIncludes (Join-Path $PSScriptRoot 'test_motor.c') @dualSources -lm -o $testExe
if ($LASTEXITCODE -ne 0) { throw "Motor host build failed (exit $LASTEXITCODE)" }
& $testExe
if ($LASTEXITCODE -ne 0) { throw "Motor host checks failed (exit $LASTEXITCODE)" }
$singleTestExe = Join-Path $outputDir 'test_single_motor.exe'
& $GccPath -std=c11 -Wall -Wextra -Werror @dualIncludes (Join-Path $PSScriptRoot 'test_single_motor.c') @dualSources -lm -o $singleTestExe
if ($LASTEXITCODE -ne 0) { throw "Single motor host build failed (exit $LASTEXITCODE)" }
& $singleTestExe
if ($LASTEXITCODE -ne 0) { throw "Single motor host checks failed (exit $LASTEXITCODE)" }
$doubleTestExe = Join-Path $outputDir 'test_double_motor.exe'
& $GccPath -std=c11 -Wall -Wextra -Werror @dualIncludes (Join-Path $PSScriptRoot 'test_double_motor.c') @dualSources -lm -o $doubleTestExe
if ($LASTEXITCODE -ne 0) { throw "Double motor host build failed (exit $LASTEXITCODE)" }
& $doubleTestExe
if ($LASTEXITCODE -ne 0) { throw "Double motor host checks failed (exit $LASTEXITCODE)" }

