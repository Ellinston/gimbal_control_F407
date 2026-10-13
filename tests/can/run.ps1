param([string]$GccPath = 'D:/STM32/Clion/CLion 2026.1.3/bin/mingw/bin/gcc.exe')
$ErrorActionPreference = 'Stop'
$GccPath = (Resolve-Path -LiteralPath $GccPath).ProviderPath
$env:PATH = (Split-Path -Parent $GccPath) + ';' + $env:PATH
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$outputDir = Join-Path $projectRoot 'build/can-host-tests'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$testExe = Join-Path $outputDir 'test_can.exe'
& $GccPath -std=c11 -Wall -Wextra -Werror -I $PSScriptRoot -I (Join-Path $projectRoot 'Bsp/can') -I (Join-Path $projectRoot 'Bsp/usart') (Join-Path $PSScriptRoot 'test_can.c') -o $testExe
if ($LASTEXITCODE -ne 0) { throw "CAN host build failed (exit $LASTEXITCODE)" }
& $testExe
if ($LASTEXITCODE -ne 0) { throw "CAN host checks failed (exit $LASTEXITCODE)" }

