param([string]$GccPath = 'D:/STM32/Clion/CLion 2026.1.3/bin/mingw/bin/gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$GccPath = (Resolve-Path -LiteralPath $GccPath).Path
$env:PATH = (Split-Path $GccPath) + ';' + $env:PATH
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force build/motor-host-tests | Out-Null
    foreach ($fixedCurrent in @(0,600,-600,20000)) {
        $testExe = "build/motor-host-tests/test_m3508_$fixedCurrent.exe"
        & $GccPath -std=c11 -Wall -Wextra -Werror "-DFIXED_CURRENT=$fixedCurrent" -I tests/can -I Bsp/can -I Task tests/motor/test_m3508.c Task/m3508_task.c -lm -o $testExe
        if ($LASTEXITCODE -ne 0) { throw "M3508 host build failed: FIXED_CURRENT=$fixedCurrent" }
        & (Join-Path $projectRoot $testExe)
        if ($LASTEXITCODE -ne 0) { throw "M3508 checks failed: FIXED_CURRENT=$fixedCurrent" }
    }
} finally { Pop-Location }
