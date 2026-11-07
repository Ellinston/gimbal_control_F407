param(
    [string]$GccPath = 'D:/STM32/Clion/CLion 2026.1.3/bin/mingw/bin/gcc.exe'
)
$ErrorActionPreference = 'Stop'
$GccPath = (Resolve-Path -LiteralPath $GccPath).ProviderPath
$env:PATH = (Split-Path -Parent $GccPath) + ';' + $env:PATH
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$outputDir = Join-Path $projectRoot 'build/usart-host-tests'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$testExe = Join-Path $outputDir 'test_usart.exe'
# 调用前将 MinGW bin 加入当前终端 PATH，供 GCC 子进程查找依赖 DLL。
$LASTEXITCODE = 0
& $GccPath -std=c11 -Wall -Wextra -Werror -I $PSScriptRoot -I $outputDir -I (Join-Path $projectRoot 'Bsp/usart') -I (Join-Path $projectRoot 'Bsp/can') -I (Join-Path $projectRoot 'Task') (Join-Path $PSScriptRoot 'test_usart.c') -o $testExe
if ($LASTEXITCODE -ne 0) { throw "Host test build failed (exit $LASTEXITCODE)" }
$LASTEXITCODE = 0
& $testExe
if ($LASTEXITCODE -ne 0) { throw "Host test failed (exit $LASTEXITCODE)" }

