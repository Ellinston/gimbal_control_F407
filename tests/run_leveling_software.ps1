param(
    [string]$GccPath = 'D:/STM32/Clion/CLion 2026.1.3/bin/mingw/bin/gcc.exe',
    [string]$CMakePath = 'D:/STM32/Clion/CLion 2026.1.3/bin/cmake/win/x64/bin/cmake.exe'
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$reportDir = Join-Path $projectRoot 'build/leveling-software-verification'
New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
# Host programs use mocks only. This runner has no flash/serial/debugger commands.
& "$PSScriptRoot/imu_angle/run.ps1" -GccPath $GccPath -Stage Measurement 2>&1 |
    Tee-Object -FilePath (Join-Path $reportDir '1-measurement.log')
& "$PSScriptRoot/imu_angle/run.ps1" -GccPath $GccPath -Stage Geometry 2>&1 |
    Tee-Object -FilePath (Join-Path $reportDir '2-geometry.log')
& "$PSScriptRoot/motor/run.ps1" -GccPath $GccPath 2>&1 |
    Tee-Object -FilePath (Join-Path $reportDir '3-motor.log')
& "$PSScriptRoot/usart/run.ps1" -GccPath $GccPath 2>&1 |
    Tee-Object -FilePath (Join-Path $reportDir '3-usart.log')
& "$PSScriptRoot/can/run.ps1" -GccPath $GccPath 2>&1 |
    Tee-Object -FilePath (Join-Path $reportDir '3-can.log')
& $CMakePath --build (Join-Path $projectRoot 'build/Debug') 2>&1 |
    Tee-Object -FilePath (Join-Path $reportDir '3-arm.log')
if ($LASTEXITCODE -ne 0) { throw 'ARM compile/link failed' }
$elf = Join-Path $projectRoot 'build/Debug/gimbal_1.elf'
[ordered]@{
    completed_at = (Get-Date -Format o)
    scope = 'Host synthetic/mock tests and ARM compile/link only; no hardware operations'
    stages = @('measurement passed','geometry passed','integration and regressions passed','ARM link passed')
    elf_sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $elf).Hash
    motor_header_sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath (Join-Path $projectRoot 'device/motor/motor.h')).Hash
} | ConvertTo-Json | Set-Content -Encoding UTF8 (Join-Path $reportDir 'result.json')
