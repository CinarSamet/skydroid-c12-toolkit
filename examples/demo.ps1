# Skydroid C12 - quick demo sequence for a screen/video recording.
#
# Usage:
#   .\examples\demo.ps1 -CameraIp 192.168.144.108
#
# Runs a short sequence of movement + IMU commands with a pause between
# each one, so it's easy to record and easy to follow.

param(
    [Parameter(Mandatory = $true)]
    [string]$CameraIp
)

function Run-Step {
    param([string]$Label, [string[]]$CmdArgs)
    Write-Host ""
    Write-Host "==> $Label" -ForegroundColor Cyan
    python -m skydroid_c12.cli $CameraIp @CmdArgs
    Start-Sleep -Seconds 2
}

Run-Step "Centering"                 @("center")
Run-Step "Pan right 30 degrees"      @("right", "30")
Run-Step "Tilt up 10 degrees"        @("tilt", "10")
Run-Step "Pan + tilt together"       @("point", "-20", "-15")
Run-Step "Watching IMU for 5s"       @("watch", "5")
Run-Step "Back to center"            @("center")

Write-Host ""
Write-Host "Demo finished." -ForegroundColor Green
