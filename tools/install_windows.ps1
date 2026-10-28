# SPDX-License-Identifier: Apache-2.0
# Install the xplane-bench service on the X-Plane workstation.
#
#   Run ONCE in an *elevated* PowerShell (the firewall rule needs admin):
#     powershell -ExecutionPolicy Bypass -File tools\install_windows.ps1
#
# What it does, and how to undo each:
#   1. Firewall: allow inbound TCP 8770 to Python, from the QNAP only.
#        undo: Remove-NetFirewallRule -DisplayName "xplane-bench service"
#   2. Startup: a launcher in your Startup folder starts the service at logon
#      (in your session -- X-Plane needs your desktop and GPU).
#        undo: delete "xplane-bench.cmd" from shell:startup
#   3. Starts the service now.
#   Hands-off at any time: create the file HANDS_OFF in the repo root; nothing
#   starts while it exists. Delete it to resume.
param(
    [string]$Repo = (Resolve-Path "$PSScriptRoot\.."),
    [string[]]$AllowFrom = @("10.110.10.6"),
    [int]$Port = 8770
)
$ErrorActionPreference = "Stop"
$py = (Get-Command python).Source
# The Microsoft Store Python runs from an alias; the real binary is what listens.
$pyReal = (& $py -c "import sys; print(sys.executable)").Trim()

if (-not (Test-Path "$Repo\config.local.yaml")) { throw "config.local.yaml missing -- copy config.example.yaml and fill in tokens" }

$admin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator)
if ($admin) {
    Get-NetFirewallRule -DisplayName "xplane-bench service" -ErrorAction SilentlyContinue | Remove-NetFirewallRule
    New-NetFirewallRule -DisplayName "xplane-bench service" -Direction Inbound -Action Allow -Protocol TCP `
        -LocalPort $Port -RemoteAddress $AllowFrom -Program $pyReal -Profile Any | Out-Null
    Write-Host "firewall: TCP $Port open to $($AllowFrom -join ', ') for $pyReal"
} else {
    Write-Warning "not elevated: skipped the firewall rule. Re-run as administrator to open port $Port to the QNAP."
}

$startup = [Environment]::GetFolderPath("Startup")
$cmd = "@echo off`r`nstart `"xplane-bench`" /min cmd /c `"$Repo\tools\run_service.cmd`"`r`n"
Set-Content -Path "$startup\xplane-bench.cmd" -Value $cmd -Encoding ascii
Write-Host "startup: $startup\xplane-bench.cmd"

$running = Get-CimInstance Win32_Process | Where-Object { $_.Name -like "python*" -and $_.CommandLine -like "*xpbench.service*" }
if (-not $running) {
    Start-Process -FilePath "$startup\xplane-bench.cmd" -WindowStyle Minimized
    Write-Host "service started"
} else { Write-Host "service already running (pid $($running.ProcessId -join ','))" }
Start-Sleep 3
try { Invoke-RestMethod "http://127.0.0.1:$Port/health" | ConvertTo-Json -Depth 4 } catch { Write-Warning "health check failed: $_" }
