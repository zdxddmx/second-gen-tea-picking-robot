#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Run all seven simulation scenarios against the host firmware build and
    report a pass/fail table.

.DESCRIPTION
    Each scenario injects a different disturbance into the *plant* (never into
    the controller) and asserts a scenario-specific outcome.  For the fault
    scenarios, a completed pick cycle is a *failure*: the whole point is that the
    protection stops the robot.

    This wrapper exists so that `scripts/run_demo.py` and the documentation can
    call one command and get a machine-readable summary, rather than each
    reader re-deriving the pass/fail rules.

.PARAMETER Duration
    Simulated seconds per scenario (default 9).

.PARAMETER Csv
    Also write one CSV trace per scenario into `data/logs/`.

.EXAMPLE
    ./scripts/run_scenarios.ps1
    ./scripts/run_scenarios.ps1 -Duration 12 -Csv
#>
[CmdletBinding()]
param(
    [int]$Duration = 9,
    [switch]$Csv,
    [string]$Exe = 'build/host/tea_picking_robot_host.exe'
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    if (-not (Test-Path $Exe)) {
        throw "$Exe not found. Run scripts/build_host.ps1 first."
    }
    New-Item -ItemType Directory -Force -Path data/logs | Out-Null

    $scenarios = @(
        @{ L='A'; Name='nominal pick'         },
        @{ L='B'; Name='target position error'},
        @{ L='C'; Name='force sensor noise'   },
        @{ L='D'; Name='excessive grip force' },
        @{ L='E'; Name='communication timeout'},
        @{ L='F'; Name='motor stall'          },
        @{ L='G'; Name='emergency stop'       }
    )

    $results = @()
    Write-Host ''
    Write-Host '========== tea-picking robot: scenario sweep ==========' -ForegroundColor Cyan

    # Pull one field out of the simulator's summary block.  Declared once, above
    # the loop: defining a function inside the loop body would rebind it on
    # every iteration, which PowerShell allows but which makes the script harder
    # to reason about.
    function Get-SummaryField {
        param([string]$Text, [string]$Pattern)
        $m = [regex]::Match($Text, $Pattern)
        if ($m.Success) { return $m.Groups[1].Value.Trim() }
        return '?'
    }

    foreach ($s in $scenarios) {
        $runArgs = @('--scenario', $s.L, '--duration', "$Duration", '--quiet')
        if ($Csv) {
            $runArgs += @('--csv', "data/logs/host_scenario_$($s.L).csv")
        }
        $text = (& $Exe @runArgs 2>&1 | Out-String)
        $verdict = if ($text -match 'Simulation: PASS') { 'PASS' } else { 'FAIL' }

        $results += [pscustomobject]@{
            Scenario = $s.L
            What     = $s.Name
            Verdict  = $verdict
            Cycles   = Get-SummaryField $text 'pick cycles\s+:\s+(\d+)'
            Faults   = Get-SummaryField $text 'faults\s+:\s+(\S+)'
            PeakN    = Get-SummaryField $text 'peak grip force\s+:\s+([\d.]+) N'
            Final    = Get-SummaryField $text 'final state\s+:\s+(\S+)'
            Misses   = Get-SummaryField $text 'deadline misses\s+:\s+(\d+)'
        }
    }

    $results | Format-Table -AutoSize | Out-String | Write-Host
    $failed = @($results | Where-Object { $_.Verdict -ne 'PASS' })
    Write-Host ("scenarios: {0}/{1} passed" -f ($results.Count - $failed.Count), $results.Count)
    if ($failed.Count -gt 0) {
        Write-Host ("RESULT: FAIL ({0})" -f (($failed | ForEach-Object { $_.Scenario }) -join ',')) -ForegroundColor Red
        exit 1
    }
    Write-Host 'RESULT: PASS' -ForegroundColor Green
} finally {
    Pop-Location
}
