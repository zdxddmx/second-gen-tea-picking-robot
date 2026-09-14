#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Build and run the tea-picking robot firmware in host (PC simulation) mode.

.DESCRIPTION
    Compiles every hardware-independent firmware source for the host target and
    links it with the deterministic physics backend.  This is the build used by
    the C unit tests and by `scripts/run_demo.py`.

    It exists alongside the root Makefile because the firmware was actually
    developed on a Windows machine that has a working `gcc` but no `make` on
    PATH; the Makefile is the portable entry point and this script is the one
    that was exercised during development.

.PARAMETER Target
    `host` builds the simulation binary, `test` builds and runs the C unit
    tests, `both` does both.

.PARAMETER Run
    After building `host`, run the simulator with `-RunArgs`.

.PARAMETER RunArgs
    Arguments passed straight through to the simulator.

.EXAMPLE
    ./scripts/build_host.ps1
    ./scripts/build_host.ps1 -Target both
    ./scripts/build_host.ps1 -Run -RunArgs '--scenario','A','--duration','8'
#>
[CmdletBinding()]
param(
    [ValidateSet('host', 'test', 'both')]
    [string]$Target = 'host',
    [switch]$Run,
    # Declared as an explicit array rather than via ValueFromRemainingArguments:
    # PowerShell would otherwise try to bind the simulator's own `--flag value`
    # pairs to this script's parameters and reject them as ambiguous.
    [string[]]$RunArgs = @()
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root

<#
.SYNOPSIS
    Run a native command and surface its complete output on failure.

.DESCRIPTION
    With `$ErrorActionPreference = 'Stop'`, PowerShell turns a native command's
    first stderr line into a terminating error and *discards the rest*.  For a
    compiler that is the worst possible behaviour: the line you actually need is
    usually the fourth.  This helper captures everything and prints it verbatim
    before failing.
#>
function Invoke-Native {
    param(
        [Parameter(Mandatory = $true)][string]$Exe,
        # Taken as one explicit array rather than via ValueFromRemainingArguments,
        # because PowerShell would try to bind the compiler's own `-o` and `-lm`
        # flags to this function's parameters.
        [string[]]$Arguments = @()
    )
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $output = & $Exe @Arguments 2>&1
        $code = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $previous
    }
    if ($code -ne 0) {
        $output | ForEach-Object { Write-Host $_ }
        throw "$Exe failed with exit code $code"
    }
    if ($output) { $output | ForEach-Object { Write-Host $_ } }
}

try {
    $incDirs = @(
        'firmware/Core', 'firmware/Application',
        'firmware/Middleware/utils', 'firmware/Middleware/osal',
        'firmware/Debug', 'firmware/BSP', 'firmware/BSP/host',
        'firmware/Control', 'firmware/Safety', 'firmware/Protocol'
    )
    $includes = $incDirs | ForEach-Object { "-I$_" }
    $defines = @('-DTP_PLATFORM_HOST=1')

    # The firmware common set: everything except the target-specific backends.
    # `firmware/BSP/stm32f407/` is excluded by construction (it is never
    # globbed), and the three target-only translation units in `firmware/Core/`
    # are excluded by name because they include the STM32 HAL and the FreeRTOS
    # headers, neither of which exists in a host build.
    $targetOnly = @('main_stm32.c', 'stm32f4xx_it.c', 'freertos_hooks.c')
    $common = @()
    $common += Get-ChildItem firmware/Core/*.c |
        Where-Object { $targetOnly -notcontains $_.Name }
    $common += Get-ChildItem firmware/Application/*.c
    $common += Get-ChildItem firmware/Middleware/utils/*.c
    $common += Get-Item firmware/Middleware/osal/osal_baremetal.c
    $common += Get-ChildItem firmware/Debug/*.c
    $common += Get-ChildItem firmware/BSP/*.c
    $common += Get-ChildItem firmware/BSP/host/*.c
    $common += Get-ChildItem firmware/Control/*.c
    $common += Get-ChildItem firmware/Protocol/*.c
    $common += Get-ChildItem firmware/Safety/*.c

    New-Item -ItemType Directory -Force -Path build/host | Out-Null
    New-Item -ItemType Directory -Force -Path data/logs, data/results | Out-Null

    $flags = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-O2', '-g') +
             $defines + $includes

    if ($Target -eq 'host' -or $Target -eq 'both') {
        Write-Host '== building host simulation binary ==' -ForegroundColor Cyan
        # `$common` contains main_host.c (the simulator's main) and does NOT
        # contain the test tree, so exactly one `main()` is linked here.
        $src = ($common | ForEach-Object { $_.FullName })
        Invoke-Native gcc ($flags + $src + @(
            '-o', 'build/host/tea_picking_robot_host.exe', '-lm'))
        $exe = Get-Item build/host/tea_picking_robot_host.exe
        Write-Host ("   built {0} ({1:N0} bytes)" -f $exe.Name, $exe.Length) `
            -ForegroundColor Green
    }

    if ($Target -eq 'test' -or $Target -eq 'both') {
        $testSrc = @(Get-ChildItem firmware/Tests/*.c -ErrorAction SilentlyContinue)
        if ($testSrc.Count -eq 0) {
            Write-Warning 'firmware/Tests is empty; skipping the unit-test build'
        } else {
            Write-Host '== building C unit tests ==' -ForegroundColor Cyan
            # The unit tests link the whole firmware *except* main_host.c, and
            # add the test tree.  This is deliberate: the tests exercise the
            # production translation units, not copies of them.
            $src = ($common | Where-Object { $_.Name -ne 'main_host.c' } |
                    ForEach-Object { $_.FullName }) +
                   ($testSrc | ForEach-Object { $_.FullName })
            Invoke-Native gcc ($flags + $src + @(
                '-o', 'build/host/tea_picking_robot_tests.exe', '-lm'))
            Write-Host '== running C unit tests ==' -ForegroundColor Cyan
            Invoke-Native ./build/host/tea_picking_robot_tests.exe @()
        }
    }

    if ($Run) {
        Write-Host '== running host simulation ==' -ForegroundColor Cyan
        & ./build/host/tea_picking_robot_host.exe @RunArgs
        exit $LASTEXITCODE
    }
} finally {
    Pop-Location
}
