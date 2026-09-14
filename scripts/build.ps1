<#
=============================================================================
 scripts/build.ps1 - thin wrapper around the root Makefile, for Windows

 This script deliberately contains **no build logic of its own**.  It resolves
 the project root, forwards its arguments to `make`, and translates the
 friendly positional action names into make targets.  Anything more would be a
 second place where the build is defined, and the second place is always the one
 that goes stale.

 Why this exists alongside build.sh
 ----------------------------------
 On the machine this port was developed on, `make` (GNU Make 3.80) uses
 `cmd.exe` as its recipe shell because `sh.exe` is not on PATH.  That is why the
 Makefile itself is written with cmd.exe-compatible recipes.  `build.sh` needs a
 real bash; this script needs nothing but PowerShell, which Windows always has.
 Neither wrapper is required: `make <target>` works directly.

 Usage
 -----
   .\scripts\build.ps1                  # same as: make host
   .\scripts\build.ps1 host             # host firmware binary + unit tests
   .\scripts\build.ps1 test             # build and run the host unit tests
   .\scripts\build.ps1 stm32            # STM32F407VET6 ELF + .bin + .hex
   .\scripts\build.ps1 freertos         # make stm32 RTOS=freertos
   .\scripts\build.ps1 size             # size of the STM32 image
   .\scripts\build.ps1 clean
   .\scripts\build.ps1 help
   .\scripts\build.ps1 all              # clean, host, test, stm32
   .\scripts\build.ps1 -MakeArgs 'stm32','RTOS=freertos'   # raw make arguments

 Exit status is make's, so this is safe to use as a CI step.
=============================================================================
#>

[CmdletBinding()]
param(
    # Positional action, e.g. host, test, stm32, freertos, size, clean, all.
    [Parameter(Position = 0)]
    [string] $Action = 'host',

    # Raw argument list forwarded to make verbatim, e.g. 'stm32','RTOS=freertos'.
    [Parameter(Position = 1, ValueFromRemainingArguments = $true)]
    [string[]] $MakeArgs = @()
)

$ErrorActionPreference = 'Stop'

function Show-Usage {
    Get-Content -LiteralPath $PSCommandPath |
        Select-Object -Skip 4 -First 34 |
        ForEach-Object { $_ -replace '^#\s?', '' }
}

# `help` is handled here, before the try/finally below.  Calling `exit 0` from
# inside a `try` block with a `finally` makes Windows PowerShell 5.1 report an
# exit code of -1 after the finally block has run - a wrapper whose "print the
# help" action returns a failure code is worse than no wrapper, because a CI job
# then treats a successful invocation as a failed one.
if ($Action -in @('help', '-h', '--help')) {
    Show-Usage
    exit 0
}

# The result is communicated through `$script:MakeExit` rather than through the
# function's return value.  A PowerShell function returns *everything* it emits,
# so `$rc = Invoke-MakeTarget ...` would capture make's stdout as well as the
# exit code and produce garbage such as
#     "==> size: FAILED (exit code --- arm-none-eabi-size --- 57500 ... 0)".
$script:MakeExit = 0

function Invoke-MakeTarget {
    param([string[]] $Targets)

    Write-Host "==> make $($Targets -join ' ')"
    & make @Targets
    # Not returned, and `exit` must happen *after* the finally block below:
    # Windows PowerShell 5.1 reports -1 as the process exit code when `exit`
    # runs inside a `try` that has a `finally`.
    $script:MakeExit = $LASTEXITCODE
}

# --- Locate the project root (the directory containing the Makefile) ---------
$scriptDir   = Split-Path -Parent $PSCommandPath
$projectRoot = Split-Path -Parent $scriptDir

if (-not (Test-Path -LiteralPath (Join-Path $projectRoot 'Makefile'))) {
    Write-Error "build.ps1: no Makefile in $projectRoot"
    exit 2
}

Push-Location $projectRoot
$makeExit = 0
try {
    if (-not (Get-Command make -ErrorAction SilentlyContinue)) {
        Write-Host "build.ps1: 'make' is not on PATH. Install GNU Make, or use the compiler directly." -ForegroundColor Red
        $makeExit = 2
    }
    elseif ($MakeArgs.Count -gt 0) {
        # Raw passthrough: the caller knows exactly what it wants.
        Invoke-MakeTarget (@($Action) + $MakeArgs)
        $makeExit = $script:MakeExit
    }
    else {
        switch ($Action.ToLowerInvariant()) {
            'host'      { Invoke-MakeTarget @('host');      $makeExit = $script:MakeExit }
            'test'      { Invoke-MakeTarget @('test');      $makeExit = $script:MakeExit }
            'stm32'     { Invoke-MakeTarget @('stm32');     $makeExit = $script:MakeExit }
            'target'    { Invoke-MakeTarget @('stm32');     $makeExit = $script:MakeExit }
            'firmware'  { Invoke-MakeTarget @('stm32');     $makeExit = $script:MakeExit }
            'freertos'  { Invoke-MakeTarget @('stm32', 'RTOS=freertos'); $makeExit = $script:MakeExit }
            'rtos'      { Invoke-MakeTarget @('stm32', 'RTOS=freertos'); $makeExit = $script:MakeExit }
            'size'      { Invoke-MakeTarget @('size');      $makeExit = $script:MakeExit }
            'sections'  { Invoke-MakeTarget @('sections');  $makeExit = $script:MakeExit }
            'symbols'   { Invoke-MakeTarget @('symbols');   $makeExit = $script:MakeExit }
            'clean'     { Invoke-MakeTarget @('clean');     $makeExit = $script:MakeExit }
            'distclean' { Invoke-MakeTarget @('distclean'); $makeExit = $script:MakeExit }
            'all' {
                # Clean first so a stale object can never make a later step look
                # green, and stop at the first failure rather than reporting a
                # summary for a run that was already broken.
                foreach ($t in @('clean', 'host', 'test', 'stm32')) {
                    Invoke-MakeTarget @($t)
                    $makeExit = $script:MakeExit
                    if ($makeExit -ne 0) { break }
                }
            }
            default {
                Write-Host "build.ps1: unknown action '$Action'" -ForegroundColor Red
                Write-Host ''
                Show-Usage
                $makeExit = 2
            }
        }
    }

    if ($makeExit -eq 0) {
        Write-Host "==> $Action`: OK" -ForegroundColor Green
    } else {
        Write-Host "==> $Action`: FAILED (exit code $makeExit)" -ForegroundColor Red
    }
}
finally {
    Pop-Location
}

# The exit code is set here, outside the try/finally: Windows PowerShell 5.1
# reports -1 rather than the requested code when `exit` runs inside a `try` that
# has a `finally`, which would turn every successful invocation into a CI failure.
exit $makeExit
