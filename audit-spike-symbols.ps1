# Throwaway audit helper for the step-1 spike (doc/driver-spike-handover.md §2.1).
#
# WHAT IT DOES (default): a case-sensitive grep for a CALL to each symbol in
# spike-public-symbols.txt across the spike test sources. One line per symbol:
#
#   <header>  <symbol>  <file:line>  <the line's text>
#
# A free function must appear as `f(` or `spike::f(`; a `Class::method` row matches either
# `Class::method(` or a member call `.method(` / `->method(`, because that is what C++
# tests actually write. Destructors are excluded from the audit entirely (see below).
#
# The printed line always contains the text that matched. The script verifies that before
# printing and throws if it ever fails.
#
# -MemberCalls additionally resolves CONSTRUCTORS, which the default mode cannot: a
# construction is `spike::Logger logger;`, not a call spelled `Logger::Logger(`.
#
# NEITHER MODE IS COVERAGE. A call site in a test file does not prove the line ran, and
# proves nothing about assertions. Same-named methods on different classes collide
# (`remove`, `init`, `cleanup`, `initContext`, ...) and cannot be told apart by text.
# Symbols reached only indirectly (`cross`/`mul` via `rotate`/`compose`) report NONE while
# still executing. Real line coverage needs OpenCppCoverage — still open decision 3 in the
# handover.
#
# USAGE
#   powershell -ExecutionPolicy Bypass -File .\audit-spike-symbols.ps1
#   ... -MemberCalls                   # also resolves constructors
#   ... -Symbol ServerEnvironment      # only rows whose symbol matches this regex
#   ... -ZeroOnly                      # only the symbols with no match
#   ... -DriverTestOnly                # only symbols matched exclusively in SpikeDriverTest.cpp
#   ... -Dataset <path> -TestDirectory <path> -TestFilter <glob>

[CmdletBinding()]
param(
    [switch]$MemberCalls,
    [string]$Symbol,
    [switch]$ZeroOnly,
    [switch]$DriverTestOnly,
    [string]$Dataset,
    [string]$TestDirectory,
    [string]$TestFilter = 'Spike*Test.cpp'
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $Dataset)
{
    $Dataset = Join-Path $repoRoot 'spike-public-symbols.txt'
}
if (-not $TestDirectory)
{
    $TestDirectory = Join-Path $repoRoot 'tests'
}

if (-not (Test-Path -LiteralPath $Dataset))
{
    throw "Dataset not found: $Dataset"
}

# ---- dataset ----------------------------------------------------------------------

$lineNumber = 0
$entries = foreach ($line in Get-Content -LiteralPath $Dataset)
{
    $lineNumber++
    $text = $line.Trim()
    if ($text -eq '' -or $text.StartsWith('#'))
    {
        continue
    }

    $parts = $text -split '\|'
    if ($parts.Count -ne 2)
    {
        throw "$Dataset line ${lineNumber}: expected '<file>|<symbol>', got '$text'"
    }

    [pscustomobject]@{
        Header = $parts[0].Trim()
        Symbol = $parts[1].Trim()
    }
}

$entries = @($entries)
if ($entries.Count -eq 0)
{
    throw "No symbols read from $Dataset"
}

# Destructors are dropped from the audit, not reported as NONE. No C++ in these tests
# calls `~Class()` by name: a destructor runs when the object leaves scope, so text search
# can neither confirm nor deny it and a NONE row would only be noise. Their execution is
# established by the objects the tests construct, and would show up in real line coverage.
$destructors = @($entries | Where-Object { $_.Symbol.Contains('~') })
$entries = @($entries | Where-Object { -not $_.Symbol.Contains('~') })
if ($entries.Count -eq 0)
{
    throw "No non-destructor symbols read from $Dataset"
}

if ($Symbol)
{
    $entries = @($entries | Where-Object { $_.Symbol -match $Symbol })
    if ($entries.Count -eq 0)
    {
        throw "No dataset symbol matches -Symbol '$Symbol'"
    }
}

# ---- search patterns --------------------------------------------------------------

# Default mode: where the symbol is CALLED. The name must be followed by an argument
# list — without the `\(` the word "logs" in a comment matched the symbol `log`.
#
#   free function   `f(` / `spike::f(`, but NOT `obj.f(` — the `.`/`->` exclusions stop
#                   `context.properties.add(...)` (a fake's member) counting as spike::add
#   Class::method   either the qualified form `Class::method(` (explicit/static call) or
#                   the way C++ actually calls it: `.method(` / `->method(`
#
# A `"` immediately before the name is excluded in both cases: since serveFactoryRequest
# logs the line `HmdDriverFactory("%s")`, the assertion `logged("HmdDriverFactory(\"...`
# otherwise counted as a call to the DLL export, which no unit test can make.
function Get-LiteralPattern([string]$symbol)
{
    $qualified = $symbol -split '::'

    if ($qualified.Count -eq 1)
    {
        return "(?<![\w~.`"])(?<!->)" + [regex]::Escape($symbol) + "\s*\("
    }

    $member = $qualified[1]
    return "(?<![\w~`"])" + [regex]::Escape($symbol) + "\s*\(" +
           "|(?:\.|->)\s*" + [regex]::Escape($member) + "\s*\("
}

# -MemberCalls mode: how the symbol is actually written at a call site.
#
# The member-call alternative cannot carry a `(?<![\w~])` lookbehind: in
# `properties.add(` the `.` is preceded by a word character, so the lookbehind would
# reject the very call it is meant to find.
#
# COLLISION, UNFIXABLE BY TEXT: `.method(` names no class, so two spike types with the
# same method name (`init`, `cleanup`, ...) cannot be told apart, and such a row may
# report the other type's call. The summary lists every affected name.
function Get-CallPattern([string]$symbol)
{
    $qualified = $symbol -split '::'

    if ($qualified.Count -eq 1)
    {
        # Free function. The lookbehinds stop it matching a member call of the same name
        # (`context.properties.add(...)` is the fake's member, not spike::add).
        return "(?<![\w~.])(?<!->)(?:spike::)?" + [regex]::Escape($qualified[0]) + "\s*\("
    }

    $class = $qualified[0]
    $member = $qualified[1]

    if ($member -eq "~$class")
    {
        # No C++ syntax calls a destructor by name in these tests; NONE is the answer.
        return [regex]::Escape($member) + "\s*\("
    }

    if ($member -eq $class)
    {
        # A construction: `spike::Logger logger;`, `Fake x{...}`, `Class(args)`. The \b
        # stops class SpikeObserver matching the fixture name SpikeObserverTest(.
        return "(?<![\w~])(?:spike::)?" + [regex]::Escape($class) + "\b\s*(?:\(|\{|[A-Za-z_]\w*\s*[;{(=])"
    }

    return "(?:\.|->)\s*" + [regex]::Escape($member) + "\s*\("
}

# ---- search -----------------------------------------------------------------------

$testFiles = @(Get-ChildItem -Path $TestDirectory -Filter $TestFilter | ForEach-Object { $_.FullName })
if ($testFiles.Count -eq 0)
{
    throw "No test sources matching '$TestFilter' found in $TestDirectory"
}

$results = foreach ($entry in $entries)
{
    if ($MemberCalls)
    {
        $pattern = Get-CallPattern $entry.Symbol
    }
    else
    {
        $pattern = Get-LiteralPattern $entry.Symbol
    }

    # -CaseSensitive is not optional: Select-String defaults to case-insensitive, which
    # made `SpikeServer::init` match `server->Init(` - a different function.
    # NOT named $matches: that is a PowerShell automatic variable, and -notmatch below
    # would overwrite it.
    $hits = @(Select-String -Path $testFiles -Pattern $pattern -CaseSensitive)

    # Drop declarations/definitions in both modes: a fake's
    # `std::vector<Sample> sample() override` or `void add(uint32_t index, ...)` is the
    # test's own code, not a call into src/spike. Detected as "a type token immediately
    # precedes the name", which a call site never has.
    $bareName = ($entry.Symbol -split '::')[-1]
    $definition = "(?:^|[\s,(])[A-Za-z_][\w:<>,*&\s]*\s+" + [regex]::Escape($bareName) + "\s*\("
    $hits = @($hits | Where-Object { $_.Line -notmatch $definition })

    # Self-check: whatever is printed must really be on that line.
    foreach ($hit in $hits)
    {
        $matched = $hit.Matches[0].Value
        if (-not $hit.Line.Contains($matched))
        {
            throw ("Script bug: reporting {0}:{1} for '{2}' with match '{3}', which is not on the line: {4}" -f
                   $hit.Filename, $hit.LineNumber, $entry.Symbol, $matched, $hit.Line.Trim())
        }
    }

    [pscustomobject]@{
        Header  = $entry.Header
        Symbol  = $entry.Symbol
        Matches = $hits
        Files   = @($hits | ForEach-Object { $_.Filename } | Sort-Object -Unique) -join ','
    }
}

$results = @($results)

$shown = $results
if ($ZeroOnly)
{
    $shown = @($results | Where-Object { $_.Matches.Count -eq 0 })
}
elseif ($DriverTestOnly)
{
    $shown = @($results | Where-Object { $_.Matches.Count -gt 0 -and $_.Files -eq 'SpikeDriverTest.cpp' })
}

# ---- report: one line per symbol --------------------------------------------------

foreach ($result in $shown)
{
    if ($result.Matches.Count -eq 0)
    {
        Write-Output ("{0,-20} {1,-48} {2,-34} {3}" -f $result.Header, $result.Symbol, 'NONE', '')
        continue
    }

    $first = $result.Matches[0]
    $location = "{0}:{1}" -f $first.Filename, $first.LineNumber
    Write-Output ("{0,-20} {1,-48} {2,-34} {3}" -f $result.Header, $result.Symbol, $location,
                  $first.Line.Trim())
}

$zero = @($results | Where-Object { $_.Matches.Count -eq 0 })
$mode = if ($MemberCalls) { 'call search incl. constructors (-MemberCalls)' } else { 'call search' }
Write-Output ''
Write-Output "Mode: $mode, case-sensitive"
Write-Output "Dataset: $Dataset"
Write-Output "Test sources: $TestDirectory\$TestFilter ($($testFiles.Count) files)"
Write-Output "Symbols audited: $($results.Count) (destructors excluded: $($destructors.Count))"
Write-Output "No match: $($zero.Count)"

# A `.method(` match names no class, so any member name declared by two spike types has
# rows that may be reporting the OTHER type's call. Named here instead of quietly trusted.
$ambiguous = @($results |
    Where-Object { $_.Symbol.Contains('::') } |
    Group-Object { ($_.Symbol -split '::')[-1] } |
    Where-Object { $_.Count -gt 1 } |
    ForEach-Object { $_.Name })
if ($ambiguous.Count -gt 0)
{
    Write-Output ("Ambiguous member names (matched line may belong to the other class): {0}" -f
                  ($ambiguous -join ', '))
}

Write-Output 'Neither mode is coverage: a match is not proof of execution or of an assertion.'
