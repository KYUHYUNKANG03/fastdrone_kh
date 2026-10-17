# sensor8_chain.ps1 - one command on PC-63: the sensor-condition experiment with the tune8 gains.
# ASCII only (Windows PowerShell reads a BOM-less file as ANSI).
#
#   irm "https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school/sensor8_chain.ps1" -OutFile $HOME\sensor8_chain.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\sensor8_chain.ps1 -Topic <ntfy topic>
#
# Results so far, at any time (does not disturb the run; writes a zip to the Desktop):
#   powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\sensor8_chain.ps1 -Collect
#
# What it runs (school/sensor8_run.py, plan fixed in the script):
#   controllers V13 F13 GSLQR (+ CPID on the V_L scenario), the tune8 records main8 used (sha256 pinned),
#   sensor seeds 1020-1039, 27 conditions in priority order: truth, nominal, tier 1 IMU/rpm, tier 2 navigation,
#   tier 3 combined, tier 4 model mismatch x sensor. About 7,500 trials, about 3-4 days on 7 workers.
#
# The background process:
#   0. makes sure fds8 exists and passed its checks (runs tune8_all.ps1 without a controller; quick on PC-63)
#   1. brings in the F13 and GSLQR tune8 records (GitHub records/ of the tune8 branch) next to V13 and CPID
#   2. writes the plan and the lane files (results\sensor8), runs the lanes in parallel, restarting any that die
#   3. keeps Windows awake, pushes progress to the phone (every 10% and when a condition finishes)
#   4. at the end writes summary.md and a zip of all rows to the Desktop
# Re-running the same line after a reboot/logoff resumes everything (finished trials are kept).
# Uses $HOME\fds8 only. Never edits control/, models/ or configs/ and never moves the git checkout.
param(
    [string]$Topic = '',
    [switch]$Run,
    [switch]$Collect,
    [switch]$Traces,
    [int]$Workers = 0,
    [switch]$Queue,
    [int]$PollSeconds = 300
)

$ErrorActionPreference = 'Continue'
$ExpectedHead = 'f5dfa079adbaccce13d6009b50ae296f5659082b'
$Raw = 'https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school'
$RawRecords = 'https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/records'
$Fds = Join-Path $HOME 'fds8'
$Vpy = Join-Path $Fds '.venv\Scripts\python.exe'
$Work = Join-Path $HOME 'sensor8'
$OutRel = 'results\sensor8'
$RunnerRel = 'results\sensor8\sensor8_run.py'
$Pc = $env:COMPUTERNAME

function Stamp { Get-Date -Format 'MM-dd HH:mm' }
function Say([string]$Text) {
    $line = "[{0}] {1}" -f (Get-Date -Format 'MM-dd HH:mm:ss'), $Text
    Write-Host $line
    if (Test-Path $Work) { Add-Content -Path (Join-Path $Work 'chain.log') -Value $line }
}
function Send-Note([string]$Message) {
    if ($Topic) {
        try { Invoke-RestMethod -Method Post -Uri "https://ntfy.sh/$Topic" -Body "$Pc sensor8: $Message $(Stamp)" -TimeoutSec 20 | Out-Null }
        catch { Say "ntfy failed: $($_.Exception.Message)" }
    }
}
function Send-File([string]$Path) {
    if ($Topic -and (Test-Path $Path)) {
        try {
            Invoke-RestMethod -Method Put -Uri "https://ntfy.sh/$Topic" -InFile $Path -Headers @{ Filename = (Split-Path $Path -Leaf); Title = "$Pc sensor8 file" } -TimeoutSec 60 | Out-Null
        } catch { Say "ntfy file upload failed: $($_.Exception.Message)" }
    }
}
function Stop-With([string]$Text) { Say "FAIL: $Text"; Send-Note "FAIL $Text"; exit 1 }
function Get-Desktop {
    $d = [Environment]::GetFolderPath('Desktop')
    if (-not $d -or -not (Test-Path $d)) { $d = Join-Path $HOME 'Desktop' }
    if (-not (Test-Path $d)) { $d = $HOME }
    return $d
}
function Invoke-Collect {
    # summary.md/csv + one zip of every finished row (no trajectories unless -Traces) on the Desktop
    $zip = Join-Path (Get-Desktop) ("sensor8_{0}_rows_{1}.zip" -f $Pc, (Get-Date -Format 'MMdd-HHmm'))
    $collectArgs = @($RunnerRel, 'collect', '--out', $OutRel, '--pc', $Pc, '--zip', $zip, '--logs', $Work)
    if ($Traces) { $collectArgs += '--traces' }
    Push-Location $Fds
    # capture the output: anything left uncaptured in a function becomes part of its return value
    $text = (& $Vpy @collectArgs 2>&1 | Out-String).Trim()
    $code = $LASTEXITCODE
    Pop-Location
    if ($text) { Say $text }
    if ($code -ne 0) { return $null }
    return $zip
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null

# ---------------- -Collect: results so far, then exit ----------------
if ($Collect) {
    if (-not (Test-Path (Join-Path $Fds $RunnerRel))) { Write-Host "FAIL: no sensor8 run in $Fds yet"; exit 1 }
    $env:PYTHONUTF8 = '1'
    Push-Location $Fds
    & $Vpy $RunnerRel status --out $OutRel --pc $Pc
    Pop-Location
    $zip = Invoke-Collect
    if ($null -eq $zip) { Write-Host 'FAIL: collect failed'; exit 1 }
    Write-Host "zip: $zip"
    Write-Host "summary: $(Join-Path $Fds "$OutRel\$Pc\summary.md")"
    exit 0
}

# ---------------- launcher: start the hidden background run and return ----------------
if (-not $Run) {
    $others = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*sensor8_chain.ps1*-Run*' })
    if ($others.Count -gt 0) { Say "sensor8 chain is already running (pid $($others[0].ProcessId)) - nothing to do"; exit 0 }
    $argList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath, '-Run', '-PollSeconds', "$PollSeconds", '-Workers', "$Workers")
    if ($Topic) { $argList += @('-Topic', $Topic) }
    if ($Queue) { $argList += '-Queue' }
    Start-Process powershell -WindowStyle Hidden -ArgumentList $argList
    Say "sensor8 chain started in the background on $Pc. Log: $Work\chain.log. You can close this window."
    exit 0
}

# ---------------- background run ----------------
try {
    Add-Type -Namespace Sensor8 -Name Power -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint flags);'
    [Sensor8.Power]::SetThreadExecutionState([uint32]2147483649) | Out-Null
} catch { Say "could not set keep-awake: $($_.Exception.Message)" }

# another main8 chain, main8 shard or tuning on this PC would share the 8 cores and 16 GB
function Get-Busy {
    return @(Get-CimInstance Win32_Process | Where-Object {
        $cl = $_.CommandLine
        $cl -and $_.ProcessId -ne $PID -and (
            ($cl -like '*main8_chain.ps1*-Run*') -or
            ($cl -like '*main_distributed*run-shard*') -or
            ($cl -like '*control.arena_tune *'))
    })
}
$busy = Get-Busy
if ($busy.Count -gt 0) {
    if (-not $Queue) { Stop-With "$($busy.Count) main8/tuning processes are running on this PC - add -Queue to wait for them" }
    Say "queued: $($busy.Count) other main8/tuning processes are running - starting when they finish"
    Send-Note 'queued - starts by itself when the run already on this PC finishes'
    while ((Get-Busy).Count -gt 0) { Start-Sleep -Seconds $PollSeconds }
    Say 'this PC is free now - starting'
}

# ---------------- 0. fds8 set up and checked (quick when already done) ----------------
$TuneAll = Join-Path $HOME 'tune8_all.ps1'
try { Invoke-WebRequest -Uri "$Raw/tune8_all.ps1" -OutFile $TuneAll -UseBasicParsing } catch { }
if (-not (Test-Path $TuneAll)) { Stop-With 'could not download tune8_all.ps1' }
Say 'checking fds8 (setup and checks; quick on a PC that ran main8)'
$setupArgs = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $TuneAll)
if ($Topic) { $setupArgs += @('-Topic', $Topic) }
& powershell @setupArgs *> (Join-Path $Work 'setup.log')
if ($LASTEXITCODE -ne 0) { Stop-With "fds8 setup/checks failed - see $Work\setup.log" }
if (-not (Test-Path $Vpy)) { Stop-With 'fds8\.venv not found after setup' }
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    $minGit = Join-Path $HOME 'mingit\cmd'
    if (Test-Path (Join-Path $minGit 'git.exe')) { $env:Path = "$minGit;$env:Path" }
    else { Stop-With 'git not found after setup' }
}
Set-Location $Fds
$head = (git rev-parse HEAD).Trim()
if ($head -ne $ExpectedHead) { Stop-With "fds8 is at $head, not the tune8 commit - run tune8_all.ps1 once to restore it" }
$env:OMP_NUM_THREADS = '1'; $env:OPENBLAS_NUM_THREADS = '1'; $env:VECLIB_MAXIMUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'; $env:PYTHONUTF8 = '1'

# ---------------- 1. runner script and the four tune8 records ----------------
$OutDir = Join-Path $Fds $OutRel
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$Runner = Join-Path $Fds $RunnerRel
$fresh = Join-Path $Work 'sensor8_run.py'
try { Invoke-WebRequest -Uri "$Raw/sensor8_run.py" -OutFile $fresh -UseBasicParsing -ErrorAction Stop } catch { Say "download of sensor8_run.py failed: $($_.Exception.Message)" }
if (Test-Path $fresh) {
    if (-not (Test-Path $Runner)) { Copy-Item $fresh $Runner }
    elseif ((Get-FileHash $fresh).Hash -ne (Get-FileHash $Runner).Hash) {
        if (Test-Path (Join-Path $OutDir 'sensor8.plan.json')) {
            Say 'sensor8_run.py on GitHub differs from the one this run started with - keeping the old one (finished trials stay valid)'
        } else { Copy-Item $fresh $Runner -Force }
    }
}
if (-not (Test-Path $Runner)) { Stop-With 'sensor8_run.py not available' }

$TuneDir = Join-Path $Fds 'results\arena\tuning\tune8'
$FinalDir = Join-Path $Fds 'results\arena\tuning\tune8_final'
New-Item -ItemType Directory -Force -Path $FinalDir | Out-Null
foreach ($c in @('V13', 'F13', 'GSLQR', 'CPID')) {
    $dst = Join-Path $FinalDir "$c.record.json"
    if (Test-Path $dst) { continue }
    $src = Join-Path $TuneDir "$c.record.json"
    if (Test-Path $src) { Copy-Item $src $dst; Say "record $c from tune8"; continue }
    # never re-encode: the sha256 of the exact bytes is checked (Invoke-WebRequest -OutFile keeps the bytes)
    $tmp = Join-Path $Work "$c.record.json"
    try {
        Invoke-WebRequest -Uri "$RawRecords/$c.record.json" -OutFile $tmp -UseBasicParsing -ErrorAction Stop
        Copy-Item $tmp $dst
        Say "record $c from GitHub records/"
    } catch { Stop-With "could not get $c.record.json ($($_.Exception.Message))" }
}

# ---------------- 2. plan and lanes ----------------
Say 'writing the plan (about 1 min)'
& $Vpy $RunnerRel plan --root . --out $OutRel *> (Join-Path $Work 'plan.log')
if ($LASTEXITCODE -ne 0) { Stop-With "plan refused - see $Work\plan.log" }
$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
Say "disk free $freeGb GB"
if ($freeGb -lt 5) { Stop-With "only $freeGb GB free on C:" }

$lanesFile = Join-Path $OutDir "$Pc.lanes.json"
if (-not (Test-Path $lanesFile)) {
    if ($Workers -gt 0) { $n = $Workers }
    else {
        $cores = (Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfCores -Sum).Sum
        # an F13 process holds about 1.2 GiB
        $text = (& $Vpy -m control.arena_suggest_workers 1.2 --physical-cores $cores 2>&1 | Out-String)
        $n = 1
        if ($text -match 'scenario-workers\s+(\d+)') { $n = [int]$Matches[1] }
    }
    $n = [math]::Max(1, $n)
    & $Vpy $RunnerRel lanes --out $OutRel --pc $Pc --n $n *> (Join-Path $Work 'lanes.log')
    if ($LASTEXITCODE -ne 0) { Stop-With "lanes refused - see $Work\lanes.log" }
}
$counts = (& $Vpy -c "import json,sys; d=json.load(open(sys.argv[1], encoding='utf-8')); print(' '.join(str(len(l['trials'])) for l in d['lanes']))" $lanesFile | Out-String).Trim()
$want = @($counts -split '\s+' | Where-Object { $_ } | ForEach-Object { [int]$_ })
$n = $want.Count
if ($n -lt 1) { Stop-With 'could not read the lanes file' }
$total = 0
foreach ($w in $want) { $total += $w }
Say "lanes $n, trials $total, results in $OutDir\$Pc"
Send-Note "started: $n workers, $total trials, disk $freeGb GB"

function Get-LaneHave([int]$I) {
    $folder = Join-Path $OutDir "$Pc\lane$I\trials"
    if (-not (Test-Path $folder)) { return 0 }
    return @(Get-ChildItem $folder -Filter '*.json').Count
}
function Test-Lane([int]$I) {
    $procs = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*sensor8_run.py*run-lane --index $I *" })
    return ($procs.Count -gt 0)
}
function Start-Lane([int]$I) {
    $log = Join-Path $Work "lane_$I.log"
    $err = Join-Path $Work "lane_$I.err"
    Start-Process $Vpy -WindowStyle Hidden -WorkingDirectory $Fds -RedirectStandardOutput $log -RedirectStandardError $err -ArgumentList $RunnerRel, 'run-lane', '--index', "$I", '--pc', $Pc, '--root', '.', '--out', $OutRel | Out-Null
}
function Get-DoneConditions {
    $t = (& $Vpy $RunnerRel status --out $OutRel --pc $Pc --done-conditions 2>$null | Out-String).Trim()
    return @($t -split '\s+' | Where-Object { $_ })
}

# ---------------- 3. run and watch ----------------
$restarts = @{}
for ($i = 0; $i -lt $n; $i++) { $restarts[$i] = 0 }
$noted = @{}
foreach ($c in (Get-DoneConditions)) { $noted[$c] = $true }
$lastTenth = -1
$lastDone = -1
$lastChange = Get-Date
$stallSent = $false
while ($true) {
    $running = 0
    $done = 0
    for ($i = 0; $i -lt $n; $i++) {
        $have = Get-LaneHave $i
        $done += $have
        if ($have -ge $want[$i]) { continue }
        if (Test-Lane $i) { $running += 1; continue }
        if ($restarts[$i] -ge 5) { Send-Note "lane $i keeps stopping - see $Work\lane_$i.err"; $restarts[$i] = -100; continue }
        if ($restarts[$i] -lt 0) { continue }
        if ($restarts[$i] -gt 0) { Say "restarting lane $i (attempt $($restarts[$i]))" }
        $restarts[$i] += 1
        Start-Lane $i
        $running += 1
    }
    if ($done -ne $lastDone) { $lastDone = $done; $lastChange = Get-Date; $stallSent = $false }
    $finished = @(Get-DoneConditions | Where-Object { -not $noted[$_] })
    if ($finished.Count -gt 0) {
        foreach ($c in $finished) { $noted[$c] = $true }
        Say "condition done: $($finished -join ', ')"
        Send-Note "condition done: $($finished -join ', ') ($done/$total trials)"
    }
    $tenth = [math]::Floor(10 * $done / [math]::Max(1, $total))
    if ($tenth -gt $lastTenth) {
        if ($lastTenth -ge 0) { Send-Note "$done/$total trials" }
        $lastTenth = $tenth
    }
    if ($done -ge $total) { break }
    if ($running -eq 0) { Stop-With "no lane is running and $($total - $done) trials are left - see $Work" }
    if (((New-TimeSpan -Start $lastChange -End (Get-Date)).TotalMinutes -gt 180) -and -not $stallSent) {
        Send-Note 'no new trial for 3 h - check this PC'; $stallSent = $true
    }
    Start-Sleep -Seconds $PollSeconds
}

# ---------------- 4. summary and zip ----------------
$zip = Invoke-Collect
$summary = Join-Path $OutDir "$Pc\summary.md"
Send-File $summary
$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
if ($null -ne $zip) {
    Say "sensor8 finished: $done/$total trials. zip $zip (disk free $freeGb GB)"
    Send-Note "DONE $done/$total trials - zip on the Desktop: $(Split-Path $zip -Leaf)"
} else {
    Say "sensor8 finished: $done/$total trials, but the zip failed - run the -Collect line"
    Send-Note "DONE $done/$total trials - zip failed, run the -Collect line"
}
exit 0
