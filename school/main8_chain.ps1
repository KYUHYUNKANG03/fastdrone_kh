# main8_chain.ps1 - one command per school PC: run this PC's part of the tune8 main experiment.
# Adapted from main7_chain.ps1. ASCII only (Windows PowerShell reads a BOM-less file as ANSI).
#
# One line (PowerShell), with the controllers this PC tuned in tune8:
#   irm "https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school/main8_chain.ps1" -OutFile $HOME\main8_chain.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\main8_chain.ps1 -Controllers F13 -Topic <ntfy topic>
#   PC-61: -Controllers M17   PC-62: -Controllers F13   PC-63: -Controllers V13,CPID   PC-64: -Controllers GSLQR
#
# It starts a hidden background process and returns; you can close the window. The background process:
#   1. waits until each controller's tune8 tuning is complete at 60 (it never starts tuning - tune8_all.ps1 does)
#   2. waits for (or runs) the pre-main gate; a controller whose gate FAILED stops the chain - no main run for it
#   3. copies the final records to results\arena\tuning\tune8_final\ and writes this PC's main spec
#      (school/main8_make_spec.py: seeds 1020-1039, ladder 1020-1029, ladder only with V13)
#   4. makes shards and runs them in parallel (control.main_distributed run-shard), restarting any that die
#   5. keeps Windows from sleeping while it works (screen may turn off) and pushes progress to the phone
# Re-running the same line after a reboot/logoff resumes everything (finished trials are kept).
# Uses $HOME\fds8 only. Never edits control/, models/ or configs/ and never moves the git checkout.
param(
    [Parameter(Mandatory = $true)][string[]]$Controllers,
    [string]$Topic = '',
    [switch]$Run,
    [int]$Workers = 0,
    [int]$PollSeconds = 300
)

$ErrorActionPreference = 'Continue'
$ExpectedHead = 'f5dfa079adbaccce13d6009b50ae296f5659082b'
$Budget = 60
$Raw = 'https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school'
$Fds = Join-Path $HOME 'fds8'
$Vpy = Join-Path $Fds '.venv\Scripts\python.exe'
$Work = Join-Path $HOME 'main8'
$Gib = @{ M17 = 2.0; F13 = 1.2; V13 = 0.6; GSLQR = 0.6; CPID = 0.6 }
$Order = @('V13', 'M17', 'F13', 'GSLQR', 'CPID')

# accept "V13,CPID" as well as V13 CPID
$Controllers = @($Controllers | ForEach-Object { $_ -split ',' } | ForEach-Object { $_.Trim() } | Where-Object { $_ })
foreach ($c in $Controllers) { if ($Order -notcontains $c) { Write-Host "FAIL: unknown controller $c"; exit 1 } }
$Controllers = @($Order | Where-Object { $Controllers -contains $_ })
$Tag = ($Controllers -join '_')
$Pc = $env:COMPUTERNAME

function Stamp { Get-Date -Format 'MM-dd HH:mm' }
function Say([string]$Text) {
    $line = "[{0}] {1}" -f (Get-Date -Format 'MM-dd HH:mm:ss'), $Text
    Write-Host $line
    if (Test-Path $Work) { Add-Content -Path (Join-Path $Work "chain_$Tag.log") -Value $line }
}
function Send-Note([string]$Message) {
    if ($Topic) {
        try { Invoke-RestMethod -Method Post -Uri "https://ntfy.sh/$Topic" -Body "$Pc main8 $Tag`: $Message $(Stamp)" -TimeoutSec 20 | Out-Null }
        catch { Say "ntfy failed: $($_.Exception.Message)" }
    }
}
function Stop-With([string]$Text) { Say "FAIL: $Text"; Send-Note "FAIL $Text"; exit 1 }

New-Item -ItemType Directory -Force -Path $Work | Out-Null
if (-not (Test-Path $Vpy)) { Stop-With 'fds8\.venv not found - run tune8_all.ps1 on this PC first' }

# ---------------- launcher: start the hidden background run and return ----------------
if (-not $Run) {
    $others = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*main8_chain.ps1*-Run*' -and $_.CommandLine -like "*-Controllers $($Controllers -join ',') *" })
    if ($others.Count -gt 0) { Say "main8 chain for $Tag is already running (pid $($others[0].ProcessId)) - nothing to do"; exit 0 }
    $argList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath, '-Run', '-Controllers', ($Controllers -join ','), '-PollSeconds', "$PollSeconds", '-Workers', "$Workers")
    if ($Topic) { $argList += @('-Topic', $Topic) }
    Start-Process powershell -WindowStyle Hidden -ArgumentList $argList
    Say "main8 chain started in the background for $Tag on $Pc. Log: $Work\chain_$Tag.log. You can close this window."
    exit 0
}

# ---------------- background run ----------------
Set-Location $Fds
$head = (git rev-parse HEAD).Trim()
if ($head -ne $ExpectedHead) { Stop-With "fds8 is at $head, not the tune8 commit - run tune8_all.ps1 once to restore it" }
$env:OMP_NUM_THREADS = '1'; $env:OPENBLAS_NUM_THREADS = '1'; $env:VECLIB_MAXIMUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'; $env:PYTHONUTF8 = '1'

# keep the system awake while this process lives (screen may still turn off; logoff/reboot still stop it)
try {
    Add-Type -Namespace Main8 -Name Power -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint flags);'
    [Main8.Power]::SetThreadExecutionState([uint32]2147483649) | Out-Null
} catch { Say "could not set keep-awake: $($_.Exception.Message)" }

$Helper = Join-Path $Work 'main8_make_spec.py'
$Gate = Join-Path $Work 'tune8_gate.py'
try {
    Invoke-WebRequest -Uri "$Raw/main8_make_spec.py" -OutFile $Helper -UseBasicParsing
    Invoke-WebRequest -Uri "$Raw/tune8_gate.py" -OutFile $Gate -UseBasicParsing
} catch { if (-not (Test-Path $Helper) -or -not (Test-Path $Gate)) { Stop-With "download failed: $($_.Exception.Message)" } }

$TuneDir = Join-Path $Fds 'results\arena\tuning\tune8'
$FinalDir = Join-Path $Fds 'results\arena\tuning\tune8_final'
$MainDir = Join-Path $Fds 'results\main8'
New-Item -ItemType Directory -Force -Path $FinalDir, $MainDir | Out-Null
$cores = (Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfCores -Sum).Sum

function Get-Record([string]$C) {
    $p = Join-Path $TuneDir "$C.record.json"
    if (-not (Test-Path $p)) { return $null }
    try { return (Get-Content $p -Raw | ConvertFrom-Json) } catch { return $null }
}
function Test-Final([string]$C) {
    $r = Get-Record $C
    return ($null -ne $r) -and ($r.status -eq 'complete') -and ([int]$r.budget -eq $Budget) -and ([int]$r.spent -eq $Budget)
}
function Test-Tuning([string]$C) {
    $procs = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*arena_tune*--controllers $C *arena_tune8*" })
    return ($procs.Count -gt 0)
}
function Test-GateRunning([string]$C) {
    $procs = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*tune8_gate.py*' -and $_.CommandLine -like "*--controllers $C*" })
    return ($procs.Count -gt 0)
}

# ---------------- 1. wait for tuning to finish at 60 ----------------
Send-Note "chain started (waiting for tune8 at $Budget, gate, then main)"
$waitSent = @{}
while ($true) {
    $pending = @($Controllers | Where-Object { -not (Test-Final $_) })
    if ($pending.Count -eq 0) { break }
    foreach ($c in $pending) {
        if (-not (Test-Tuning $c)) {
            Stop-With "$c tune8 tuning is not complete and not running - run the tune8_all.ps1 line for $c first"
        }
        if (-not $waitSent[$c]) {
            $r = Get-Record $c
            $spent = if ($null -ne $r) { [int]$r.spent } else { 0 }
            Say "waiting for $c tuning ($spent/$Budget)"
            $waitSent[$c] = $true
        }
    }
    Start-Sleep -Seconds $PollSeconds
}
Say "tuning complete at $Budget for $($Controllers -join ', ')"

# ---------------- 2. gate ----------------
foreach ($c in $Controllers) {
    $gateJson = Join-Path $TuneDir "$c.gate.json"
    # tune8_all's watcher runs the gate when tuning completes (it polls every 5 min). Give it 10 min to start,
    # wait while a gate is running (up to 2 h), and only then run the gate here.
    $waited = 0
    while ((-not (Test-Path $gateJson)) -and ($waited -lt 7200)) {
        if ((-not (Test-GateRunning $c)) -and ($waited -ge 600)) { break }
        Start-Sleep -Seconds 60; $waited += 60
    }
    if (-not (Test-Path $gateJson)) {
        Say "running the gate for $c"
        & $Vpy $Gate --root $Fds --controllers $c *> (Join-Path $Work "gate_$c.log")
    }
    if (-not (Test-Path $gateJson)) { Stop-With "no gate result for $c (see $Work\gate_$c.log)" }
    $g = $null
    try { $g = Get-Content $gateJson -Raw | ConvertFrom-Json } catch { }
    if ($null -eq $g) { Stop-With "could not read $gateJson" }
    if (-not $g.gate_pass) { Stop-With "$c gate FAIL - main experiment NOT started (see $gateJson)" }
    Say "$c gate PASS"
}
Send-Note "gate PASS for $($Controllers -join ', '), preparing main"

# ---------------- 3. final records + spec ----------------
foreach ($c in $Controllers) {
    foreach ($ext in @('record.json', 'jsonl')) {
        $src = Join-Path $TuneDir "$c.$ext"
        $dst = Join-Path $FinalDir "$c.$ext"
        if (-not (Test-Path $dst)) { Copy-Item $src $dst }
        elseif ((Get-FileHash $src).Hash -ne (Get-FileHash $dst).Hash) { Stop-With "$dst differs from the tuning output - do not edit tune8_final by hand" }
    }
}
$specRel = "results/main8/$($Pc)_$Tag.spec.json"
$shardsRel = "results/main8/$($Pc)_$Tag.shards.json"
$outDir = Join-Path $MainDir "$($Pc)_$Tag"
$spec = Join-Path $Fds $specRel
$shards = Join-Path $Fds $shardsRel
if (-not (Test-Path $spec)) {
    & $Vpy $Helper --root $Fds --controllers @Controllers --out $spec *> (Join-Path $Work "spec_$Tag.log")
    if ($LASTEXITCODE -ne 0) { Stop-With "main spec could not be made (see $Work\spec_$Tag.log)" }
}
$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
Say "disk free $freeGb GB"
if ($freeGb -lt 5) { Stop-With "only $freeGb GB free on C: - main results need several GB" }
if ($freeGb -lt 20) { Send-Note "WARNING disk free only $freeGb GB (main may need ~20 GB)" }

# ---------------- 4. shards ----------------
if ($Workers -gt 0) {
    $workersNow = $Workers
} else {
    $perWorker = ($Controllers | ForEach-Object { $Gib[$_] } | Measure-Object -Maximum).Maximum
    $text = (& $Vpy -m control.arena_suggest_workers $perWorker --physical-cores $cores 2>&1 | Out-String)
    $workersNow = 1
    if ($text -match 'scenario-workers\s+(\d+)') { $workersNow = [int]$Matches[1] }
    # PC-61 measurement: an M17 process holds about 3.7 GiB, not 2.0 - never more than 2 on a 16 GiB PC
    if ($Controllers -contains 'M17') { $workersNow = [math]::Min($workersNow, 2) }
}
$workersNow = [math]::Max(1, $workersNow)
if (-not (Test-Path $shards)) {
    & $Vpy -m control.main_distributed make-shards --spec $specRel --n $workersNow --out $shardsRel *> (Join-Path $Work "shards_$Tag.log")
    if ($LASTEXITCODE -ne 0) { Stop-With "make-shards refused (see $Work\shards_$Tag.log)" }
}
# per-shard trial counts via Python (Windows PowerShell's ConvertFrom-Json stops at about 2 MB; shards files reach 3 MB)
$counts = (& $Vpy -c "import json,sys; d=json.load(open(sys.argv[1], encoding='utf-8')); print(' '.join(str(len(s['trials'])) for s in d['shards']))" $shards | Out-String).Trim()
$want = @($counts -split '\s+' | Where-Object { $_ } | ForEach-Object { [int]$_ })
$n = $want.Count
if ($n -lt 1) { Stop-With 'could not read the shards file' }
$total = ($want | Measure-Object -Sum).Sum
Say "shards $n, trials $total, out $outDir"
Send-Note "main started: $n workers, $total trials, disk $freeGb GB"

function Get-ShardHave([int]$I) {
    $folder = Join-Path $outDir "shard$I\trials"
    if (-not (Test-Path $folder)) { return 0 }
    return @(Get-ChildItem $folder -Filter '*.json' | Where-Object { $_.Name -notlike '*.navigation.json' }).Count
}
function Get-Done {
    $d = 0
    for ($i = 0; $i -lt $n; $i++) { $d += Get-ShardHave $i }
    return $d
}
function Test-Shard([int]$I) {
    $procs = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*run-shard*$($Pc)_$Tag.shards.json*--index $I *" })
    return ($procs.Count -gt 0)
}
function Start-Shard([int]$I) {
    $log = Join-Path $Work "shard_$($Tag)_$I.log"
    $err = Join-Path $Work "shard_$($Tag)_$I.err"
    Start-Process $Vpy -WindowStyle Hidden -RedirectStandardOutput $log -RedirectStandardError $err -ArgumentList '-m', 'control.main_distributed', 'run-shard', '--shards', $shardsRel, '--index', "$I", '--output', "results/main8/$($Pc)_$Tag" | Out-Null
}

# ---------------- 5. run and watch ----------------
$shardRestarts = @{}
for ($i = 0; $i -lt $n; $i++) { $shardRestarts[$i] = 0 }
$lastTenth = -1
$lastDone = -1
$lastChange = Get-Date
$stallSent = $false
while ($true) {
    $running = 0
    for ($i = 0; $i -lt $n; $i++) {
        if ((Get-ShardHave $i) -ge $want[$i]) { continue }
        if (Test-Shard $i) { $running += 1; continue }
        if ($shardRestarts[$i] -ge 5) { Send-Note "shard $i keeps stopping - see $Work\shard_$($Tag)_$i.err"; $shardRestarts[$i] = -100; continue }
        if ($shardRestarts[$i] -lt 0) { continue }
        if ($shardRestarts[$i] -gt 0) { Say "restarting shard $i (attempt $($shardRestarts[$i]))" }
        $shardRestarts[$i] += 1
        Start-Shard $i
        $running += 1
    }
    $done = Get-Done
    if ($done -ne $lastDone) { $lastDone = $done; $lastChange = Get-Date; $stallSent = $false }
    $tenth = [math]::Floor(10 * $done / [math]::Max(1, $total))
    if ($tenth -gt $lastTenth) {
        if ($lastTenth -ge 0) { Send-Note "main $done/$total trials" }
        $lastTenth = $tenth
    }
    if ($done -ge $total) { break }
    if ($running -eq 0) { Stop-With "no shard is running and $($total - $done) trials are left - see $Work" }
    if (((New-TimeSpan -Start $lastChange -End (Get-Date)).TotalMinutes -gt 180) -and -not $stallSent) {
        Send-Note "no new main trial for 3 h - check this PC"; $stallSent = $true
    }
    Start-Sleep -Seconds $PollSeconds
}
$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
Say "main finished: $done/$total trials in $outDir (disk free $freeGb GB)"
Send-Note "MAIN DONE $done/$total trials, results in fds8\results\main8\$($Pc)_$Tag"
exit 0
