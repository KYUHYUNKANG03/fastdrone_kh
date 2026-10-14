# main8_chain.ps1 - one command per school PC: run this PC's part of the tune8 main experiment.
# Adapted from main7_chain.ps1. ASCII only (Windows PowerShell reads a BOM-less file as ANSI).
#
# A. One PC per controller (the PC that tuned it):
#   irm "https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school/main8_chain.ps1" -OutFile $HOME\main8_chain.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\main8_chain.ps1 -Controllers F13 -Topic <ntfy topic>
#   PC-62: -Controllers F13   PC-63: -Controllers V13,CPID   PC-64: -Controllers GSLQR
#
# B. One controller split over several PCs (-Shards = total pieces, -Index = the pieces this PC runs):
#   tuning PC : ... -Controllers M17 -Shards 12 -Index 0,1 -Topic <ntfy topic>
#   other PCs : ... -Controllers M17 -Shards 12 -Index 2,3 -Topic <ntfy topic>    (4,5 / 6,7 / 8,9 / 10,11)
#   Every PC makes the same spec and the same shards file (both are deterministic), runs only its indices,
#   and writes to its own folder; the folders are merged afterwards (control.main_distributed merge).
#   A PC that did not tune the controller needs <C>.record.json (and <C>.gate.json if available) from the
#   tuning PC. It looks for them on its Desktop, in Downloads, at the top of every drive (USB), and in
#   records/ of the tune8 branch on GitHub, every 5 minutes, and starts by itself once it finds them.
#   On the tuning PC the chain sends both files to the phone (ntfy attachment) when the gate passes.
#
# The background process:
#   0. makes sure fds8 exists and passed its checks (runs tune8_all.ps1 without a controller; 15 min on a new PC)
#   1. waits until each controller's tune8 record is complete at 60 (it never starts tuning)
#   2. waits for (or runs) the pre-main gate; a controller whose gate FAILED stops the chain - no main run for it
#   3. copies the final records to results\arena\tuning\tune8_final\ and writes the main spec
#      (school/main8_make_spec.py: seeds 1020-1039, ladder 1020-1029, ladder only with V13)
#   4. makes shards and runs this PC's shards in parallel (control.main_distributed run-shard), restarting any that die
#   5. keeps Windows from sleeping while it works (screen may turn off) and pushes progress to the phone
# Re-running the same line after a reboot/logoff resumes everything (finished trials are kept).
# Uses $HOME\fds8 only. Never edits control/, models/ or configs/ and never moves the git checkout.
param(
    [Parameter(Mandatory = $true)][string[]]$Controllers,
    [string]$Topic = '',
    [switch]$Run,
    [int]$Workers = 0,
    [int]$Shards = 0,
    [string]$Index = '',
    [int]$PollSeconds = 300
)

$ErrorActionPreference = 'Continue'
$ExpectedHead = 'f5dfa079adbaccce13d6009b50ae296f5659082b'
$Budget = 60
$Raw = 'https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/school'
$RawRecords = 'https://raw.githubusercontent.com/KYUHYUNKANG03/fastdrone_kh/tune8/records'
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

$Split = ($Shards -gt 0)
$RunIdx = @()
if ($Split) {
    $RunIdx = @("$Index" -split '[,\s]+' | Where-Object { $_ } | ForEach-Object { [int]$_ } | Sort-Object -Unique)
    if ($RunIdx.Count -eq 0) { Write-Host 'FAIL: -Shards needs -Index (for example -Index 0,1)'; exit 1 }
    foreach ($i in $RunIdx) { if ($i -lt 0 -or $i -ge $Shards) { Write-Host "FAIL: index $i is outside 0..$($Shards - 1)"; exit 1 } }
    $FileTag = "$($Tag)_split$Shards"
    $LogTag = "$($FileTag)_idx$($RunIdx -join '-')"
} elseif ("$Index") {
    Write-Host 'FAIL: -Index needs -Shards'; exit 1
} else {
    $FileTag = "$($Pc)_$Tag"
    $LogTag = $Tag
}

function Stamp { Get-Date -Format 'MM-dd HH:mm' }
function Say([string]$Text) {
    $line = "[{0}] {1}" -f (Get-Date -Format 'MM-dd HH:mm:ss'), $Text
    Write-Host $line
    if (Test-Path $Work) { Add-Content -Path (Join-Path $Work "chain_$LogTag.log") -Value $line }
}
function Send-Note([string]$Message) {
    if ($Topic) {
        $label = if ($Split) { "$Tag [$($RunIdx -join ',') of $Shards]" } else { $Tag }
        try { Invoke-RestMethod -Method Post -Uri "https://ntfy.sh/$Topic" -Body "$Pc main8 $label`: $Message $(Stamp)" -TimeoutSec 20 | Out-Null }
        catch { Say "ntfy failed: $($_.Exception.Message)" }
    }
}
function Send-File([string]$Path) {
    if ($Topic -and (Test-Path $Path)) {
        try {
            Invoke-RestMethod -Method Put -Uri "https://ntfy.sh/$Topic" -InFile $Path -Headers @{ Filename = (Split-Path $Path -Leaf); Title = "$Pc main8 file" } -TimeoutSec 60 | Out-Null
        } catch { Say "ntfy file upload failed: $($_.Exception.Message)" }
    }
}
function Stop-With([string]$Text) { Say "FAIL: $Text"; Send-Note "FAIL $Text"; exit 1 }

New-Item -ItemType Directory -Force -Path $Work | Out-Null

# ---------------- launcher: start the hidden background run and return ----------------
if (-not $Run) {
    $others = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*main8_chain.ps1*-Run*' -and $_.CommandLine -like "*-Controllers $($Controllers -join ',') *" })
    if ($others.Count -gt 0) { Say "main8 chain for $Tag is already running (pid $($others[0].ProcessId)) - nothing to do"; exit 0 }
    $argList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath, '-Run', '-Controllers', ($Controllers -join ','), '-PollSeconds', "$PollSeconds", '-Workers', "$Workers", '-Shards', "$Shards")
    if ($Split) { $argList += @('-Index', ($RunIdx -join ',')) }
    if ($Topic) { $argList += @('-Topic', $Topic) }
    Start-Process powershell -WindowStyle Hidden -ArgumentList $argList
    Say "main8 chain started in the background for $Tag on $Pc. Log: $Work\chain_$LogTag.log. You can close this window."
    exit 0
}

# ---------------- background run ----------------
# keep the system awake while this process lives (screen may still turn off; logoff/reboot still stop it)
try {
    Add-Type -Namespace Main8 -Name Power -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint flags);'
    [Main8.Power]::SetThreadExecutionState([uint32]2147483649) | Out-Null
} catch { Say "could not set keep-awake: $($_.Exception.Message)" }

# ---------------- 0. fds8 set up and checked (quick when already done) ----------------
$TuneAll = Join-Path $HOME 'tune8_all.ps1'
try { Invoke-WebRequest -Uri "$Raw/tune8_all.ps1" -OutFile $TuneAll -UseBasicParsing } catch { }
if (-not (Test-Path $TuneAll)) { Stop-With 'could not download tune8_all.ps1' }
Say 'checking fds8 (setup and checks; about 15 min on a PC that has not run them yet)'
$setupArgs = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $TuneAll)
if ($Topic) { $setupArgs += @('-Topic', $Topic) }
& powershell @setupArgs *> (Join-Path $Work "setup_$LogTag.log")
if ($LASTEXITCODE -ne 0) { Stop-With "fds8 setup/checks failed - see $Work\setup_$LogTag.log" }
if (-not (Test-Path $Vpy)) { Stop-With 'fds8\.venv not found after setup' }
# a PC without Git for Windows uses the portable MinGit that tune8_all.ps1 unpacked; the shard workers
# record the git revision, and merge refuses rows without it
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

$Helper = Join-Path $Work 'main8_make_spec.py'
$Gate = Join-Path $Work 'tune8_gate.py'
try {
    Invoke-WebRequest -Uri "$Raw/main8_make_spec.py" -OutFile $Helper -UseBasicParsing
    Invoke-WebRequest -Uri "$Raw/tune8_gate.py" -OutFile $Gate -UseBasicParsing
} catch { if (-not (Test-Path $Helper) -or -not (Test-Path $Gate)) { Stop-With "download failed: $($_.Exception.Message)" } }

$TuneDir = Join-Path $Fds 'results\arena\tuning\tune8'
$FinalDir = Join-Path $Fds 'results\arena\tuning\tune8_final'
$MainDir = Join-Path $Fds 'results\main8'
New-Item -ItemType Directory -Force -Path $TuneDir, $FinalDir, $MainDir | Out-Null
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
# Split mode on a PC that did not tune C: bring in <C>.record.json (+ <C>.gate.json) from Desktop, Downloads,
# a drive root (USB) or GitHub. The files are checked later (complete at 60, tune8 config, gate on this record).
function Import-Shared([string]$C) {
    $rec = "$C.record.json"; $gat = "$C.gate.json"
    $dirs = @((Join-Path $HOME 'Desktop'), (Join-Path $HOME 'Downloads'))
    foreach ($d in @(Get-PSDrive -PSProvider FileSystem -ErrorAction SilentlyContinue)) {
        if ($d.Root) { $dirs += $d.Root; $dirs += (Join-Path $d.Root 'tune8') }
    }
    foreach ($d in $dirs) {
        $src = Join-Path $d $rec
        if (Test-Path $src) {
            Copy-Item $src (Join-Path $TuneDir $rec) -Force
            if (Test-Path (Join-Path $d $gat)) { Copy-Item (Join-Path $d $gat) (Join-Path $TuneDir $gat) -Force }
            Say "imported $rec from $d"
            return $true
        }
    }
    $tmp = Join-Path $Work $rec
    try {
        Invoke-WebRequest -Uri "$RawRecords/$rec" -OutFile $tmp -UseBasicParsing -ErrorAction Stop
        Copy-Item $tmp (Join-Path $TuneDir $rec) -Force
        try {
            Invoke-WebRequest -Uri "$RawRecords/$gat" -OutFile (Join-Path $Work $gat) -UseBasicParsing -ErrorAction Stop
            Copy-Item (Join-Path $Work $gat) (Join-Path $TuneDir $gat) -Force
        } catch { }
        Say "imported $rec from GitHub"
        return $true
    } catch { return $false }
}

# ---------------- 1. wait for tuning to finish at 60 ----------------
Send-Note "chain started (waiting for tune8 at $Budget, gate, then main)"
$waitSent = @{}
while ($true) {
    $pending = @($Controllers | Where-Object { -not (Test-Final $_) })
    if ($pending.Count -eq 0) { break }
    foreach ($c in $pending) {
        if (Test-Tuning $c) {
            if (-not $waitSent[$c]) {
                $r = Get-Record $c
                $spent = if ($null -ne $r) { [int]$r.spent } else { 0 }
                Say "waiting for $c tuning ($spent/$Budget)"
                $waitSent[$c] = $true
            }
        } elseif ($Split) {
            if (-not (Import-Shared $c) -and -not $waitSent[$c]) {
                Say "waiting for $c.record.json (Desktop, Downloads, USB or GitHub records/)"
                Send-Note "waiting for $c.record.json and $c.gate.json (put them on the Desktop or a USB stick)"
                $waitSent[$c] = $true
            }
        } else {
            Stop-With "$c tune8 tuning is not complete and not running - run the tune8_all.ps1 line for $c first"
        }
    }
    if (@($Controllers | Where-Object { -not (Test-Final $_) }).Count -eq 0) { break }
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
    if ($Split -and (Test-Path (Join-Path $TuneDir "$c.jsonl"))) {
        # only the tuning PC has the .jsonl; the other PCs of this split need these two files - send them to the phone
        Send-File (Join-Path $TuneDir "$c.record.json")
        Send-File $gateJson
    }
}
Send-Note "gate PASS for $($Controllers -join ', '), preparing main"

# ---------------- 3. final records + spec ----------------
foreach ($c in $Controllers) {
    foreach ($ext in @('record.json', 'jsonl')) {
        $src = Join-Path $TuneDir "$c.$ext"
        $dst = Join-Path $FinalDir "$c.$ext"
        if (-not (Test-Path $src)) { continue }              # a split helper has the record but not the jsonl
        if (-not (Test-Path $dst)) { Copy-Item $src $dst }
        elseif ((Get-FileHash $src).Hash -ne (Get-FileHash $dst).Hash) { Stop-With "$dst differs from the tuning output - do not edit tune8_final by hand" }
    }
}
$specRel = "results/main8/$FileTag.spec.json"
$shardsRel = "results/main8/$FileTag.shards.json"
$outRel = if ($Split) { "results/main8/$($Pc)_$FileTag" } else { "results/main8/$FileTag" }
$outDir = Join-Path $Fds ($outRel -replace '/', '\')
$spec = Join-Path $Fds $specRel
$shardsFile = Join-Path $Fds $shardsRel
if (-not (Test-Path $spec)) {
    & $Vpy $Helper --root $Fds --controllers @Controllers --out $spec *> (Join-Path $Work "spec_$LogTag.log")
    if ($LASTEXITCODE -ne 0) { Stop-With "main spec could not be made (see $Work\spec_$LogTag.log)" }
}
$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
Say "disk free $freeGb GB"
if ($freeGb -lt 5) { Stop-With "only $freeGb GB free on C: - main results need several GB" }
if ($freeGb -lt 20) { Send-Note "WARNING disk free only $freeGb GB (main may need ~20 GB)" }

# ---------------- 4. shards ----------------
if ($Split) {
    $nShards = $Shards
} elseif ($Workers -gt 0) {
    $nShards = $Workers
} else {
    $perWorker = ($Controllers | ForEach-Object { $Gib[$_] } | Measure-Object -Maximum).Maximum
    $text = (& $Vpy -m control.arena_suggest_workers $perWorker --physical-cores $cores 2>&1 | Out-String)
    $nShards = 1
    if ($text -match 'scenario-workers\s+(\d+)') { $nShards = [int]$Matches[1] }
    # PC-61 measurement: an M17 process holds about 3.7 GiB, not 2.0 - never more than 2 on a 16 GiB PC
    if ($Controllers -contains 'M17') { $nShards = [math]::Min($nShards, 2) }
}
$nShards = [math]::Max(1, $nShards)
if (-not (Test-Path $shardsFile)) {
    & $Vpy -m control.main_distributed make-shards --spec $specRel --n $nShards --out $shardsRel *> (Join-Path $Work "shards_$LogTag.log")
    if ($LASTEXITCODE -ne 0) { Stop-With "make-shards refused (see $Work\shards_$LogTag.log)" }
}
# per-shard trial counts via Python (Windows PowerShell's ConvertFrom-Json stops at about 2 MB; shards files reach 3 MB)
$counts = (& $Vpy -c "import json,sys; d=json.load(open(sys.argv[1], encoding='utf-8')); print(' '.join(str(len(s['trials'])) for s in d['shards']))" $shardsFile | Out-String).Trim()
$want = @($counts -split '\s+' | Where-Object { $_ } | ForEach-Object { [int]$_ })
$n = $want.Count
if ($n -lt 1) { Stop-With 'could not read the shards file' }
if ($Split -and ($n -ne $Shards)) { Stop-With "shards file has $n pieces, not $Shards - delete $shardsFile only if no trial has run yet" }
if (-not $Split) { $RunIdx = @(0..($n - 1)) }
$total = 0
foreach ($i in $RunIdx) { $total += $want[$i] }
Say "shards $n, running $($RunIdx -join ',') here, trials $total, out $outDir"
Send-Note "main started: $($RunIdx.Count) workers, $total trials, disk $freeGb GB"

function Get-ShardHave([int]$I) {
    $folder = Join-Path $outDir "shard$I\trials"
    if (-not (Test-Path $folder)) { return 0 }
    return @(Get-ChildItem $folder -Filter '*.json' | Where-Object { $_.Name -notlike '*.navigation.json' }).Count
}
function Get-Done {
    $d = 0
    foreach ($i in $RunIdx) { $d += Get-ShardHave $i }
    return $d
}
function Test-Shard([int]$I) {
    $procs = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*run-shard*$FileTag.shards.json*--index $I *" })
    return ($procs.Count -gt 0)
}
function Start-Shard([int]$I) {
    $log = Join-Path $Work "shard_$($FileTag)_$I.log"
    $err = Join-Path $Work "shard_$($FileTag)_$I.err"
    Start-Process $Vpy -WindowStyle Hidden -RedirectStandardOutput $log -RedirectStandardError $err -ArgumentList '-m', 'control.main_distributed', 'run-shard', '--shards', $shardsRel, '--index', "$I", '--output', $outRel | Out-Null
}

# ---------------- 5. run and watch ----------------
$shardRestarts = @{}
foreach ($i in $RunIdx) { $shardRestarts[$i] = 0 }
$lastTenth = -1
$lastDone = -1
$lastChange = Get-Date
$stallSent = $false
while ($true) {
    $running = 0
    foreach ($i in $RunIdx) {
        if ((Get-ShardHave $i) -ge $want[$i]) { continue }
        if (Test-Shard $i) { $running += 1; continue }
        if ($shardRestarts[$i] -ge 5) { Send-Note "shard $i keeps stopping - see $Work\shard_$($FileTag)_$i.err"; $shardRestarts[$i] = -100; continue }
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
Send-Note "MAIN DONE $done/$total trials, results in fds8\$($outRel -replace '/', '\')"
exit 0
