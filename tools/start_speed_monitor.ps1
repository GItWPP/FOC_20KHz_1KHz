param(
    [string]$Port='COM11',
    [int]$Baud=1500000,
    [int]$RefreshHz=60,
    [double]$StatsWindow=5,
    [double]$Seconds=0,
    [string]$Output='',
    [string]$Python='python',
    [switch]$NoPlot,
    [switch]$ReadOnly
)
$ErrorActionPreference='Stop'
$taskPython=(Get-Command $Python -ErrorAction Stop).Source
if(-not $Output){$Output=Join-Path $PSScriptRoot ('monitor_output\speed_'+(Get-Date -Format 'yyyyMMdd_HHmmss'))}
$taskImports='import serial'
if(-not $NoPlot){$taskImports+='; import numpy, pyqtgraph, PyQt5'}
& $taskPython -c $taskImports
if($LASTEXITCODE -ne 0){throw 'Missing Python packages. Run: python -m pip install -r tools/requirements.txt'}
$taskArgs=@('-u',(Join-Path $PSScriptRoot 'monitor_speed.py'),'--port',$Port,'--baud',$Baud,
    '--refresh-hz',$RefreshHz,'--stats-window',$StatsWindow.ToString([Globalization.CultureInfo]::InvariantCulture),
    '--output',$Output,'--seconds',$Seconds.ToString([Globalization.CultureInfo]::InvariantCulture))
if($NoPlot){$taskArgs+='--no-plot'}else{$taskArgs+='--plot'}
if($ReadOnly){$taskArgs+='--read-only'}
Write-Host "FOC console: $Port $Baud/8N1; output=$Output"
Write-Host 'Control GUI closes with STOP and drive-off readback. -NoPlot/-ReadOnly never sends commands.'
& $taskPython @taskArgs
if($LASTEXITCODE -ne 0){throw "FOC monitor exited with code $LASTEXITCODE"}
