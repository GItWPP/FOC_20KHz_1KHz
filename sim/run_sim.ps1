param(
    [string]$VivadoBin='C:\Xilinx\Vivado\2024.2\bin',
    [string]$OutputDir='',
    [string[]]$TopNames=@('tb_foc_uart_rx','tb_foc_uart_control','tb_foc_control_uart','tb_foc_uart_top')
)
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
if(-not $OutputDir){$OutputDir=Join-Path $env:TEMP ('foc_sim_'+(Get-Date -Format 'yyyyMMdd_HHmmss'))}
New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
Push-Location -LiteralPath $OutputDir
try {
    $files=@(Get-ChildItem -LiteralPath (Join-Path $taskRoot 'rtl') -Filter '*.v' | ForEach-Object FullName)
    $files+=@(Get-ChildItem -LiteralPath $PSScriptRoot -Filter 'tb_*.v' | ForEach-Object FullName)
    & (Join-Path $VivadoBin 'xvlog.bat') @files *> 'compile.log'
    if($LASTEXITCODE -ne 0){throw "Verilog compilation failed: $OutputDir\compile.log"}
    @('run all','quit') | Set-Content -LiteralPath 'run_all.tcl' -Encoding ascii
    foreach($tb in $TopNames){
        & (Join-Path $VivadoBin 'xelab.bat') $tb -s $tb -debug typical -log "$tb-elaborate.log" *> "$tb-elaborate-console.log"
        if($LASTEXITCODE -ne 0){throw "Elaboration failed: $tb"}
        & (Join-Path $VivadoBin 'xsim.bat') $tb -tclbatch 'run_all.tcl' -log "$tb-simulate.log" *> "$tb-console.log"
        $simExit=$LASTEXITCODE
        $simLog=Get-Content -LiteralPath "$tb-simulate.log" -Raw
        if($simExit -ne 0 -or $simLog -match 'FAIL|Fatal|ERROR:' -or $simLog -notmatch 'PASS:'){throw "Simulation failed: $tb; see $OutputDir"}
        ($simLog -split "`r?`n" | Where-Object {$_ -match '^PASS:'}) | Write-Output
    }
    Write-Host "Simulation results: $OutputDir"
} finally {Pop-Location}
