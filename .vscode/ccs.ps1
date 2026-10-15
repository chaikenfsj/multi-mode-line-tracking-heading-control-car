param(
    [ValidateSet('Build', 'Flash', 'Check', 'Open')]
    [string]$Action = 'Build'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$ccsRoot = 'C:\ti\ccs2050\ccs'
$cli = Join-Path $ccsRoot 'eclipse\ccs-server-cli.bat'
$dslite = Join-Path $ccsRoot 'ccs_base\DebugServer\bin\DSLite.exe'
$targetConfig = Join-Path $projectRoot 'targetConfigs\MSPM0G3507.ccxml'

function Build-Project {
    [xml]$project = Get-Content -Raw -LiteralPath (Join-Path $projectRoot '.project')
    $projectName = $project.projectDescription.name
    & $cli '-noSplash' '-workspace' $projectRoot '-application' 'com.ti.ccs.apps.buildProject' '-ccs.projects' $projectName '-ccs.configuration' 'Debug'
    if ($LASTEXITCODE -ne 0) { throw "CCS build failed (exit $LASTEXITCODE). Flash cancelled." }
    $script:program = Join-Path $projectRoot "Debug\$projectName.out"
    if (!(Test-Path -LiteralPath $script:program)) { throw "Build output is missing: $script:program" }
    Write-Host "Build OK: $script:program"
}

function Check-Probe {
    [xml]$config = Get-Content -Raw -LiteralPath $targetConfig
    if ($config.configurations.configuration.connection.id -ne 'SEGGER J-Link Emulator') {
        throw 'This task expects the existing J-Link target configuration. Check the selected probe in CCS.'
    }
    $devices = @(Get-PnpDevice -PresentOnly -ErrorAction Stop | Where-Object {
        $_.InstanceId -match '^USB\\VID_1366&' -or $_.FriendlyName -match 'J-?Link|XDS110|CMSIS-DAP|DAPLink|ST-?Link'
    })
    $jlinks = @($devices | Where-Object { $_.InstanceId -match '^USB\\VID_1366&' -and $_.InstanceId -notmatch '&MI_' })
    $otherProbes = @($devices | Where-Object { $_.FriendlyName -match 'XDS110|CMSIS-DAP|DAPLink|ST-?Link' })
    if ($jlinks.Count -ne 1 -or $otherProbes.Count -gt 0) {
        throw 'Connect exactly one J-Link probe and power the MSPM0G3507 board. This project is configured for J-Link.'
    }
    Write-Host "Detected: $($jlinks[0].FriendlyName)"
}

try {
    if (!(Test-Path -LiteralPath $cli)) { throw "CCS installation not found: $ccsRoot" }
    switch ($Action) {
        'Build' { Build-Project }
        'Check' { Check-Probe }
        'Flash' {
            Check-Probe
            Build-Project
            & $dslite 'flash' '-c' $targetConfig '-f' '-v' '-r' '2' '-u' $script:program
            if ($LASTEXITCODE -ne 0) { throw "DSLite failed (exit $LASTEXITCODE)." }
            Write-Host 'Flash and verification completed; system reset requested.'
        }
        'Open' {
            Start-Process -FilePath (Join-Path $ccsRoot 'theia\ccstudio.exe') -ArgumentList ('"' + $projectRoot + '"') -WindowStyle Normal
        }
    }
    exit 0
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}
