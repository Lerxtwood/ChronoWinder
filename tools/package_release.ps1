param(
    [Parameter(Mandatory = $true)]
    [string]$Version,

    [string]$Environment = "esp32-c3-devkitm-1",
    [string]$RepoOwner = "Lerxtwood",
    [string]$RepoName = "ChronoWinder",
    [string]$OutputRoot = "release"
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$platformio = "platformio"
$localPlatformio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"
if (Test-Path $localPlatformio) {
    $platformio = $localPlatformio
}

Push-Location $repoRoot
try {
    & $platformio run -e $Environment
    if ($LASTEXITCODE -ne 0) {
        throw "PlatformIO build failed with exit code $LASTEXITCODE"
    }

    $buildFirmware = Join-Path ".pio\build" (Join-Path $Environment "firmware.bin")
    if (-not (Test-Path $buildFirmware)) {
        throw "Firmware binary was not found at $buildFirmware"
    }

    $outputDir = Join-Path $OutputRoot $Version
    New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

    $outputFirmware = Join-Path $outputDir "firmware.bin"
    Copy-Item -Force $buildFirmware $outputFirmware

    $firmwareFile = Get-Item $outputFirmware
    $hash = (Get-FileHash $outputFirmware -Algorithm SHA256).Hash.ToLowerInvariant()
    $commit = (git -c "safe.directory=$repoRoot" rev-parse --short HEAD).Trim()
    if ($LASTEXITCODE -ne 0) {
        throw "Unable to determine Git commit"
    }
    $tag = $Version

    $manifest = [ordered]@{
        version = $Version
        board = $Environment
        firmware = "https://github.com/$RepoOwner/$RepoName/releases/download/$tag/firmware.bin"
        sha256 = $hash
        size = $firmwareFile.Length
        commit = $commit
        builtAtUtc = (Get-Date).ToUniversalTime().ToString("yyyy-MM-ddTHH:mm:ssZ")
    }

    $manifestPath = Join-Path $outputDir "manifest.json"
    $manifest | ConvertTo-Json | Set-Content -Encoding UTF8 $manifestPath

    Write-Host "Release package created:"
    Write-Host "  Firmware: $outputFirmware"
    Write-Host "  Manifest: $manifestPath"
    Write-Host "  SHA-256:  $hash"
    Write-Host "  Size:     $($firmwareFile.Length) bytes"
}
finally {
    Pop-Location
}
