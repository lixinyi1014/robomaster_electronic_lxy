# Flash firmware through the OpenOCD instance that Scope already runs,
# so CLion never opens the ST-Link itself (no conflict with Scope).
# Requires: Scope is open and Connected (OpenOCD Tcl port 6666 listening).
param(
    [string]$Elf = (Join-Path $PSScriptRoot "..\build\Debug\rm5.elf"),
    [string]$OcdHost = "127.0.0.1",
    [int]$Port = 6666
)
$ErrorActionPreference = "Stop"
if (-not (Test-Path $Elf)) { Write-Host "ELF not found: $Elf  (build the project first)"; exit 1 }
$elfPath = (Resolve-Path $Elf).Path -replace '\\', '/'
$age = (Get-Date) - (Get-Item $Elf).LastWriteTime
Write-Host "ELF : $elfPath  (built $([int]$age.TotalSeconds)s ago)"

try {
    $client = New-Object System.Net.Sockets.TcpClient
    $client.Connect($OcdHost, $Port)
} catch {
    Write-Host "Cannot reach OpenOCD at ${OcdHost}:$Port."
    Write-Host "Open Scope, select 'ST-Link wired' and click Connect first, then run this again."
    exit 1
}
$stream = $client.GetStream()
$stream.ReadTimeout = 60000
$cmd = "set __r [catch {program {$elfPath} verify reset} __e]; format {%d|%s} `$__r `$__e"
$bytes = [System.Text.Encoding]::ASCII.GetBytes($cmd) + [byte]0x1a
Write-Host "Flashing via Scope's OpenOCD ..."
$stream.Write($bytes, 0, $bytes.Length)

$buf = New-Object byte[] 4096
$ms = New-Object System.IO.MemoryStream
while ($true) {
    $n = $stream.Read($buf, 0, $buf.Length)
    if ($n -le 0) { break }
    $ms.Write($buf, 0, $n)
    if ($buf[$n - 1] -eq 0x1a) { break }
}
$client.Close()
$resp = [System.Text.Encoding]::UTF8.GetString($ms.ToArray()).TrimEnd([char]0x1a)
$code, $msg = $resp.Split('|', 2)
if ($code -eq "0") {
    Write-Host "OK: programmed, verified and reset."
    Write-Host "If you changed variables/structs, click Connect again in Scope to reload the ELF."
    exit 0
} else {
    Write-Host "FAILED: $msg $resp"
    Write-Host "Check the target power / SWD wiring and the OpenOCD log in Scope."
    exit 1
}
