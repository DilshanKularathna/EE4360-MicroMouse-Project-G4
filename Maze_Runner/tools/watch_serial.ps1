# Logs the Arduino serial output to a file so it can be read back.
# Close the PlatformIO/Arduino serial monitor first (only one program can use the port).
# Usage: powershell -File tools\watch_serial.ps1 -Seconds 60
param(
    [string]$Port = "COM7",
    [int]$Baud = 115200,
    [int]$Seconds = 60,
    [string]$Log = "$PSScriptRoot\serial_log.txt"
)

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.NewLine = "`n"
$sp.ReadTimeout = 500
$sp.Open()

$writer = New-Object System.IO.StreamWriter $Log, $true   # append, keep earlier runs
$writer.WriteLine("===== Session $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') =====")
$writer.Flush()
$end = (Get-Date).AddSeconds($Seconds)
Write-Host "Logging $Port for $Seconds s to $Log"

while ((Get-Date) -lt $end) {
    try {
        $line = $sp.ReadLine().TrimEnd("`r")
        $writer.WriteLine($line)
        $writer.Flush()
        Write-Host $line
    } catch [System.TimeoutException] {
    }
}

$writer.Close()
$sp.Close()
Write-Host "Done."
