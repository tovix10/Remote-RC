$ErrorActionPreference = 'Stop'

$portName = 'COM9'
$baudRate = 19200
$serialPort = $null

function Read-SerialResponse {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [int]$TimeoutMilliseconds = 1500
    )

    $response = [System.Collections.Generic.List[byte]]::new()
    $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMilliseconds)

    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $response.Add([byte]$Port.ReadByte())
        }
        catch [System.TimeoutException] {
            break
        }
    }

    return ,$response.ToArray()
}

function Test-Stk500Response {
    param([byte[]]$Response)

    for ($index = 0; $index -lt $Response.Length - 1; $index++) {
        if ($Response[$index] -eq 0x14 -and $Response[$index + 1] -eq 0x10) {
            return $true
        }
    }

    $text = [System.Text.Encoding]::ASCII.GetString($Response)
    return $text.Contains('AVR ISP')
}

try {
    $serialPort = [System.IO.Ports.SerialPort]::new(
        $portName,
        $baudRate,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One
    )
    $serialPort.ReadTimeout = 300
    $serialPort.WriteTimeout = 1000
    $serialPort.Open()

    Start-Sleep -Milliseconds 1500
    $serialPort.DiscardInBuffer()

    Write-Host "Menguji STK500 pada $portName ($baudRate baud)..."
    $serialPort.Write([byte[]]@(0x30, 0x20), 0, 2)
    $response = Read-SerialResponse -Port $serialPort
    $ready = Test-Stk500Response -Response $response

    if (-not $ready) {
        $serialPort.DiscardInBuffer()
        $serialPort.Write([byte[]]@(0x31, 0x20), 0, 2)
        $response = Read-SerialResponse -Port $serialPort
        $ready = Test-Stk500Response -Response $response
    }

    $responseHex = ($response | ForEach-Object { $_.ToString('X2') }) -join ' '
    if ($ready) {
        Write-Host "SIAP: Nano di $portName merespons protokol ArduinoISP/STK500 pada $baudRate baud." -ForegroundColor Green
        Write-Host "Respons: $responseHex"
        Write-Host 'Uji sinkronisasi berhasil; ini belum menguji pemrograman chip target secara fisik.'
        exit 0
    }

    Write-Host "BELUM SIAP: tidak ditemukan respons STK500 yang valid dari Nano di $portName." -ForegroundColor Red
    Write-Host "Respons: $(if ($responseHex) { $responseHex } else { '<kosong>' })"
    exit 1
}
catch {
    Write-Host "GAGAL: tidak dapat memverifikasi ArduinoISP pada $portName. $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
finally {
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
    }
    if ($serialPort) {
        $serialPort.Dispose()
    }
}
