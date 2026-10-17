param(
    [ValidateSet('Flash', 'Isp', 'Loopback')]
    [string]$Phase = 'Flash',
    [switch]$ClockJumperConnected,
    [switch]$LoopbackWired,
    [switch]$ManualReset
)

$ErrorActionPreference = 'Stop'

$project = $PSScriptRoot
$environment = 'nano_programmer'
$portName = 'COM9'
$baudRate = 19200
$invalidSignatures = @('000000', 'ffffff')

function Invoke-ExternalCommand {
    param(
        [string]$FilePath,
        [string[]]$Arguments,
        [string]$Description
    )

    Write-Host "`n=== $Description ===" -ForegroundColor Cyan
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & $FilePath @Arguments
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }

    if ($exitCode -ne 0) {
        throw "$Description gagal dengan exit code $exitCode."
    }
}

function Get-PlatformIoExecutable {
    $command = Get-Command pio -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    $candidate = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe'
    if (Test-Path -LiteralPath $candidate -PathType Leaf) {
        return $candidate
    }

    throw 'PlatformIO CLI (pio.exe) tidak ditemukan.'
}

function Get-AvrdudeFiles {
    $packageRoot = Join-Path $env:USERPROFILE '.platformio\packages\tool-avrdude'
    $executable = Get-ChildItem -LiteralPath $packageRoot -Recurse -File -Filter 'avrdude.exe' -ErrorAction SilentlyContinue |
        Select-Object -First 1
    $configuration = Get-ChildItem -LiteralPath $packageRoot -Recurse -File -Filter 'avrdude.conf' -ErrorAction SilentlyContinue |
        Select-Object -First 1

    if (-not $executable -or -not $configuration) {
        throw 'avrdude.exe atau avrdude.conf tidak ditemukan pada paket PlatformIO.'
    }

    return @{
        Executable = $executable.FullName
        Configuration = $configuration.FullName
    }
}

function Test-IspSignatures {
    if (-not $ClockJumperConnected) {
        throw "Sebelum uji ISP, hubungkan D9 programmer ke kaki XTAL1 ATmega328P target, pastikan GND bersama, lalu jalankan ulang dengan -Phase Isp -ClockJumperConnected."
    }
    if (-not ([System.IO.Ports.SerialPort]::GetPortNames() -contains $portName)) {
        throw "$portName tidak terdeteksi."
    }

    $avrdude = Get-AvrdudeFiles
    $results = @()
    foreach ($bitClock in @(1, 10, 50, 250)) {
        $arguments = @(
            '-C', $avrdude.Configuration,
            '-p', 'm328p',
            '-c', 'stk500v1',
            '-P', $portName,
            '-b', "$baudRate",
            '-B', "$bitClock",
            '-v',
            '-n'
        )

        $previousPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            $output = & $avrdude.Executable @arguments 2>&1
            $exitCode = $LASTEXITCODE
        }
        finally {
            $ErrorActionPreference = $previousPreference
        }

        $outputText = ($output | ForEach-Object { "$_" }) -join "`n"
        $signatureMatch = [regex]::Match($outputText, 'Device\s+signature\s*=\s*0x([0-9a-fA-F]{6})')
        $signature = if ($signatureMatch.Success) { $signatureMatch.Groups[1].Value.ToLowerInvariant() } else { $null }
        $validSignature = $signature -and ($signature -notin $invalidSignatures)
        $clockWarning = $outputText -match 'does not support adjustable bitclock speed; ignoring -B'

        $results += [PSCustomObject]@{
            BitClock = $bitClock
            Signature = $signature
            ValidSignature = [bool]$validSignature
            ExitCode = $exitCode
            BitClockIgnored = [bool]$clockWarning
            Output = $outputText
        }

        $displaySignature = if ($signature) { "0x$signature" } else { '<tidak terbaca>' }
        $status = if ($validSignature) { 'SIGNATURE TERBACA' } else { 'TIDAK ADA SIGNATURE VALID' }
        Write-Host "`n-B $bitClock us: $status; signature $displaySignature; avrdude exit $exitCode"
        if ($clockWarning) {
            Write-Host '  Peringatan avrdude: -B diabaikan oleh programmer stk500v1 ArduinoISP ini.' -ForegroundColor Yellow
        }
        $diagnostic = $outputText -split "`r?`n" |
            Where-Object { $_ -match 'Device signature|does not support adjustable bitclock|not in sync|getsync|initialization failed|AVR device initialized' } |
            Select-Object -Last 5
        foreach ($line in $diagnostic) {
            Write-Host "  $($line.Trim())"
        }
    }

    $successful = @($results | Where-Object ValidSignature)
    if ($successful.Count -gt 0) {
        $first = $successful[0]
        Write-Host "`nVONIS: target merespons ISP dengan signature 0x$($first.Signature)." -ForegroundColor Green
        if ($first.Signature -eq '1e950f') {
            Write-Host 'Chip terdeteksi sebagai ATmega328P. Parameter baca/burn yang sesuai:'
            Write-Host "  avrdude -p m328p -c stk500v1 -P $portName -b $baudRate -B $($first.BitClock)"
            Write-Host 'Pilih berkas bootloader dan nilai fuse sesuai varian board; skrip ini tidak mengubah fuse atau flash target.'
        } else {
            Write-Host 'Signature bukan ATmega328P; cocokkan part number dengan signature sebelum menulis fuse atau flash.'
        }
    } else {
        Write-Host "`nVONIS: target tidak memberikan signature ISP yang valid dalam pengujian ini." -ForegroundColor Red
        Write-Host 'Hasil tersebut TIDAK membuktikan silicon mati: fuse clock, XTAL1/jumper, RESET tepat di kaki MCU, suplai, jalur MOSI/MISO/SCK, atau proteksi masih dapat menyebabkan gejala sama.'
        if (@($results | Where-Object BitClockIgnored).Count -gt 0) {
            Write-Host 'Keempat percobaan bukan empat kecepatan SPI berbeda: avrdude 6.3 memperingatkan bahwa -B diabaikan untuk programmer stk500v1/ArduinoISP.'
            Write-Host 'Sketch ArduinoISP menggunakan SPI_CLOCK tetap; karena itu kegagalan sesudah uji ini belum cukup untuk menyatakan chip rusak total.'
        }
    }

    return $results
}

function Test-SpiLoopback {
    if (-not $LoopbackWired) {
        throw 'Matikan daya, lepaskan jalur D11/MOSI dan D12/MISO dari target, lalu jumper D11 ke D12 pada programmer. Setelah wiring aman, jalankan ulang dengan -Phase Loopback -LoopbackWired.'
    }
    if (-not ([System.IO.Ports.SerialPort]::GetPortNames() -contains $portName)) {
        throw "$portName tidak terdeteksi."
    }

    $serialPort = [System.IO.Ports.SerialPort]::new(
        $portName,
        $baudRate,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One
    )
    $serialPort.ReadTimeout = 300
    $serialPort.WriteTimeout = 1000

    try {
        $serialPort.Open()
        Start-Sleep -Milliseconds 1500
        $serialPort.DiscardInBuffer()
        $serialPort.Write([byte[]]@(0x7F, 0xA5, 0x20), 0, 3)

        $response = [System.Collections.Generic.List[byte]]::new()
        $deadline = [DateTime]::UtcNow.AddSeconds(2)
        while ([DateTime]::UtcNow -lt $deadline -and $response.Count -lt 3) {
            try {
                $response.Add([byte]$serialPort.ReadByte())
            }
            catch [System.TimeoutException] {
                break
            }
        }

        $bytes = $response.ToArray()
        $hex = ($bytes | ForEach-Object { $_.ToString('X2') }) -join ' '
        if ($bytes.Length -eq 3 -and $bytes[0] -eq 0x14 -and $bytes[1] -eq 0xA5 -and $bytes[2] -eq 0x10) {
            Write-Host "LOOPBACK LULUS: respons MOSI->MISO sesuai pola A5 (14 A5 10)." -ForegroundColor Green
            return $true
        }

        Write-Host "LOOPBACK GAGAL: respons aktual '$hex', diharapkan '14 A5 10'." -ForegroundColor Red
        return $false
    }
    finally {
        if ($serialPort.IsOpen) {
            $serialPort.Close()
        }
        $serialPort.Dispose()
    }
}

try {
    Set-Location -LiteralPath $project
    $pio = Get-PlatformIoExecutable
    $source = Join-Path $project 'src\main.cpp'
    $sourceText = [System.IO.File]::ReadAllText($source)
    if ($sourceText -notmatch 'TCCR1A\s*=\s*_BV\(COM1A0\)' -or
        $sourceText -notmatch 'TCCR1B\s*=\s*0x09' -or
        $sourceText -notmatch 'OCR1A\s*=\s*0' -or
        $sourceText -notmatch 'case\s+0x7F') {
        throw 'src\main.cpp belum memuat clock D9 dan perintah loopback diagnostik yang diharapkan.'
    }

    switch ($Phase) {
        'Flash' {
            Invoke-ExternalCommand -FilePath $pio -Arguments @('run', '-e', $environment) -Description 'Build ArduinoISP dengan clock XTAL1'
            if ($ManualReset) {
                Write-Host "`nTekan dan tahan RESET Nano programmer saat hitung mundur mencapai 2; lepaskan saat hitung mundur selesai agar bootloader menerima upload." -ForegroundColor Yellow
                for ($seconds = 45; $seconds -ge 1; $seconds--) {
                    Write-Host -NoNewline "`rUpload dimulai dalam $seconds detik... "
                    Start-Sleep -Seconds 1
                }
                Write-Host "`rMemulai upload...                              "
            }
            Invoke-ExternalCommand -FilePath $pio -Arguments @('run', '-e', $environment, '-t', 'upload') -Description 'Upload via bootloader UART ke COM9'
            Write-Host "`nUpload selesai. Sambungkan D9 programmer ke kaki XTAL1 chip target (bukan XTAL2); pastikan GND bersama dan tidak ada dua catu 5 V yang saling memberi daya." -ForegroundColor Yellow
            Write-Host "Setelah jumper terpasang, jalankan: powershell -ExecutionPolicy Bypass -File `"$PSCommandPath`" -Phase Isp -ClockJumperConnected"
        }
        'Isp' {
            Test-IspSignatures | Out-Null
        }
        'Loopback' {
            $passed = Test-SpiLoopback
            if (-not $passed) {
                exit 1
            }
        }
    }
}
catch {
    Write-Host "GAGAL: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
