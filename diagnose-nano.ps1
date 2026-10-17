$ErrorActionPreference = 'Stop'

function Find-AvrdudePackage {
    $platformIoHomes = @()

    if ($env:PLATFORMIO_CORE_DIR) {
        $platformIoHomes += $env:PLATFORMIO_CORE_DIR
    }
    if ($env:USERPROFILE) {
        $platformIoHomes += (Join-Path $env:USERPROFILE '.platformio')
    }
    if ($env:HOME) {
        $platformIoHomes += (Join-Path $env:HOME '.platformio')
    }

    $platformIoHomes = $platformIoHomes | Select-Object -Unique
    foreach ($platformIoHome in $platformIoHomes) {
        $packagesDir = Join-Path $platformIoHome 'packages'
        if (Test-Path -LiteralPath $packagesDir) {
            $candidate = Get-ChildItem -LiteralPath $packagesDir -Directory -Filter 'tool-avrdude*' |
                Sort-Object Name -Descending |
                ForEach-Object {
                    $exe = Get-ChildItem -LiteralPath $_.FullName -Filter 'avrdude.exe' -File -Recurse |
                        Select-Object -First 1
                    if ($exe) {
                        $conf = Get-ChildItem -LiteralPath $_.FullName -Filter 'avrdude.conf' -File -Recurse |
                            Select-Object -First 1
                        if ($conf) {
                            [PSCustomObject]@{
                                Executable = $exe.FullName
                                Config = $conf.FullName
                            }
                        }
                    }
                } | Select-Object -First 1

            if ($candidate) {
                return $candidate
            }
        }
    }

    throw "Tidak menemukan avrdude.exe dan avrdude.conf di paket PlatformIO. Pastikan PlatformIO telah memasang tool-avrdude."
}

function Get-SerialPortCandidates {
    $ports = @()

    if (Get-Command Get-CimInstance -ErrorAction SilentlyContinue) {
        $ports = @(Get-CimInstance -ClassName Win32_SerialPort -ErrorAction SilentlyContinue |
            Where-Object { $_.DeviceID -match '^COM\d+$' } |
            ForEach-Object {
                [PSCustomObject]@{
                    Port = $_.DeviceID
                    Description = $_.Name
                    PnpDeviceId = $_.PNPDeviceID
                }
            })
    }

    if ($ports.Count -eq 0) {
        $ports = @([System.IO.Ports.SerialPort]::GetPortNames() |
            ForEach-Object {
                [PSCustomObject]@{
                    Port = $_
                    Description = 'Serial port'
                    PnpDeviceId = ''
                }
            })
    }

    return @($ports | Sort-Object { [int]($_.Port -replace '\D', '') })
}

try {
    $avrdude = Find-AvrdudePackage
    Write-Host "avrdude : $($avrdude.Executable)"
    Write-Host "Config  : $($avrdude.Config)"

    $ports = @(Get-SerialPortCandidates)
    if ($ports.Count -eq 0) {
        throw 'Tidak ada port COM aktif yang terdeteksi. Sambungkan Nano dan periksa driver USB-serial.'
    }

    Write-Host "`nPort COM yang terdeteksi:"
    for ($i = 0; $i -lt $ports.Count; $i++) {
        $deviceHint = ''
        if ($ports[$i].PnpDeviceId -match 'VID_([0-9A-F]{4})&PID_([0-9A-F]{4})') {
            $deviceHint = " [VID_$($Matches[1])&PID_$($Matches[2])]"
        }
        Write-Host ("  [{0}] {1} - {2}{3}" -f ($i + 1), $ports[$i].Port, $ports[$i].Description, $deviceHint)
    }

    if ($ports.Count -eq 1) {
        $selectedPort = $ports[0].Port
        Write-Host "`nMenggunakan $selectedPort."
    }
    else {
        $choice = Read-Host "`nPilih nomor port COM Arduino Nano"
        $portIndex = 0
        if (-not [int]::TryParse($choice, [ref]$portIndex) -or $portIndex -lt 1 -or $portIndex -gt $ports.Count) {
            throw "Pilihan tidak valid: $choice"
        }
        $selectedPort = $ports[$portIndex - 1].Port
    }

    $tests = @(
        [PSCustomObject]@{ Chip = 'm328p'; Baud = 115200 }
        [PSCustomObject]@{ Chip = 'm328p'; Baud = 57600 }
        [PSCustomObject]@{ Chip = 'm328p'; Baud = 19200 }
        [PSCustomObject]@{ Chip = 'm168'; Baud = 115200 }
        [PSCustomObject]@{ Chip = 'm168'; Baud = 57600 }
        [PSCustomObject]@{ Chip = 'm168'; Baud = 19200 }
    )
    $results = @()

    foreach ($test in $tests) {
        Write-Host "`nUji chip $($test.Chip), baud $($test.Baud) pada $selectedPort"
        for ($seconds = 3; $seconds -ge 1; $seconds--) {
            Write-Host -NoNewline "`rMulai dalam $seconds detik... "
            Start-Sleep -Seconds 1
        }
        Write-Host "`rMenjalankan avrdude...                 "

        $previousErrorActionPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            $output = & $avrdude.Executable `
                '-C' $avrdude.Config `
                '-p' $test.Chip `
                '-c' 'arduino' `
                '-P' $selectedPort `
                '-b' "$($test.Baud)" `
                '-v' 2>&1
            $exitCode = $LASTEXITCODE
        }
        finally {
            $ErrorActionPreference = $previousErrorActionPreference
        }
        $outputText = ($output | Out-String)
        $signatureMatch = [regex]::Match($outputText, 'Device\s+signature\s*=\s*0x([0-9a-fA-F]+)')
        $signature = if ($signatureMatch.Success) { "0x$($signatureMatch.Groups[1].Value)" } else { $null }
        $responded = [bool]$signature

        if ($responded) {
            $chipHint = [regex]::Match($outputText, '\(probably\s+(m\d+[a-z0-9]*)\)')
            $detectedChip = if ($chipHint.Success) { $chipHint.Groups[1].Value } else { $test.Chip }
            Write-Host "SIGNATURE TERBACA: target $($test.Chip), chip terdeteksi $detectedChip @ $($test.Baud) baud, signature $signature" -ForegroundColor Green
            if ($exitCode -ne 0) {
                Write-Host "  avrdude mengembalikan exit code $exitCode; target yang diuji mungkin tidak cocok dengan chip." -ForegroundColor Yellow
            }
        }
        else {
            Write-Host "Tidak berhasil: $($test.Chip) @ $($test.Baud) baud (exit code $exitCode)." -ForegroundColor Yellow
            $diagnosticLines = $outputText -split "`r?`n" |
                Where-Object { $_ -match 'getsync|recv|signature|error|initialization|not in sync' } |
                Select-Object -Last 4
            foreach ($line in $diagnosticLines) {
                Write-Host "  $($line.Trim())"
            }
        }

        $results += [PSCustomObject]@{
            Chip = $test.Chip
            Baud = $test.Baud
            Success = $responded
            Signature = $signature
            DetectedChip = if ($responded) { $detectedChip } else { $null }
            ExitCode = $exitCode
        }
    }

    Write-Host "`n=== Ringkasan ==="
    $successfulTests = @($results | Where-Object Success)
    if ($successfulTests.Count -gt 0) {
        foreach ($result in $successfulTests) {
            Write-Host ("Terbaca: target {0}, chip {1} @ {2} baud, signature {3}" -f $result.Chip, $result.DetectedChip, $result.Baud, $result.Signature) -ForegroundColor Green
        }
    }
    else {
        Write-Host 'Tidak ada kombinasi yang berhasil membaca signature.' -ForegroundColor Red
        Write-Host 'Pastikan Serial Monitor tertutup, port benar, dan coba tekan-lepas RESET saat hitung mundur berakhir.'
    }
}
catch {
    Write-Error $_
    exit 1
}
