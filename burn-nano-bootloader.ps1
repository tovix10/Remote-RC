param()

$ErrorActionPreference = 'Stop'

$Port = 'COM9'
$ProgrammerBaud = 19200
$Part = 'm328p'

function Get-PlatformIoHomes {
    $homes = @()

    if ($env:PLATFORMIO_CORE_DIR) {
        $homes += $env:PLATFORMIO_CORE_DIR
    }
    if ($env:USERPROFILE) {
        $homes += (Join-Path $env:USERPROFILE '.platformio')
    }
    if ($HOME) {
        $homes += (Join-Path $HOME '.platformio')
    }

    return @($homes | Where-Object { $_ } | Select-Object -Unique)
}

function Find-PackageFile {
    param(
        [string]$PackageName,
        [string]$FileName
    )

    foreach ($platformIoHome in (Get-PlatformIoHomes)) {
        $packageRoot = Join-Path (Join-Path $platformIoHome 'packages') $PackageName
        if (Test-Path -LiteralPath $packageRoot -PathType Container) {
            $match = Get-ChildItem -LiteralPath $packageRoot -Recurse -File -Filter $FileName |
                Select-Object -First 1
            if ($match) {
                return $match.FullName
            }
        }
    }

    return $null
}

function Find-BootloaderFile {
    param([string]$FileName)

    foreach ($platformIoHome in (Get-PlatformIoHomes)) {
        $packageRoot = Join-Path (Join-Path $platformIoHome 'packages') 'framework-arduino-avr'
        if (Test-Path -LiteralPath $packageRoot -PathType Container) {
            $match = Get-ChildItem -LiteralPath $packageRoot -Recurse -File -Filter $FileName |
                Select-Object -First 1
            if ($match) {
                return $match.FullName
            }
        }
    }

    return $null
}

try {
    $avrdude = Find-PackageFile -PackageName 'tool-avrdude' -FileName 'avrdude.exe'
    $avrdudeConfig = Find-PackageFile -PackageName 'tool-avrdude' -FileName 'avrdude.conf'
    $optiboot = Find-BootloaderFile -FileName 'optiboot_atmega328.hex'
    $legacyBootloader = Find-BootloaderFile -FileName 'ATmegaBOOT_168_atmega328.hex'

    if (-not $avrdude -or -not $avrdudeConfig) {
        throw 'Tidak menemukan avrdude.exe dan/atau avrdude.conf di paket PlatformIO tool-avrdude.'
    }
    if (-not $optiboot -and -not $legacyBootloader) {
        throw 'Tidak menemukan file bootloader ATmega328P di framework-arduino-avr.'
    }

    Write-Host 'Pilih bootloader target ATmega328P:'
    if ($optiboot) {
        Write-Host '  [1] Optiboot (optiboot_atmega328.hex; Nano PlatformIO saat ini)'
    }
    if ($legacyBootloader) {
        Write-Host '  [2] ATmegaBOOT lama (ATmegaBOOT_168_atmega328.hex)'
    }

    while ($true) {
        $choice = Read-Host 'Pilihan'
        if ($choice -eq '1' -and $optiboot) {
            $bootloader = $optiboot
            $bootloaderName = 'Optiboot'
            break
        }
        if ($choice -eq '2' -and $legacyBootloader) {
            $bootloader = $legacyBootloader
            $bootloaderName = 'ATmegaBOOT lama'
            break
        }
        Write-Host 'Pilihan tidak tersedia. Pilih nomor bootloader yang terdeteksi.' -ForegroundColor Yellow
    }

    Write-Host "`nNano PlatformIO menggunakan high fuse 0xDA. High fuse 0xDE hanya gunakan jika memang cocok dengan konfigurasi board Anda."
    $highFuseInput = Read-Host 'High fuse [DA/DE] (default DA)'
    if ([string]::IsNullOrWhiteSpace($highFuseInput)) {
        $highFuseInput = 'DA'
    }
    $highFuseInput = $highFuseInput.Trim().ToUpperInvariant()
    if ($highFuseInput -notin @('DA', 'DE')) {
        throw "High fuse tidak valid: $highFuseInput. Pilih DA atau DE."
    }
    $highFuse = '0x' + $highFuseInput

    if (-not ([System.IO.Ports.SerialPort]::GetPortNames() -contains $Port)) {
        Write-Host "Peringatan: $Port tidak terdeteksi. Pastikan port benar dan tidak digunakan Serial Monitor." -ForegroundColor Yellow
    }

    Write-Host "`n=== Rencana burn bootloader ===" -ForegroundColor Cyan
    Write-Host "avrdude       : $avrdude"
    Write-Host "Konfigurasi   : $avrdudeConfig"
    Write-Host "Bootloader    : $bootloader ($bootloaderName)"
    Write-Host 'Programmer    : stk500v1'
    Write-Host "Port / baud   : $Port / $ProgrammerBaud"
    Write-Host "Part          : $Part"
    Write-Host 'LFuse / EFuse : 0xFF / 0xFD'
    Write-Host "HFuse         : $highFuse"
    Write-Host 'Unlock / lock : 0x3F / 0x0F'

    Write-Host "`nSebelum lanjut:" -ForegroundColor Yellow
    Write-Host '- Tutup Serial Monitor/Plotter dan proses lain yang memakai COM9.'
    Write-Host '- Pastikan kabel ISP: D10 programmer ke RESET target, D11 ke MOSI, D12 ke MISO, D13 ke SCK, serta GND dan VCC.'
    Write-Host '- Target harus mendapat catu yang sesuai; hindari dua sumber 5 V yang saling memberi daya.'
    Write-Host '- Reset fisik yang dimaksud di bawah adalah tombol RESET Nano programmer, bukan RESET target.'
    Write-Host '- Tanpa kapasitor: tahan RESET programmer saat hitung mundur, lalu lepaskan tepat saat avrdude mulai membuka COM9. Trik timing ini tidak selalu berhasil.'
    Write-Host '- Cara lebih andal: pasang kapasitor 10 uF antara RESET dan GND pada Nano programmer untuk menekan auto-reset DTR; jika elektrolit, kaki + ke RESET dan - ke GND. Jangan pasang pada target.'
    Write-Host '- Baud 19200 adalah baud serial ArduinoISP, bukan baud upload bootloader target.'

    $confirmation = Read-Host "`nKetik FLASH untuk menghapus/menulis fuse, bootloader, dan lock bit pada chip di COM9"
    if ($confirmation -cne 'FLASH') {
        Write-Host 'Dibatalkan; avrdude tidak dijalankan.' -ForegroundColor Yellow
        exit 0
    }

    Write-Host "`nMulai dalam 5 detik. Siapkan tombol RESET programmer jika mencoba trik timing tanpa kapasitor."
    for ($seconds = 5; $seconds -ge 1; $seconds--) {
        Write-Host -NoNewline "`rMulai dalam $seconds detik... "
        Start-Sleep -Seconds 1
    }
    Write-Host "`rMenjalankan avrdude...                         "

    $arguments = @(
        '-C', $avrdudeConfig,
        '-p', $Part,
        '-c', 'stk500v1',
        '-P', $Port,
        '-b', "$ProgrammerBaud",
        '-e',
        '-U', 'lock:w:0x3F:m',
        '-U', 'lfuse:w:0xFF:m',
        '-U', ('hfuse:w:{0}:m' -f $highFuse),
        '-U', 'efuse:w:0xFD:m',
        '-U', ('flash:w:{0}:i' -f $bootloader),
        '-U', 'lock:w:0x0F:m'
    )

    $previousErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & $avrdude @arguments
        $avrdudeExitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }

    if ($avrdudeExitCode -ne 0) {
        throw "avrdude gagal dengan exit code $avrdudeExitCode. Periksa koneksi ISP, port, baud, dan reset programmer."
    }

    Write-Host "`nBootloader berhasil ditulis dan lock bit disetel ke 0x0F." -ForegroundColor Green
}
catch {
    Write-Error $_
    exit 1
}