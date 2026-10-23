/*
 * ============================================================================
 * FIRMWARE TRANSMITTER (TX) - RC UNIVERSAL HYBRID (ESC & MOSFET)
 * ============================================================================
 * Fitur: 10 Slot Memori, 4 Mode Terbang, Pemilihan Tipe Motor, 
 * Continuous Trim, Failsafe Analog Mati (Stik Kabel), & Telemetry Alarm.
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <PS2X_lib.h>
#include <EEPROM.h>

// ============================================================================
// 1. KONFIGURASI PIN HARDWARE (SESUAIKAN DENGAN JALUR KABEL NANO)
// ============================================================================
#define PIN_RADIO_CE   9  // Pin CE untuk modul NRF24L01
#define PIN_RADIO_CSN  10 // Pin CSN untuk modul NRF24L01
#define PS2_DAT        4  // Pin Data Stik PS2
#define PS2_CMD        5  // Pin Command Stik PS2
#define PS2_ATT        6  // Pin Attention (CS) Stik PS2
#define PS2_CLK        7  // Pin Clock Stik PS2
#define PIN_LED_HIJAU  2  // Indikator TX Siap / Armed
#define PIN_LED_MERAH  3  // Indikator Peringatan / Error / Disarm
#define PIN_BUZZER     8  // Output Suara (Buzzer Aktif)

const byte BIND_ADDR[6] = "BIND0"; // Alamat Pipa Khusus untuk proses Binding
const uint8_t BIND_CH   = 10;      // Channel/Frekuensi khusus Binding
const uint8_t MAX_MODELS = 10;     // Maksimal kapasitas slot pesawat di EEPROM

// ============================================================================
// 2. STRUKTUR DATA (PAKET YANG DIKIRIM KE RX)
// ============================================================================
// Struktur data untuk proses Binding (Menyamakan Channel & Pipa)
struct BindPayload {
  byte pipeAddr[5];
  uint8_t opChannel;
  uint8_t checkSum;
};

// Struktur data utama untuk mengontrol pesawat (Dikirim setiap 30ms)
struct ControlData {
  uint8_t throttle;  // Nilai gas (0 - 255)
  int8_t  rudder;    // Nilai belok kiri/kanan (-127 s/d 127)
  int8_t  elevator;  // Nilai naik/turun (-127 s/d 127)
  int8_t  aileron;   // Nilai putar badan (-127 s/d 127)
  uint8_t isArmed;   // Status keamanan motor (0 = Terkunci, 1 = Hidup)
  uint8_t auxFlag;   // Mode Terbang (1=Normal, 2=Diferensial, 3=VTail, 4=Mobil)
  uint8_t motorType; // Jenis Output Motor (0 = ESC Brushless, 1 = MOSFET DC)
};
ControlData txData;

// Struktur data balasan dari pesawat (Telemetri)
struct TelemetryData {
  uint16_t batteryMilliVolt; // Tegangan baterai pesawat
  uint8_t  failsafeActive;   // Status RX (apakah sedang hilang sinyal)
};
TelemetryData rxTelem;

// ============================================================================
// 3. DEKLARASI OBJEK & VARIABEL GLOBAL
// ============================================================================
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
PS2X ps2x;

uint8_t currentSlot = 1;     // Slot EEPROM yang sedang aktif (1-10)
byte activePipe[5];          // Alamat radio model saat ini
uint8_t activeChannel;       // Frekuensi radio model saat ini
uint8_t activeMode = 1;      // Mode terbang yang sedang aktif
uint8_t activeMotorType = 0; // Tipe Motor (0 = ESC)

int throttleTrim = 0, rudderTrim = 0, elevatorTrim = 0, aileronTrim = 0;

unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL = 30; // Kecepatan kirim data = 30ms
unsigned long lastAckTime = 0;        // Catatan waktu terakhir RX membalas data

bool isArmed = false;           // Status Arming (Motor nyala/mati)
bool ps2Initialized = false;    // Status apakah stik berhasil konek
bool isTelemetryLost = false;   // Status deteksi putus sinyal

// ============================================================================
// 4. FUNGSI MANAJEMEN BUZZER & LED MERAH SINKRON (AUDIO & VISUAL)
// ============================================================================
unsigned long buzzerEndTime = 0;
bool buzzerActive = false;

// [FUNGSI] Menyalakan buzzer & LED Merah tanpa menghentikan kode (Non-Blocking)
void beep(unsigned int duration) {
  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(PIN_LED_MERAH, HIGH); // LED Merah Sinkron Nyala
  buzzerActive = true;
  buzzerEndTime = millis() + duration;
}

// [FUNGSI] Mematikan buzzer & LED Merah jika waktunya sudah habis (Dipanggil terus di loop)
void handleBuzzer() {
  if (buzzerActive && millis() >= buzzerEndTime) {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED_MERAH, LOW); // LED Merah Sinkron Mati
    buzzerActive = false;
  }
}

// [FUNGSI] Menyalakan buzzer & LED Merah dengan delay kaku (Hanya dipakai saat Setting/Booting)
void beepBlocking(unsigned int duration) {
  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(PIN_LED_MERAH, HIGH); // LED Merah Sinkron Nyala
  delay(duration);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_MERAH, LOW);  // LED Merah Sinkron Mati
}

// ============================================================================
// 5. FUNGSI MANAJEMEN EEPROM (PENYIMPANAN DATA SLOT)
// ============================================================================
// [FUNGSI] Mencari alamat blok memori untuk slot tertentu
int getSlotOffset(uint8_t slot) {
  return 100 + ((slot - 1) * 15); // Tiap slot punya ruang 15 byte
}

// [FUNGSI] Menyimpan konfigurasi Mode, Trim, dan Tipe Motor secara permanen ke Nano
void saveConfigToSlot() {
  int offset = getSlotOffset(currentSlot);
  EEPROM.update(offset + 7,  activeMode);
  EEPROM.update(offset + 8,  (uint8_t)(throttleTrim + 128)); 
  EEPROM.update(offset + 9,  (uint8_t)(rudderTrim + 128));
  EEPROM.update(offset + 10, (uint8_t)(elevatorTrim + 128));
  EEPROM.update(offset + 11, (uint8_t)(aileronTrim + 128));
  EEPROM.update(offset + 12, activeMotorType);
  
  // Nada konfirmasi penyimpanan berhasil
  beepBlocking(100); delay(150); beepBlocking(100); delay(150); beepBlocking(300); 
  Serial.println(F("[EEPROM] Konfigurasi Berhasil Disimpan!"));
}

// [FUNGSI] Mengambil memori pengaturan dari Nano saat pindah slot / pertama nyala
void loadModel(uint8_t slot) {
  int offset = getSlotOffset(slot);
  
  // Jika slot belum pernah dipakai, buat format data acak baru (inisialisasi)
  if (EEPROM.read(offset) != 0xAA) { 
    EEPROM.update(offset, 0xAA);
    EEPROM.update(offset + 1, random(20, 95)); // Channel acak
    for (int i = 0; i < 5; i++) EEPROM.update(offset + 2 + i, 'A' + random(0, 26)); // Pipa acak
    EEPROM.update(offset + 7, 1);    // Default Mode Normal (1)
    EEPROM.update(offset + 8, 128);  // Default Trim Thr 0
    EEPROM.update(offset + 9, 128);  // Default Trim Rud 0
    EEPROM.update(offset + 10, 128); // Default Trim Ele 0
    EEPROM.update(offset + 11, 128); // Default Trim Ail 0
    EEPROM.update(offset + 12, 0);   // Default Tipe ESC (0)
  }

  // Tarik data dari EEPROM ke variabel aktif
  activeChannel = EEPROM.read(offset + 1);
  for (int i = 0; i < 5; i++) activePipe[i] = EEPROM.read(offset + 2 + i);
  activeMode = EEPROM.read(offset + 7);
  throttleTrim = (int)EEPROM.read(offset + 8) - 128;
  rudderTrim   = (int)EEPROM.read(offset + 9) - 128;
  elevatorTrim = (int)EEPROM.read(offset + 10) - 128;
  aileronTrim  = (int)EEPROM.read(offset + 11) - 128;
  activeMotorType = EEPROM.read(offset + 12);

  // Simpan slot terakhir yang dibuka agar saat Nano mati, menyala di slot yang sama
  EEPROM.update(50, slot); 
  currentSlot = slot;
  
  Serial.print(F("[LOAD] Slot: ")); Serial.print(currentSlot);
  Serial.print(F(" | Mode: ")); Serial.print(activeMode);
  Serial.print(F(" | Motor: ")); Serial.println(activeMotorType == 0 ? F("ESC") : F("MOSFET"));

  // Bunyikan jumlah slot (contoh: slot 3 bunyi bip 3 kali)
  for (int i = 0; i < slot; i++) { beepBlocking(100); delay(200); }
}

// ============================================================================
// 6. FUNGSI BINDING (MENJODOHKAN TX DAN RX)
// ============================================================================
// [FUNGSI] Mengirim data pipa & channel ke RX baru selama 10 Detik
void executeBindingProcess() {
  Serial.println(F("[BIND] Memulai Proses Binding 10 Detik..."));
  radio.setChannel(BIND_CH);
  radio.openWritingPipe(BIND_ADDR);
  radio.stopListening();

  BindPayload bp;
  memcpy(bp.pipeAddr, activePipe, 5);
  bp.opChannel = activeChannel;
  bp.checkSum = activeChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4];

  unsigned long bindStartTime = millis();
  while (millis() - bindStartTime < 10000) {
    radio.write(&bp, sizeof(BindPayload));
    digitalWrite(PIN_BUZZER, HIGH); digitalWrite(PIN_LED_MERAH, HIGH); 
    delay(40); 
    digitalWrite(PIN_BUZZER, LOW); digitalWrite(PIN_LED_MERAH, LOW);
    digitalWrite(PIN_LED_HIJAU, HIGH); delay(80); digitalWrite(PIN_LED_HIJAU, LOW);
    delay(150);
  }
  beepBlocking(500); // Nada Binding Selesai
  Serial.println(F("[BIND] Selesai."));
}

// ============================================================================
// 7. FUNGSI UTAMA: SETUP (DIJALANKAN SEKALI SAAT DINYALAKAN)
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_HIJAU, OUTPUT);
  pinMode(PIN_LED_MERAH, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW); 
  randomSeed(analogRead(A6) + analogRead(A7));

  Serial.println(F("\n=== BOOTING TRANSMITTER ==="));
  
  // Baca slot terakhir yang dipakai
  uint8_t lastSlot = EEPROM.read(50);
  if (lastSlot < 1 || lastSlot > MAX_MODELS) lastSlot = 1;
  loadModel(lastSlot); // Panggil fungsi Load Data

  // Coba hubungkan ke Stik PS2 maksimal 15 kali percobaan
  int error = -1; byte retry = 0;
  while (error != 0 && retry < 15) {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
    if (error == 0) { ps2Initialized = true; break; }
    delay(300); retry++;
  }

  // Cek modul antena NRF24L01
  if (!radio.begin()) { 
    Serial.println(F("[FATAL] Radio NRF24 Tidak Terdeteksi! Cek Kabel."));
    while(1); // Sistem berhenti total jika antena tidak ada
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.enableDynamicPayloads();
  radio.enableAckPayload(); // Aktifkan fitur terima data (telemetri)

  if (ps2Initialized) {
    // --- CEK KOMBINASI TOMBOL BINDING (L1 + R1) SAAT NYALA ---
    ps2x.read_gamepad(false, 0);
    if (ps2x.Button(PSB_L1) && ps2x.Button(PSB_R1)) executeBindingProcess();
    
    // --- PENGECEKAN PRA-TERBANG (CEK KABEL & CEK GAS NOL) ---
    bool preFlightSafe = false;
    while (!preFlightSafe) {
      
      // validRead mendeteksi apakah kabel nyambung & stik aktif
      bool validRead = ps2x.read_gamepad(false, 0);
      int ly = ps2x.Analog(PSS_LY); // Baca posisi tuas gas (0 atas, 255 bawah)

      Serial.print(F("[PRE-FLIGHT] Valid(Stik Konek): ")); Serial.print(validRead);
      Serial.print(F(" | LY(Tuas Gas): ")); Serial.println(validRead ? ly : -1);

      if (!validRead) {
        // ERROR 1: Stik Putus / Analog Mati
        digitalWrite(PIN_LED_MERAH, HIGH);
        beepBlocking(80); delay(80); beepBlocking(80);
        digitalWrite(PIN_LED_MERAH, LOW);
        delay(760); 
      } 
      else if (ly < 235) { 
        // ERROR 2: Gas belum di bawah (Berbahaya jika baling-baling nyala)
        digitalWrite(PIN_LED_MERAH, HIGH);
        beepBlocking(60); 
        digitalWrite(PIN_LED_MERAH, LOW);
        delay(200);
      } 
      else {
        // LOLOS CEK AMAN
        preFlightSafe = true; 
      }
    }
  }

  // Buka jalur komunikasi NRF24 sesuai data model aktif
  radio.setChannel(activeChannel);
  radio.openWritingPipe(activePipe);
  radio.stopListening();
  
  // Nada Siap Terbang
  beepBlocking(200);
  digitalWrite(PIN_LED_HIJAU, HIGH);
  Serial.println(F("=== TX SIAP BEROPERASI ==="));
}

// ============================================================================
// 8. FUNGSI UTAMA: LOOP (DIJALANKAN BERULANG-ULANG TERUS MENERUS)
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();
  
  // Matikan buzzer/LED secara otomatis jika waktunya habis
  handleBuzzer(); 

  // --- ALARM: KEHILANGAN SINYAL TELEMETRI DARI PESAWAT (>1 Detik) ---
  if (currentMillis - lastAckTime > 1000 && isArmed) {
    if (!isTelemetryLost) {
      Serial.println(F("[WARNING] Telemetri RX Putus! Pesawat Terlalu Jauh."));
      isTelemetryLost = true;
    }
    // Bunyikan Alarm + LED Merah (100ms nyala, 900ms mati)
    if ((currentMillis % 1000) < 100) {
      digitalWrite(PIN_BUZZER, HIGH);
      digitalWrite(PIN_LED_MERAH, HIGH); 
    } else if (!buzzerActive) {
      digitalWrite(PIN_BUZZER, LOW);
      digitalWrite(PIN_LED_MERAH, LOW);
    }
  } else {
    isTelemetryLost = false; // Sinyal aman
  }

  // --- PROSES PEMBACAAN STIK & PENGIRIMAN DATA (SETIAP 30ms) ---
  if (currentMillis - lastTxTime >= TX_INTERVAL) {
    lastTxTime = currentMillis;
    
    int thrVal = 0, rudVal = 0, eleVal = 0, ailVal = 0;
    bool analogNormal = false; // Indikator keamanan stik

    if (ps2Initialized) {
      
      // JIKA STIK BERHASIL DIBACA (TIDAK KENDOR / ANALOG NYALA)
      if (ps2x.read_gamepad(false, 0)) {
        analogNormal = true;
        
        // Baca murni sumbu analog
        int rawLY = ps2x.Analog(PSS_LY); int rawLX = ps2x.Analog(PSS_LX);
        int rawRY = ps2x.Analog(PSS_RY); int rawRX = ps2x.Analog(PSS_RX);

        // --- KONTROL MENU KOMBINASI TOMBOL (Ditekan bersama tombol SELECT) ---
        if (ps2x.Button(PSB_SELECT)) {
          // Ganti Slot (SELECT + KANAN)
          if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
             currentSlot = (currentSlot % MAX_MODELS) + 1;
             isArmed = false; loadModel(currentSlot);
             radio.setChannel(activeChannel); radio.openWritingPipe(activePipe);
          }
          // Ganti Mode Terbang (SELECT + SEGITIGA)
          else if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
             activeMode = (activeMode % 4) + 1;
             Serial.print(F("[SETTING] Mode Terbang: ")); Serial.println(activeMode);
             beepBlocking(400); delay(200); 
             for(int i=0; i<activeMode; i++){ beepBlocking(100); delay(150); }
          }
          // Ganti Tipe Motor (SELECT + KOTAK)
          else if (ps2x.ButtonPressed(PSB_SQUARE)) {
             activeMotorType = (activeMotorType == 0) ? 1 : 0;
             Serial.print(F("[SETTING] Tipe Motor: ")); Serial.println(activeMotorType == 0 ? F("ESC") : F("MOSFET"));
             if(activeMotorType == 0) beepBlocking(150); // 1 Bip = ESC
             else { beepBlocking(100); delay(150); beepBlocking(100); } // 2 Bip = MOSFET
          }
          // Reset Semua Trim ke Nol (SELECT + SILANG)
          else if (ps2x.ButtonPressed(PSB_CROSS)) { 
             throttleTrim = 0; rudderTrim = 0; elevatorTrim = 0; aileronTrim = 0; 
             beep(400); 
          }
          // Simpan Pengaturan ke EEPROM (SELECT + LINGKARAN)
          else if (ps2x.ButtonPressed(PSB_CIRCLE)) saveConfigToSlot();
        }

        // --- KONTROL CONTINUOUS TRIM (Tahan L1 / R1 + Tombol Arah) ---
        static unsigned long lastTrimTime = 0;
        if (currentMillis - lastTrimTime > 150) { 
          bool trimChanged = false;
          // Trim Stik Kiri (L1) -> Gas dan Belok Kemudi
          if (ps2x.Button(PSB_L1)) {
            if (ps2x.Button(PSB_PAD_UP))    { throttleTrim = constrain(throttleTrim+3, -127, 127); trimChanged = true; }
            if (ps2x.Button(PSB_PAD_DOWN))  { throttleTrim = constrain(throttleTrim-3, -127, 127); trimChanged = true; }
            if (ps2x.Button(PSB_PAD_RIGHT)) { rudderTrim   = constrain(rudderTrim+2, -127, 127); trimChanged = true; }
            if (ps2x.Button(PSB_PAD_LEFT))  { rudderTrim   = constrain(rudderTrim-2, -127, 127); trimChanged = true; }
          }
          // Trim Stik Kanan (R1) -> Naik/Turun dan Putar Badan
          if (ps2x.Button(PSB_R1)) {
            if (ps2x.Button(PSB_PAD_UP))    { elevatorTrim = constrain(elevatorTrim+2, -127, 127); trimChanged = true; }
            if (ps2x.Button(PSB_PAD_DOWN))  { elevatorTrim = constrain(elevatorTrim-2, -127, 127); trimChanged = true; }
            if (ps2x.Button(PSB_PAD_RIGHT)) { aileronTrim  = constrain(aileronTrim+2, -127, 127); trimChanged = true; }
            if (ps2x.Button(PSB_PAD_LEFT))  { aileronTrim  = constrain(aileronTrim-2, -127, 127); trimChanged = true; }
          }
          if (trimChanged) { beep(30); lastTrimTime = currentMillis; } // Bunyi klik saat trim diubah
        }

        // --- TOMBOL START (MENGHIDUPKAN / MEMATIKAN MESIN) ---
        if (ps2x.ButtonPressed(PSB_START)) {
          if (!isArmed && rawLY < 235) { // Tolak nyala jika gas masih tinggi
            Serial.println(F("[ARM DITOLAK] Turunkan gas ke bawah dulu!"));
            beepBlocking(50); delay(50); beepBlocking(50);
          } else {
            isArmed = !isArmed; // Ubah status (Nyala/Mati)
            if (isArmed) { 
               beep(600); // Bip Panjang = Motor Siap 
            } else { 
               beepBlocking(100); delay(100); beepBlocking(100); // 2 Bip Pendek = Motor Kunci
            }
          }
        }

        // --- KALKULASI HASIL AKHIR (Stik + Trim) ---
        thrVal = (rawLY < 240) ? constrain(map(rawLY, 240, 0, 0, 255) + throttleTrim, 0, 255) : 0;
        rudVal = constrain(map(rawLX, 0, 255, -127, 127) + rudderTrim, -127, 127);
        eleVal = constrain(map(rawRY, 0, 255, 127, -127) + elevatorTrim, -127, 127);
        ailVal = constrain(map(rawRX, 0, 255, -127, 127) + aileronTrim, -127, 127);

        // Deadband (Abaikan getaran kecil pada stik tengah)
        if (abs(rudVal) < 10) rudVal = 0; 
        if (abs(ailVal) < 10) ailVal = 0; 
        if (abs(eleVal) < 10) eleVal = 0;

        // Cetak Debug Normal ke Komputer (Tiap Setengah Detik)
        static unsigned long lastDebugPrint = 0;
        if (currentMillis - lastDebugPrint >= 500) {
          lastDebugPrint = currentMillis;
          Serial.print(F("[NORMAL] Mtr:")); Serial.print(activeMotorType == 0 ? F("ESC") : F("FET"));
          Serial.print(F(" | Arm:")); Serial.print(isArmed);
          Serial.print(F(" | LY:")); Serial.print(rawLY);
          Serial.print(F(" -> Thr:")); Serial.println(thrVal);
        }

      } else {
        // ====================================================================
        // JIKA STIK GAGAL DIBACA (KABEL TERCABUT / TOMBOL ANALOG DIMATIKAN)
        // ====================================================================
        analogNormal = false;
        
        // Segera Kunci Motor (Disarm Darurat)
        if (isArmed) { 
          isArmed = false; 
          Serial.println(F("\n[EMERGENCY DISARM] Stik Analog Mati / Kabel Lepas!"));
        }
        
        // Bunyikan Alarm Bip-Bip dan LED Merah tiap 1 Detik
        static unsigned long lastAnalogBeep = 0;
        if (currentMillis - lastAnalogBeep >= 1000) {
          lastAnalogBeep = currentMillis;
          beep(150); // Fungsi beep() ini sudah menyalakan Buzzer & LED Merah
          Serial.println(F("[ALARM] Analog Mati! Cek Kabel atau Tekan Tombol Analog."));
        }
      }
    }

    // --- PACKING DATA UNTUK DIKIRIM KE PESAWAT ---
    txData.isArmed = isArmed ? 1 : 0;
    txData.auxFlag = activeMode;
    txData.motorType = activeMotorType; 

    // PAKSA NILAI GAS = 0 JIKA MOTOR TERKUNCI ATAU STIK ERROR
    if (!isArmed || !analogNormal) {
      txData.throttle = 0; txData.rudder = 0; txData.elevator = 0; txData.aileron = 0;
    } else {
      txData.throttle = (uint8_t)thrVal; txData.rudder = (int8_t)rudVal;
      txData.elevator = (int8_t)eleVal;  txData.aileron = (int8_t)ailVal;
    }

    // KIRIM DATA VIA NRF24L01
    if (radio.write(&txData, sizeof(ControlData))) {
      // Jika berhasil dikirim, periksa apakah RX menitipkan data telemetri (Baterai dll)
      if (radio.isAckPayloadAvailable()) {
        radio.read(&rxTelem, sizeof(TelemetryData));
        lastAckTime = currentMillis; // Catat waktu sukses sinyal untuk keamanan
      }
    }
  }
}
// =================================== AKHIR KODE ===================================