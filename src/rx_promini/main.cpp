/*
 * ============================================================================
 * FIRMWARE RECEIVER (RX) - RC UNIVERSAL HYBRID (PESAWAT & MOBIL RC PROPO)
 * ============================================================================
 * KEMAMPUAN & FLEKSIBILITAS FIRMWARE:
 * 1. Multi-Kendaraan (1 Firmware untuk Semua):
 *    - Mode 1 (Pesawat Normal)      : Motor + Servo Elevator & Aileron
 *    - Mode 2 (Pesawat Diferensial) : Belok pakai beda kecepatan 2 motor (Expo + Anti-Stall)
 *    - Mode 3 (Pesawat V-Tail/Delta): Mixer Elevon otomatis
 *    - Mode 4 (Mobil / Perahu RC)   : Setir EPA, ESC Maju-Mundur, AUX1, AUX2, Lampu Rem D8
 *
 * 2. Kompatibel Berbagai Hardware (Cukup Ubah Panel Konfigurasi di Bagian 0):
 *    - Mendukung ESC Brushless maupun MOSFET PWM (Mode Pesawat).
 *    - Saklar Profil ESC Mobil (MODE_ESC_SG90):
 *        * 1 = Profil ESC Modif SG90 (Kurva Expo Integer + Auto-Detach Anti Getar)
 *        * 0 = Profil ESC Brushed Asli 10A/20A/30A (Linier 1000-2000us)
 *    - Mendukung Pro Mini 3.3V maupun 5V (Kalibrasi sensor baterai cukup ubah 1 angka).
 *    - Saklar Debug Master: Ubah AKTIFKAN_SERIAL ke 0 untuk mematikan total semua Serial
 *      (nol beban RAM/Flash & nol interupsi UART saat terbang/jalan permanen).
 *
 * 3. Keamanan & Radio:
 *    - FHSS 4-Channel (Anti-Interferensi) + Auto-Binding EEPROM + Watchdog Timer 250ms.
 *    - Failsafe Dinamis: 500ms (Pesawat Mode 1-3) & 1000ms (Mobil RC Mode 4).
 *    - Telemetri Baterai 1S & Lost Model Finder (khusus Mode 1-3 Pesawat).
 *
 * ============================================================================
 * PANDUAN INSTALASI HARDWARE & PEMETAAN PIN (ARDUINO PRO MINI)
 * ============================================================================
 * 1. MODUL RADIO NRF24L01 (SPI 2 MHz):
 *    - VCC  -> 3.3V STABIL (LDO Terpisah! Pasang Elko 10-100uF + Keramik 104 di kaki VCC-GND)
 *    - GND  -> GND Arduino
 *    - CE   -> Pin D9   | CSN -> Pin D10 | MOSI -> Pin D11 | MISO -> Pin D12 | SCK -> Pin D13
 *
 * 2. PIN SISTEM, INDIKATOR & SENSOR:
 *    - Pin D2 (BIND)   -> Jumper ke GND saat dinyalakan untuk Reset & Mode Binding
 *    - Pin D3 (LED)    -> LED Status Heartbeat (+ Resistor 220-330 Ohm ke GND)
 *    - Pin A0 (VBATT)  -> Sensor Baterai 1S via Pembagi Tegangan 10k / 10k
 *
 * 3. PEMETAAN PIN OUTPUT BERSAMA (PESAWAT vs MOBIL RC):
 *    - Pin D4 (PIN_SERVO_3) : Mode 1-3: Elevator / V-Tail 1 | Mode 4: Servo Setir Kemudi
 *    - Pin D7 (PIN_SERVO_4) : Mode 1-3: Aileron  / V-Tail 2 | Mode 4: ESC Maju-Mundur
 *    - Pin D5 (PIN_MOTOR_1) : Mode 1-3: Motor Kiri / Utama  | Mode 4: AUX1 (Lampu Depan & Dim)
 *    - Pin D6 (PIN_MOTOR_2) : Mode 1-3: Motor Kanan         | Mode 4: AUX2 (Musik / Klakson)
 *    - Pin D8 (PIN_BUZZER)  : Mode 1-3: Buzzer Finder       | Mode 4: Lampu Belakang & Rem (+R 220-330 Ohm)
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>       // Komunikasi SPI berkecepatan tinggi untuk NRF24L01
#include <RF24.h>      // Library pengendali modul radio NRF24L01
#include <Servo.h>     // Pembangkit sinyal PWM presisi (1000-2000us) untuk Servo & ESC
#include <EEPROM.h>    // Penyimpanan permanen data Binding
#include <avr/wdt.h>   // Watchdog Timer (Otomatis restart jika sistem/NRF macet)

// ============================================================================
// 0. PANEL KONFIGURASI MASTER (CUKUP UBAH DI SINI SEBELUM UPLOAD)
// ============================================================================

// --- A. Saklar Debugging Serial ---
// AKTIFKAN_SERIAL  : 1 = Nyalakan Serial Monitor (tuning di meja), 0 = Mati total (WAJIB 0 saat final!)
// CEK_REGISTER_NRF : 1 = Muat printf.h & printDetails() (butuh Flash ekstra ~4KB), 0 = Hemat Flash
#define AKTIFKAN_SERIAL    0
#define CEK_REGISTER_NRF   0

// --- B. Kalibrasi Sensor Tegangan Baterai 1S (Pembagi 10k / 10k di Pin A0) ---
// 6600 untuk Pro Mini 3.3V | 10000 untuk Pro Mini 5.0V
#define BATT_MAX_MV        10000
#define LOW_BATT_1S_MV     3500  // Batas alarm baterai lemah (3500 mV = 3.5V)

// --- C. Setelan AUX & Lampu Belakang/Rem (Khusus Mode 4: Mobil / Perahu RC) ---
#define AUX2_MODE_PULSA    true  // true = Pulsa sesaat (IC Musik/Klakson), false = Saklar ON/OFF
#define AUX2_AKTIF_LOW     true  // true = Aktif saat LOW (GND), false = Aktif saat HIGH (VCC)
#define AUX2_DURASI_MS     100   // Lama tekan tombol virtual AUX2 (milidetik)
#define AUX1_KEDIP_MS      120   // Kecepatan kedip lampu dim AUX1 (milidetik)
#define REM_DURASI_MS      1000  // Lama lampu rem (D8) menyala terang sesaat setelah berhenti (ms)
#define LAMPU_KOTA_REDUP   20    // Kecerahan lampu belakang redup (10 - 30% agar kontras rem jelas)

// --- D. Setelan Setir Mobil RC (Endpoint Adjustment / EPA Mikrodetik) ---
#define STEER_CENTER_US    1500  // Titik lurus roda depan (Sub-trim: 1400 - 1600)
#define STEER_MIN_US       1295  // Batas mentok belok KIRI
#define STEER_MAX_US       1705  // Batas mentok belok KANAN
#define STEER_REVERSE      false // Ubah ke true jika arah belok kiri/kanan terbalik

// --- E. Profil ESC Maju-Mundur Mobil RC (Cukup Ubah 1 Angka di MODE_ESC_SG90) ---
// 1 = ESC Modif Bekas Servo SG90     -> Aktifkan Kurva Expo & Auto-Detach
// 0 = ESC Brushed Asli (10A/20A/30A) -> Sinyal linier standar 1000-2000us, tanpa Detach
#define MODE_ESC_SG90      1
#define ESC_DEADZONE       10    // Zona mati tengah stik (-10 s/d +10) agar tidak merayap sendiri
#define ESC_REVERSE        false // Ubah ke true jika arah maju/mundur terbalik

#if MODE_ESC_SG90 == 1
  // Profil Khusus ESC Modif SG90 (Resistor 2.2k)
  #define ESC_AUTO_DETACH  true
  #define ESC_EXPO_PERSEN  70    // 0 = Linier, 40-85 = Tarikan awal halus, gas atas tetap ngebut (Maks 100)
  #define ESC_CENTER_US    1490  // Titik netral / diam dinamo
  #define ESC_MIN_US       1380  // Batas mundur penuh (-110 us dari center)
  #define ESC_MAX_US       1600  // Batas maju penuh   (+110 us dari center)
#else
  // Profil Standar ESC Brushed Pabrikan
  #define ESC_AUTO_DETACH  false
  #define ESC_EXPO_PERSEN  0     // Murni linier (kode Expo tidak ikut di-compile)
  #define ESC_CENTER_US    1500
  #define ESC_MIN_US       1000
  #define ESC_MAX_US       2000
#endif

// --- Pengaman Otomatis Mode Final (Jangan Diubah) ---
#if AKTIFKAN_SERIAL == 1
  #if CEK_REGISTER_NRF == 1
    #include <printf.h>
  #endif
#else
  // Menghapus total semua perintah Serial dari kompilasi (0 byte RAM/Flash & 0 interupsi)
  #define Serial if(false)Serial
#endif

// ============================================================================
// 1. KONFIGURASI PIN HARDWARE (ARDUINO PRO MINI)
// ============================================================================
#define PIN_BIND       2  // Jumper ke GND saat booting untuk masuk Mode Binding
#define PIN_LED        3  // Indikator Status RX (Heartbeat LED)
#define PIN_SERVO_3    4  // Mode 1-3: Servo Elevator   | Mode 4: Servo Setir Kemudi
#define PIN_MOTOR_1    5  // [PWM] Mode 1-3: Motor Kiri | Mode 4: AUX1 (Lampu Depan/Dim)
#define PIN_MOTOR_2    6  // [PWM] Mode 1-3: Motor Kanan| Mode 4: AUX2 (Musik/Pulsa)
#define PIN_SERVO_4    7  // Mode 1-3: Servo Aileron    | Mode 4: ESC Maju-Mundur
#define PIN_BUZZER     8  // Mode 1-3: Buzzer Finder    | Mode 4: Lampu Belakang & Rem
#define PIN_RADIO_CE   9  // NRF24L01 Chip Enable
#define PIN_RADIO_CSN  10 // NRF24L01 Chip Select Not
#define PIN_VBATT      A0 // Sensor Tegangan Baterai 1S (Voltage Divider 10k/10k)

const byte BIND_ADDR[6] = "BIND0"; // Alamat darurat statis khusus proses Binding
const uint8_t BIND_CH   = 10;      // Channel darurat khusus proses Binding

// ============================================================================
// 2. STRUKTUR DATA RADIO (WAJIB 100% IDENTIK DENGAN FIRMWARE TX)
// ============================================================================
struct BindPayload {
  byte pipeAddr[5];  // Alamat rahasia 5 byte dari TX
  uint8_t opChannel; // Channel bibit FHSS dari TX
  uint8_t checkSum;  // Verifikasi XOR anti-sinyal nyasar
};

struct ControlData {
  uint8_t throttle;  // 0 sampai 255
  int8_t  rudder;    // -127 sampai +127
  int8_t  elevator;  // -127 sampai +127
  int8_t  aileron;   // -127 sampai +127
  uint8_t isArmed;   // 0 = Disarmed (Motor Terkunci), 1 = Armed (Siap Jalan/Terbang)
  uint8_t auxFlag;   // 1 = Normal, 2 = Diferensial, 3 = V-Tail, 4 = Mobil RC Propo
  uint8_t motorType; // Mode Pesawat: 0 = ESC Brushless, 1 = MOSFET PWM
};
ControlData rxData;

struct TelemetryData {
  uint8_t batteryStatus;  // 0 = Normal, 1 = Baterai Lemah (Low Batt)
  uint8_t failsafeActive; // 0 = Sinyal Aman, 1 = Failsafe Aktif
};
TelemetryData txTelem;

// ============================================================================
// 3. OBJEK & VARIABEL GLOBAL
// ============================================================================
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN, 2000000); // SPI 2 MHz agar stabil & tahan noise
Servo servo3; // Pin D4 (Elevator / Setir Mobil)
Servo servo4; // Pin D7 (Aileron / ESC Maju-Mundur Mobil)
Servo esc1;   // Pin D5 (ESC Brushless 1 di Mode Pesawat)
Servo esc2;   // Pin D6 (ESC Brushless 2 di Mode Pesawat)

byte myPipe[5];                 // Alamat permanen dari EEPROM
uint8_t myChannel;              // Bibit channel permanen dari EEPROM
bool isBound = false;           // Status apakah RX sudah memiliki pasangan TX
unsigned long lastRecvTime = 0; // Waktu terakhir paket valid diterima

// --- Sistem Lompatan Frekuensi (FHSS 4-Channel) ---
const uint8_t FHSS_LEN = 4;
uint8_t fhssChannels[FHSS_LEN];
uint8_t hopIndex = 0;
unsigned long lastHopTime = 0;

void generateFHSSChannels(uint8_t seed) {
  for (uint8_t i = 0; i < FHSS_LEN; i++) {
    fhssChannels[i] = 20 + ((seed - 20 + (i * 23)) % 80); // Rentang aman Ch 20 s/d 99
  }
  hopIndex = 0;
}

void hopToNextChannel(unsigned long currentMillis) {
  hopIndex = (hopIndex + 1) % FHSS_LEN;
  radio.stopListening();
  radio.setChannel(fhssChannels[hopIndex]);
  radio.startListening();
  lastHopTime = currentMillis;
}

// --- Status Keselamatan & Baterai ---
bool failsafeActive = true;
bool escsAttached = false;
int currentThrottle = 0;
unsigned long lowBattTimer = 0;

// --- Status AUX & Lampu Rem (Khusus Mode 4 - Non-Blocking) ---
bool aux1Nyala = false;
bool kunciAux1Toggle = false;
bool kunciAux2 = false;
unsigned long waktuPulsaAux2 = 0;
bool sedangPulsaAux2 = false;
unsigned long waktuKedipAux1 = 0;
bool statusKedipAux1 = false;
bool escJalanSebelumnya = false;
unsigned long waktuRemAktif = 0;
bool remSedangNyala = false;

// ============================================================================
// 4. FUNGSI BINDING (PROSES JABAT TANGAN PERTAMA KALI)
// ============================================================================
void enterBindingMode() {
  Serial.println(F("[BIND] Masuk Mode Binding. Menunggu TX..."));
  radio.setChannel(BIND_CH);
  radio.openReadingPipe(1, BIND_ADDR);
  radio.startListening();

  while (!isBound) {
    if (radio.available()) {
      Serial.println(F("[BIND] Paket diterima, memverifikasi checksum..."));
      BindPayload bp;
      radio.read(&bp, sizeof(BindPayload));

      uint8_t calcSum = bp.opChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4];
      if (bp.checkSum == calcSum) {
        myChannel = bp.opChannel;
        memcpy(myPipe, bp.pipeAddr, 5);

        EEPROM.update(0, 0xBB);
        EEPROM.update(1, myChannel);
        for (int i = 0; i < 5; i++) EEPROM.update(2 + i, myPipe[i]);

        isBound = true;
        Serial.println(F("[BIND] Sukses! Data tersimpan di EEPROM."));
        tone(PIN_BUZZER, 2000, 1000);
        delay(1000);
      }
    }
    digitalWrite(PIN_LED, millis() % 200 < 100);
  }
}

// ============================================================================
// 5. FUNGSI UTAMA: SETUP (DIJALANKAN SEKALI SAAT RX DINYALAKAN)
// ============================================================================
void setup() {
  Serial.begin(115200);

  #if AKTIFKAN_SERIAL == 1 && CEK_REGISTER_NRF == 1
    printf_begin();
  #endif

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);

  // 1. Inisialisasi Hardware Radio NRF24L01
  if (!radio.begin()) {
    Serial.println(F("[FATAL] Radio NRF24 Rusak atau Kabel Lepas!"));
    while (1);
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  #if AKTIFKAN_SERIAL == 1 && CEK_REGISTER_NRF == 1
    Serial.println(F("\n=== CEK REGISTER NRF24L01 ==="));
    radio.printDetails();
    Serial.println(F("=============================\n"));
  #endif

  // 2. Cek Memori Binding EEPROM & Tombol Reset Binding (Pin D2)
  pinMode(PIN_BIND, INPUT_PULLUP);
  if (EEPROM.read(0) != 0xBB || digitalRead(PIN_BIND) == LOW) {
    enterBindingMode();
  } else {
    myChannel = EEPROM.read(1);
    for (int i = 0; i < 5; i++) myPipe[i] = EEPROM.read(2 + i);
    isBound = true;
    Serial.println(F("[SYSTEM] Load Data Binding dari EEPROM."));
  }

  // 3. Pembacaan Awal Tegangan Baterai 1S
  analogRead(PIN_VBATT);
  delay(5);
  uint16_t initialMilliVolt = map(analogRead(PIN_VBATT), 0, 1023, 0, BATT_MAX_MV);

  Serial.print(F("[BATT 1S] Tegangan Awal: "));
  Serial.print(initialMilliVolt);
  Serial.print(F(" mV | Batas Alarm: "));
  Serial.print(LOW_BATT_1S_MV);
  Serial.println(F(" mV"));

  // 4. Racik & Aktifkan Saluran FHSS Pertama
  generateFHSSChannels(myChannel);
  Serial.print(F("[FHSS] Daftar 4 Channel RX: "));
  for (uint8_t i = 0; i < FHSS_LEN; i++) {
    Serial.print(fhssChannels[i]);
    Serial.print(F(" "));
  }
  Serial.println();

  radio.stopListening();
  delay(10);
  hopIndex = 0;
  radio.setChannel(fhssChannels[hopIndex]);
  radio.openReadingPipe(1, myPipe);
  radio.closeReadingPipe(0); // Tutup Pipe 0 agar kebal noise alamat 0x0000000000
  radio.startListening();
  lastHopTime = millis();

  txTelem.batteryStatus = 0;
  txTelem.failsafeActive = 1;
  radio.writeAckPayload(1, &txTelem, sizeof(TelemetryData));

  // 5. Inisialisasi Aman Pin Servo & ESC Saat Booting
  servo3.writeMicroseconds(STEER_CENTER_US);
  servo3.attach(PIN_SERVO_3);

  if (ESC_AUTO_DETACH) {
    pinMode(PIN_SERVO_4, OUTPUT);
    digitalWrite(PIN_SERVO_4, LOW);
  } else {
    servo4.writeMicroseconds(ESC_CENTER_US);
    servo4.attach(PIN_SERVO_4);
  }

  // 6. Aktifkan Watchdog Timer 250ms
  wdt_enable(WDTO_250MS);
  Serial.print(F("[SYSTEM] RX Siap | Profil ESC Mobil: "));
  Serial.println(MODE_ESC_SG90 ? F("SG90 + EXPO") : F("BRUSHED ASLI"));
}

// ============================================================================
// 6. FUNGSI UTAMA: LOOP (EKSEKUSI REAL-TIME)
// ============================================================================
void loop() {
  wdt_reset();
  unsigned long currentMillis = millis();

  // --- A. PENERIMAAN DATA DARI TX ---
  if (radio.available()) {
    radio.read(&rxData, sizeof(ControlData));
    lastRecvTime = currentMillis;

    static int packetCount = 0;
    static unsigned long lastCountTime = 0;
    packetCount++;

    if (currentMillis - lastCountTime >= 1000) {
      Serial.print(F("[FHSS INFO] Paket/detik: "));
      Serial.print(packetCount);
      Serial.print(F(" | Batt: "));
      Serial.print(map(analogRead(PIN_VBATT), 0, 1023, 0, BATT_MAX_MV));
      Serial.print(F(" mV | Mode: "));
      Serial.println(rxData.auxFlag);

      if (packetCount > 100) {
        Serial.println(F("[FATAL] NRF HANG TERDETEKSI! Memaksa Watchdog Reset..."));
        while (1);
      }
      packetCount = 0;
      lastCountTime = currentMillis;
    }

    if (failsafeActive) {
      failsafeActive = false;
      rxData.isArmed = 0;
      noTone(PIN_BUZZER);
      digitalWrite(PIN_BUZZER, LOW);
      Serial.println(F("[RADIO] Link Tersambung Kembali. Motor Terkunci Otomatis."));
    }

    uint16_t currentMilliVolt = map(analogRead(PIN_VBATT), 0, 1023, 0, BATT_MAX_MV);
    if (currentMilliVolt <= LOW_BATT_1S_MV) {
      if (lowBattTimer == 0) lowBattTimer = currentMillis;
      if (currentMillis - lowBattTimer >= 2000) txTelem.batteryStatus = 1;
    } else {
      lowBattTimer = 0;
      txTelem.batteryStatus = 0;
    }

    txTelem.failsafeActive = 0;

    hopToNextChannel(currentMillis);
    radio.flush_tx();
    radio.writeAckPayload(1, &txTelem, sizeof(TelemetryData));
  }

  // --- A2. PENJAGA SINKRONISASI & SWEEP RESCUE FHSS ---
  unsigned long timeSinceLastRecv = currentMillis - lastRecvTime;
  unsigned long timeOnThisChannel = currentMillis - lastHopTime;

  if (timeSinceLastRecv < 150) {
    if (timeOnThisChannel >= 35) hopToNextChannel(currentMillis);
  } else {
    if (timeOnThisChannel >= 9)  hopToNextChannel(currentMillis);
  }

  // --- B. LOGIKA FAILSAFE DINAMIS (500ms PESAWAT, 1000ms MOBIL RC) ---
  unsigned long batasFailsafe = (rxData.auxFlag == 4) ? 1000 : 500;
  if (currentMillis - lastRecvTime > batasFailsafe) {
    if (!failsafeActive) {
      Serial.println(F("[FAILSAFE] Sinyal Hilang! Memicu Failsafe."));
      failsafeActive = true;
      radio.flush_tx();
      rxData.isArmed = 0;
      rxData.throttle = 0;

      esc1.detach(); esc2.detach();
      escsAttached = false;
      pinMode(PIN_MOTOR_1, OUTPUT);
      pinMode(PIN_MOTOR_2, OUTPUT);

      if (rxData.auxFlag == 4) {
        digitalWrite(PIN_MOTOR_1, LOW);
        digitalWrite(PIN_MOTOR_2, AUX2_AKTIF_LOW ? HIGH : LOW);
        sedangPulsaAux2 = false;
        remSedangNyala = false;
        escJalanSebelumnya = false;
        noTone(PIN_BUZZER);
        digitalWrite(PIN_BUZZER, LOW);
        servo3.writeMicroseconds(STEER_CENTER_US);

        if (ESC_AUTO_DETACH) {
          if (servo4.attached()) {
            servo4.detach();
            pinMode(PIN_SERVO_4, OUTPUT);
            digitalWrite(PIN_SERVO_4, LOW);
          }
        } else {
          if (!servo4.attached()) servo4.attach(PIN_SERVO_4);
          servo4.writeMicroseconds(ESC_CENTER_US);
        }
      } else {
        if (!servo4.attached()) servo4.attach(PIN_SERVO_4);
        digitalWrite(PIN_MOTOR_1, LOW);
        digitalWrite(PIN_MOTOR_2, LOW);
        servo3.write(90);
        servo4.write(90);
      }
    }
    txTelem.failsafeActive = 1;

    // Lost Model Finder khusus Pesawat (Mode 1-3); di Mode 4 Pin D8 adalah Lampu Belakang
    if (rxData.auxFlag != 4) {
      if (currentMillis - lastRecvTime > 15000) {
        if ((currentMillis % 2000) < 300) tone(PIN_BUZZER, 2500);
        else noTone(PIN_BUZZER);
      } else {
        noTone(PIN_BUZZER);
      }
    }
  }

  // --- C. EKSEKUSI PENGGERAK (HANYA BERJALAN SAAT SINYAL AMAN) ---
  if (!failsafeActive) {

    // ========================================================================
    // JALUR MODE 4: MOBIL / PERAHU RC PROPO, AUX & LAMPU BELAKANG/REM (D8)
    // ========================================================================
    if (rxData.auxFlag == 4) {

      // 1. Bebaskan Pin 5 & 6 dari library Servo untuk dipakai AUX1 & AUX2
      if (escsAttached) {
        esc1.detach(); esc2.detach();
        pinMode(PIN_MOTOR_1, OUTPUT);
        pinMode(PIN_MOTOR_2, OUTPUT);
        escsAttached = false;
      }

      // 2. KENDALI SETIR (EPA MIKRODETIK)
      int steerInput = STEER_REVERSE ? -rxData.aileron : rxData.aileron;
      int outS3 = STEER_CENTER_US;

      if (steerInput < 0) {
        outS3 = map(steerInput, -127, 0, STEER_MIN_US, STEER_CENTER_US);
      } else {
        outS3 = map(steerInput, 0, 127, STEER_CENTER_US, STEER_MAX_US);
      }
      outS3 = constrain(outS3, 1000, 2000);

      // 3. KENDALI ESC MAJU-MUNDUR (3 TAHAP: INPUT -> KURVA EXPO INTEGER -> MAPPING)
      bool escJalan = false;
      int outS4 = ESC_CENTER_US;

      if (rxData.isArmed == 1 && abs(rxData.elevator) >= ESC_DEADZONE) {
        escJalan = true;
        int elevIn = ESC_REVERSE ? -rxData.elevator : rxData.elevator;

        // Tahap 1: Hilangkan offset deadzone agar gerak efektif mulai halus dari 0 s/d maxMag
        const long maxMag = 127 - ESC_DEADZONE;
        long mag = constrain(abs(elevIn), ESC_DEADZONE, 127) - ESC_DEADZONE;
        long expoMag = mag;

        // Tahap 2: Pembentuk Kurva Expo Kubik Murni Integer (Aktif hanya jika ESC_EXPO_PERSEN > 0)
        #if ESC_EXPO_PERSEN > 0
          long cubic = (mag * mag * mag) / (maxMag * maxMag);
          expoMag = ((mag * (100 - ESC_EXPO_PERSEN)) + (cubic * ESC_EXPO_PERSEN)) / 100;
        #endif

        // Tahap 3: Petakan hasil kurva ke lebar pulsa mikrodetik ESC
        if (elevIn > 0) {
          outS4 = map(expoMag, 0, maxMag, ESC_CENTER_US, ESC_MAX_US);
        } else {
          outS4 = map(expoMag, 0, maxMag, ESC_CENTER_US, ESC_MIN_US);
        }
        outS4 = constrain(outS4, 1000, 2000);
      }

      // 3b. DETEKSI REM OTOMATIS (TRANSISI JALAN -> BERHENTI)
      if (escJalan) {
        remSedangNyala = false;
      } else if (escJalanSebelumnya) {
        remSedangNyala = true;
        waktuRemAktif = currentMillis;
      }
      escJalanSebelumnya = escJalan;

      if (remSedangNyala && (currentMillis - waktuRemAktif >= REM_DURASI_MS)) {
        remSedangNyala = false;
      }

      // 4. PEMBACAAN PEMICU AUX (STIK KIRI)
      bool pemicuAux2       = (rxData.throttle > 180); // Stik kiri atas -> AUX2 (Musik)
      bool resetAux2        = (rxData.throttle < 70);  // Stik kiri bawah -> Reset AUX2
      bool pemicuAux1Toggle = (rxData.rudder > 80);    // Stik kiri kanan -> Toggle AUX1 (Lampu)
      bool resetAux1Toggle  = (rxData.rudder < 30);    // Stik kiri tengah -> Reset Toggle AUX1
      bool pemicuAux1Kedip  = (rxData.rudder < -80);   // Stik kiri kiri -> Tahan Dim/Kedip AUX1

      // 5. EKSEKUSI AUX2 (PIN D6) - TETAP AKTIF SAAT DISARMED
      if (AUX2_MODE_PULSA) {
        if (pemicuAux2 && !kunciAux2) {
          kunciAux2 = true;
          sedangPulsaAux2 = true;
          waktuPulsaAux2 = currentMillis;
          digitalWrite(PIN_MOTOR_2, AUX2_AKTIF_LOW ? LOW : HIGH);
        } else if (resetAux2) {
          kunciAux2 = false;
        }

        if (sedangPulsaAux2 && (currentMillis - waktuPulsaAux2 >= AUX2_DURASI_MS)) {
          sedangPulsaAux2 = false;
          digitalWrite(PIN_MOTOR_2, AUX2_AKTIF_LOW ? HIGH : LOW);
        }
      } else {
        if (pemicuAux2)      digitalWrite(PIN_MOTOR_2, AUX2_AKTIF_LOW ? LOW : HIGH);
        else if (resetAux2)  digitalWrite(PIN_MOTOR_2, AUX2_AKTIF_LOW ? HIGH : LOW);
      }

      // 6. EKSEKUSI AUX1 (PIN D5: LAMPU DEPAN & DIM) - TETAP AKTIF SAAT DISARMED
      if (pemicuAux1Toggle && !kunciAux1Toggle) {
        aux1Nyala = !aux1Nyala;
        kunciAux1Toggle = true;
      } else if (resetAux1Toggle) {
        kunciAux1Toggle = false;
      }

      if (pemicuAux1Kedip) {
        if (currentMillis - waktuKedipAux1 >= AUX1_KEDIP_MS) {
          waktuKedipAux1 = currentMillis;
          statusKedipAux1 = !statusKedipAux1;
        }
        digitalWrite(PIN_MOTOR_1, statusKedipAux1 ? HIGH : LOW);
      } else {
        digitalWrite(PIN_MOTOR_1, aux1Nyala ? HIGH : LOW);
      }

      // 7. EKSEKUSI LAMPU BELAKANG & REM (PIN D8 - PRIORITAS: REM > LAMPU KOTA > MATI)
      if (remSedangNyala) {
        // Prioritas 1: Rem 1 detik -> 100% terang (aktif meski lampu depan mati)
        digitalWrite(PIN_BUZZER, HIGH);
      } else if (aux1Nyala) {
        // Prioritas 2: Ikut lampu depan -> Redup via Software PWM (~488 Hz, bebas konflik Timer)
        const uint16_t pwmBatas = (2048UL * LAMPU_KOTA_REDUP) / 100;
        digitalWrite(PIN_BUZZER, ((micros() & 2047) < pwmBatas) ? HIGH : LOW);
      } else {
        // Prioritas 3: Lampu depan mati & tidak mengerem -> Padam
        digitalWrite(PIN_BUZZER, LOW);
      }

      // 8. UPDATE SERVO SETIR & ESC MAJU-MUNDUR
      servo3.writeMicroseconds(outS3);

      if (escJalan) {
        if (!servo4.attached()) servo4.attach(PIN_SERVO_4);
        servo4.writeMicroseconds(outS4);
      } else {
        if (ESC_AUTO_DETACH) {
          if (servo4.attached()) {
            servo4.detach();
            pinMode(PIN_SERVO_4, OUTPUT);
            digitalWrite(PIN_SERVO_4, LOW);
          }
        } else {
          if (!servo4.attached()) servo4.attach(PIN_SERVO_4);
          servo4.writeMicroseconds(ESC_CENTER_US);
        }
      }

    }
    // ========================================================================
    // JALUR MODE 1, 2, DAN 3: PESAWAT RC (UTUH & TERISOLASI)
    // ========================================================================
    else {
      digitalWrite(PIN_BUZZER, LOW); // Pastikan buzzer diam saat pesawat terkendali normal

      // 1. Throttle Slew-Rate Limiter
      int targetThrottle = rxData.isArmed ? rxData.throttle : 0;

      if (targetThrottle > currentThrottle) {
        static unsigned long lastSlewTime = 0;
        const unsigned int SLEW_INTERVAL = 5;
        const int SLEW_STEP = 2;

        if (currentMillis - lastSlewTime >= SLEW_INTERVAL) {
          currentThrottle += SLEW_STEP;
          if (currentThrottle > targetThrottle) currentThrottle = targetThrottle;
          lastSlewTime = currentMillis;
        }
      } else {
        currentThrottle = targetThrottle;
      }

      int outM1 = 0, outM2 = 0;
      int outS3 = 90, outS4 = 90;

      // 2. Throttle Deadband Lock & Mixer Aerodinamika
      if (currentThrottle < 10) {
        outM1 = 0;
        outM2 = 0;
        outS3 = map(rxData.elevator, -127, 127, 45, 135);
        outS4 = map(rxData.aileron, -127, 127, 45, 135);
      } else {
        switch (rxData.auxFlag) {
          case 1: // NORMAL
            outM1 = currentThrottle;
            outM2 = currentThrottle;
            outS3 = map(rxData.elevator, -127, 127, 45, 135);
            outS4 = map(rxData.aileron, -127, 127, 45, 135);
            break;

          case 2: // DIFERENSIAL PROPORTIONAL (Expo + Kompensasi Gas Penuh + Anti-Stall)
            {
              long ailInput = rxData.aileron;
              int16_t ailExpo = (ailInput + ((ailInput * abs(ailInput)) / 127)) / 2;
              int16_t maxDiff = ((currentThrottle * 70) / 100) + 15;
              int16_t diff = (ailExpo * maxDiff) / 127;

              outM1 = currentThrottle + diff;
              outM2 = currentThrottle - diff;

              if (outM1 > 255) { outM2 -= (outM1 - 255); outM1 = 255; }
              if (outM2 > 255) { outM1 -= (outM2 - 255); outM2 = 255; }

              if (currentThrottle > 25) {
                if (outM1 < 15) outM1 = 15;
                if (outM2 < 15) outM2 = 15;
              }
              outS3 = 90;
              outS4 = 90;
            }
            break;

          case 3: // V-TAIL / ELEVON
            outM1 = currentThrottle;
            outM2 = currentThrottle;
            outS3 = map(rxData.elevator + rxData.aileron, -254, 254, 45, 135);
            outS4 = map(rxData.elevator - rxData.aileron, -254, 254, 45, 135);
            break;
        }
      }

      outM1 = constrain(outM1, 0, 255);
      outM2 = constrain(outM2, 0, 255);

      // 3. Distribusi Output Hybrid (ESC Brushless vs MOSFET PWM)
      if (rxData.motorType == 0) {
        if (!escsAttached) {
          esc1.attach(PIN_MOTOR_1); esc2.attach(PIN_MOTOR_2);
          escsAttached = true;
        }
        if (!rxData.isArmed) {
          esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000);
        } else {
          esc1.writeMicroseconds(map(outM1, 0, 255, 1000, 2000));
          esc2.writeMicroseconds(map(outM2, 0, 255, 1000, 2000));
        }
      } else {
        if (escsAttached) {
          esc1.detach(); esc2.detach();
          pinMode(PIN_MOTOR_1, OUTPUT); pinMode(PIN_MOTOR_2, OUTPUT);
          escsAttached = false;
        }
        analogWrite(PIN_MOTOR_1, outM1);
        analogWrite(PIN_MOTOR_2, outM2);
      }

      // 4. Update Servo Sayap Pesawat
      if (!servo4.attached()) servo4.attach(PIN_SERVO_4);
      servo3.write(constrain(outS3, 45, 135));
      servo4.write(constrain(outS4, 45, 135));
    }
  }

  // --- D. SMART LED HEARTBEAT (INDIKATOR STATUS REAL-TIME) ---
  int cycle = currentMillis % 1000;
  if (failsafeActive) {
    digitalWrite(PIN_LED, (cycle < 500) ? HIGH : LOW); // Kedip santai (Failsafe)
  } else if (rxData.isArmed) {
    digitalWrite(PIN_LED, (cycle < 50) ? LOW : HIGH);  // Nyala terang, kedut mati (Armed)
  } else {
    digitalWrite(PIN_LED, (cycle < 50) ? HIGH : LOW);  // Mati, kedut nyala (Disarmed)
  }
}