/*
 * ============================================================================
 * FIRMWARE TRANSMITTER (TX) - V930 RC UNIVERSAL HYBRID (PRO MINI)
 * ============================================================================
 * Spesifikasi Hardware & Kelistrikan:
 * - Mikrokontroler : Arduino Pro Mini (ATmega168P/328P - 3.3V 8MHz)
 * - Tegangan VCC   : 3.3V (Wajib, bisa dari LDO bawaan board V930)
 * - Potensio Stik  : 3.3V (Pin A0, A1, A2, A3)
 * - Tombol Trim    : 3.3V Resistor Ladder 8 Tombol (Pin A4)
 * - Sensor Baterai : Pin A6 (Baterai TX) & Pin A7 (Status Charger TP4056)
 * - Modul NRF24L01 : 3.3V (CE = Pin 9, CSN = Pin 10)
 * - Tombol L & R   : Isolasi Jalur, Menggunakan Pull-Up Internal (Pin 4 & 5)
 * - Indikator      : LED Hijau (Pin 3), LED Merah (Pin 2), Buzzer (Pin 8)
 * 
 * ============================================================================
 * PANDUAN PENGGUNAAN & FITUR TX:
 * ============================================================================
 * [TOMBOL R] - ARMING / DISARMING (Menghidupkan / Mematikan Motor):
 * - Saat di Darat (Gas Nol) : Tekan 1x untuk Arm/Disarm cepat.
 * - Saat di Udara (Gas Naik): Tolak Arming. Wajib tahan 0.6 detik untuk 
 *                             Emergency Disarm.
 * 
 * [TOMBOL L] - MENU KOMBINASI DARAT (Harus kondisi Disarmed):
 * - Tahan L + Trim Aileron Kanan = Pindah Slot Memori Kendaraan (1-10)
 * - Tahan L + Trim Elevator Bawah= Ganti Tipe Motor (ESC Brushless / MOSFET)
 * - Tahan L + Trim Elevator Atas = Ganti Mode (1:Normal / 2:Glider / 3:Jet / 4:Mobil)
 * - Tahan L + Trim Aileron Kiri  = Simpan Pengaturan (Save EEPROM)
 * 
 * [TOMBOL L] - RESET TRIM INSTAN:
 * - Saat di Darat (Disarmed): Klik L dengan cepat (< 400ms).
 * - Saat di Udara (Armed)   : Klik L kapan saja, semua trim langsung kembali nol.
 * 
 * [TOMBOL L + R] - MODE BINDING (10 DETIK):
 * - Tahan L dan R bersamaan saat menyalakan remote untuk menjodohkan RX baru.
 * ============================================================================
 */

#include <Arduino.h>    // Library utama fungsi dasar Arduino
#include <SPI.h>        // Library komunikasi protokol SPI untuk modul radio
#include <RF24.h>       // Library pengendali modul transceiver NRF24L01
#include <EEPROM.h>     // Library penyimpanan memori permanen di dalam chip
#include "lcd_h8805a.h" // Library khusus untuk mengontrol layar LCD V930

// ============================================================================
// 0. SAKLAR MASTER DEBUGGING SERIAL (1 = NYALA, 0 = MATI TOTAL SAAT FINAL)
// ============================================================================
#define AKTIFKAN_SERIAL 0  // Ubah ke 1 saat debug di meja, ubah ke 0 saat terbang/jalan!

#if AKTIFKAN_SERIAL == 0
  // Menghapus total semua perintah Serial dari kompilasi (0 byte RAM/Flash & 0 interupsi)
  #define Serial if(false)Serial
#endif

// ============================================================================
// 1. DEFINISI PIN HARDWARE ARDUINO PRO MINI
// ============================================================================
#define PIN_RADIO_CE   9  // Pin Chip Enable NRF24L01
#define PIN_RADIO_CSN  10 // Pin Chip Select Not SPI NRF24L01
#define PIN_LED_HIJAU  3  // Pin output LED Hijau (Indikator ARMED / Mesin Hidup)
#define PIN_LED_MERAH  2  // Pin output LED Merah (Indikator DISARM / Mesin Terkunci)
#define PIN_BTN_L      4  // Pin input Tombol Bahu Kiri (Menu Konfigurasi & Reset Trim)
#define PIN_BTN_R      5  // Pin input Tombol Bahu Kanan (Saklar Arming / Disarm)
#define PIN_BUZZER     8  // Pin output Buzzer untuk alarm dan nada notifikasi

const uint8_t PIN_THR    = A2; // Stik Gas (Throttle)
const uint8_t PIN_RUD    = A3; // Stik Belok Ekor (Rudder)
const uint8_t PIN_ELE    = A1; // Stik Naik/Turun (Elevator)
const uint8_t PIN_AIL    = A0; // Stik Kemiringan Sayap (Aileron)
const uint8_t PIN_TRIM   = A4; // Rangkaian 8 Tombol Trim (Resistor Ladder)
const uint8_t PIN_BAT_TX = A6; // Pembaca tegangan baterai internal Remote (TX)
const uint8_t PIN_CHRG   = A7; // Sensor IC Charger TP4056 (Wajib resistor pull-up 10k ke 3.3V)

const byte BIND_ADDR[6]  = "BIND0"; // Alamat darurat statis khusus proses Binding
const uint8_t BIND_CH    = 10;      // Channel darurat khusus proses Binding
const uint8_t MAX_MODELS = 10;      // Kapasitas maksimal slot memori di EEPROM

// ============================================================================
// 2. STRUKTUR DATA KOMUNIKASI RADIO (WAJIB 100% IDENTIK DENGAN RX)
// ============================================================================
// Paket Data Binding (Hanya dipakai 10 detik pertama saat menahan tombol L+R)
struct BindPayload {
  byte pipeAddr[5];  // Alamat rahasia 5 byte untuk RX
  uint8_t opChannel; // Angka bibit channel FHSS (20-94)
  uint8_t checkSum;  // Verifikasi XOR agar RX tidak menangkap sinyal nyasar
};

// Paket Data Kendali Utama (Dikirim 33x per detik ke RX)
struct ControlData {
  uint8_t throttle;  // Nilai gas + trim (0 s/d 255)
  int8_t  rudder;    // Nilai rudder + trim (-127 s/d 127)
  int8_t  elevator;  // Nilai elevator + trim (-127 s/d 127)
  int8_t  aileron;   // Nilai aileron + trim (-127 s/d 127)
  uint8_t isArmed;   // 0 = Disarmed (Motor Terkunci), 1 = Armed (Siap Jalan/Terbang)
  uint8_t auxFlag;   // Mode: 1=Normal, 2=Diferensial, 3=V-Tail, 4=Mobil RC Propo
  uint8_t motorType; // Mode Pesawat: 0 = ESC Brushless, 1 = MOSFET PWM
};
ControlData txData;

// Paket Data Telemetri (Diterima balik dari RX via AckPayload)
struct TelemetryData {
  uint8_t batteryStatus;  // 0 = Normal, 1 = Baterai RX Lemah (Low Batt)
  uint8_t failsafeActive; // 0 = Terkendali, 1 = RX sedang Failsafe
};
TelemetryData rxTelem;

// ============================================================================
// 3. DEKLARASI OBJEK & VARIABEL GLOBAL SISTEM
// ============================================================================
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);

uint8_t currentSlot     = 1;  // Slot memori aktif (1 sampai 10)
byte activePipe[5];           // Alamat komunikasi slot aktif
uint8_t activeChannel   = 76; // Bibit channel slot aktif
uint8_t activeMode      = 1;  // Mode kendaraan aktif (1-4)
uint8_t activeMotorType = 0;  // Tipe motor aktif (0=ESC, 1=MOSFET)

// --- Sistem Lompatan Frekuensi (FHSS 4-Channel, Identik dengan RX) ---
const uint8_t FHSS_LEN = 4;
uint8_t fhssChannels[FHSS_LEN];
uint8_t hopIndex = 0;

// Meracik 4 saluran FHSS dari 1 angka bibit (Loncat 23 channel di rentang Ch 20-99)
void generateFHSSChannels(uint8_t seed) {
  for (uint8_t i = 0; i < FHSS_LEN; i++) {
    fhssChannels[i] = 20 + ((seed - 20 + (i * 23)) % 80);
  }
  hopIndex = 0;
}

// Memori Geseran Trim (-127 s/d 127)
int throttleTrim = 0;
int rudderTrim   = 0;
int elevatorTrim = 0;
int aileronTrim  = 0;

unsigned long lastTxTime  = 0;
const unsigned long TX_INTERVAL = 30; // 30ms = 33 paket per detik
unsigned long lastAckTime = 0;        // Waktu terakhir RX membalas ACK

bool isArmed = false; // Wajib false (Terkunci) saat remote baru dinyalakan

// --- Manajemen Buzzer Non-Blocking ---
bool buzzerActive         = false;
uint8_t beepState         = 0;
unsigned long beepTimer   = 0;
unsigned int beepInterval = 0;

// Opsi Hardware Buzzer:
const bool PAKE_BUZZER_PASIF = true; // Ubah ke false jika memakai buzzer aktif (DC biasa)
const int FREKUENSI_BUZZER   = 2500; // Nada buzzer (khusus buzzer pasif)

// ============================================================================
// 4. FUNGSI KENDALI AUDIO (BUZZER) & VISUAL (LED MERAH/HIJAU)
// ============================================================================
void nyalakanBuzzer(bool nyala) {
  if (nyala) {
    if (PAKE_BUZZER_PASIF) tone(PIN_BUZZER, FREKUENSI_BUZZER);
    else digitalWrite(PIN_BUZZER, HIGH);
  } else {
    if (PAKE_BUZZER_PASIF) noTone(PIN_BUZZER);
    else digitalWrite(PIN_BUZZER, LOW);
  }
}

// Bip tunggal non-blocking (aman dipanggil saat terbang/jalan)
void beep(unsigned int duration) {
  if (beepState != 0) return;
  beepState = 1;
  beepInterval = duration;
  beepTimer = millis();
  buzzerActive = true;
}

// Bip ganda non-blocking (notifikasi ganti menu)
void beepDouble() {
  if (beepState != 0) return;
  beepState = 3;
  beepInterval = 100;
  beepTimer = millis();
  buzzerActive = true;
}

// Pemroses buzzer & LED di latar belakang (dipanggil tiap putaran loop)
void handleAudioVisual() {
  unsigned long m = millis();
  bool isBuzzerOn = false;

  if (beepState > 0) {
    if (beepState == 1 || beepState == 3) {
      nyalakanBuzzer(true);
      isBuzzerOn = true;
    } else {
      nyalakanBuzzer(false);
      isBuzzerOn = false;
    }
    
    if (m - beepTimer >= beepInterval) {
      if (beepState == 1) {
        nyalakanBuzzer(false);
        beepState = 0;
        buzzerActive = false;
        isBuzzerOn = false;
      } else if (beepState == 3) {
        nyalakanBuzzer(false);
        beepState = 2;
        beepInterval = 80;
        beepTimer = m;
        isBuzzerOn = false;
      } else if (beepState == 2) {
        beepState = 1;
        beepInterval = 100;
        beepTimer = m;
      }
    }
  } else {
    buzzerActive = false;
  }

  // Indikator LED silang mengikuti status Armed & Buzzer
  if (isArmed) {
    digitalWrite(PIN_LED_HIJAU, isBuzzerOn ? LOW : HIGH);
    digitalWrite(PIN_LED_MERAH, isBuzzerOn ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_HIJAU, LOW);
    digitalWrite(PIN_LED_MERAH, isBuzzerOn ? LOW : HIGH);
  }
}

// Bip blocking (hanya aman dipanggil saat setup / kondisi darat Disarmed)
void beepBlocking(unsigned int duration) {
  nyalakanBuzzer(true);
  digitalWrite(PIN_LED_MERAH, LOW);
  delay(duration);
  nyalakanBuzzer(false);
  digitalWrite(PIN_LED_MERAH, HIGH);
}

// ============================================================================
// 5. FUNGSI MANAJEMEN MEMORI EEPROM (10 SLOT KENDARAAN)
// ============================================================================
int getSlotOffset(uint8_t slot) {
  return 100 + ((slot - 1) * 15); // Slot 1 mulai di alamat 100, lebar 15 byte per slot
}

void saveConfigToSlot() {
  int offset = getSlotOffset(currentSlot);
  EEPROM.update(offset + 7,  activeMode);
  EEPROM.update(offset + 8,  (uint8_t)(throttleTrim + 128)); // +128 agar nilai negatif bisa disimpan
  EEPROM.update(offset + 9,  (uint8_t)(rudderTrim + 128));
  EEPROM.update(offset + 10, (uint8_t)(elevatorTrim + 128));
  EEPROM.update(offset + 11, (uint8_t)(aileronTrim + 128));
  EEPROM.update(offset + 12, activeMotorType);

  beepBlocking(100); delay(150);
  beepBlocking(100); delay(150);
  beepBlocking(300);

  Serial.print(F("[EEPROM] Konfigurasi Berhasil Disimpan ke Slot: "));
  Serial.println(currentSlot);
}

void loadModel(uint8_t slot) {
  int offset = getSlotOffset(slot);

  // Jika segel 0xAA belum ada, format slot baru dengan alamat & channel acak
  if (EEPROM.read(offset) != 0xAA) {
    Serial.print(F("[EEPROM] Memformat Slot Baru: "));
    Serial.println(slot);
    EEPROM.update(offset, 0xAA);
    EEPROM.update(offset + 1, random(20, 95));
    for (int i = 0; i < 5; i++) {
      EEPROM.update(offset + 2 + i, 'A' + random(0, 26));
    }
    EEPROM.update(offset + 7,  1);   // Default: Mode 1 (Normal)
    EEPROM.update(offset + 8,  128); // Trim 0
    EEPROM.update(offset + 9,  128);
    EEPROM.update(offset + 10, 128);
    EEPROM.update(offset + 11, 128);
    EEPROM.update(offset + 12, 0);   // Default: ESC Brushless
  }

  activeChannel = EEPROM.read(offset + 1);
  for (int i = 0; i < 5; i++) {
    activePipe[i] = EEPROM.read(offset + 2 + i);
  }
  generateFHSSChannels(activeChannel);

  activeMode      = EEPROM.read(offset + 7);
  throttleTrim    = (int)EEPROM.read(offset + 8) - 128;
  rudderTrim      = (int)EEPROM.read(offset + 9) - 128;
  elevatorTrim    = (int)EEPROM.read(offset + 10) - 128;
  aileronTrim     = (int)EEPROM.read(offset + 11) - 128;
  activeMotorType = EEPROM.read(offset + 12);

  EEPROM.update(50, slot); // Simpan slot terakhir yang dipakai di alamat 50
  currentSlot = slot;

  // Bip sejumlah nomor slot aktif
  for (int i = 0; i < slot; i++) {
    beepBlocking(100);
    delay(200);
  }

  Serial.print(F("[EEPROM] Berhasil Load Slot: ")); Serial.print(slot);
  Serial.print(F(" | Ch: ")); Serial.print(activeChannel);
  Serial.print(F(" | Mode: ")); Serial.print(activeMode);
  Serial.print(F(" | Motor: ")); Serial.println(activeMotorType == 0 ? F("ESC") : F("MOSFET"));
  Serial.print(F("[FHSS] Daftar 4 Channel Aktif: "));
  for (uint8_t i = 0; i < FHSS_LEN; i++) {
    Serial.print(fhssChannels[i]);
    Serial.print(F(" "));
  }
  Serial.println();
}

// ============================================================================
// 6. FUNGSI PROSES BINDING (PANCARAN 10 DETIK KE RX BARU)
// ============================================================================
void executeBindingProcess() {
  Serial.println(F("[BIND] Memulai Proses Pancaran Binding 10 Detik..."));
  radio.stopListening();
  radio.setChannel(BIND_CH);
  radio.openWritingPipe(BIND_ADDR);

  BindPayload bp;
  memcpy(bp.pipeAddr, activePipe, 5);
  bp.opChannel = activeChannel;
  bp.checkSum  = activeChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4];

  unsigned long bindStartTime = millis();
  while (millis() - bindStartTime < 10000) {
    radio.write(&bp, sizeof(BindPayload));

    nyalakanBuzzer(true);
    digitalWrite(PIN_LED_MERAH, HIGH);
    digitalWrite(PIN_LED_HIJAU, LOW);
    delay(40);

    nyalakanBuzzer(false);
    digitalWrite(PIN_LED_MERAH, LOW);
    digitalWrite(PIN_LED_HIJAU, HIGH);
    delay(80);
  }

  digitalWrite(PIN_LED_HIJAU, LOW);
  beepBlocking(500);
  Serial.println(F("[BIND] Selesai! Kembali ke Frekuensi Operasional."));
}

// ============================================================================
// 7. KONVERSI STIK ANALOG KE DIGITAL (-127 s/d +127 DENGAN DEADZONE TENGAH)
// ============================================================================
int mapJoystick(int val, int lower, int middle, int upper) {
  val = constrain(val, lower, upper);
  int output = 0;

  if (val < middle - 15) {
    output = map(val, lower, middle - 15, -127, 0);
  } else if (val > middle + 15) {
    output = map(val, middle + 15, upper, 0, 127);
  }

  return output;
}

// ============================================================================
// 8. FUNGSI UTAMA: SETUP (DIJALANKAN 1x SAAT REMOTE DINYALAKAN)
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED_HIJAU, OUTPUT);
  pinMode(PIN_LED_MERAH, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BTN_L, INPUT_PULLUP);
  pinMode(PIN_BTN_R, INPUT_PULLUP);

  nyalakanBuzzer(false);
  digitalWrite(PIN_LED_MERAH, HIGH);
  digitalWrite(PIN_LED_HIJAU, LOW);

  delay(50); // Tunggu tegangan pull-up tombol L & R stabil

  // Ambil bibit angka acak dari variasi pembacaan ADC pada pin baterai (A6) & charger (A7)
  randomSeed(analogRead(PIN_BAT_TX) + analogRead(PIN_CHRG));

  // Muat slot terakhir yang dipakai dari EEPROM
  uint8_t lastSlot = EEPROM.read(50);
  if (lastSlot < 1 || lastSlot > MAX_MODELS) lastSlot = 1;
  loadModel(lastSlot);

  // Inisialisasi NRF24L01
  if (!radio.begin()) {
    Serial.println(F("[FATAL] Modul NRF24L01 Tidak Terdeteksi! Cek Kabel SPI & Power 3.3V!"));
    while (1) {
      digitalWrite(PIN_LED_MERAH, !digitalRead(PIN_LED_MERAH));
      delay(100);
    }
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setRetries(1, 3); // Tunggu 500us, ulang maksimal 3x jika gagal ACK
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  // Jika tombol L & R ditahan bersamaan saat power ON -> Masuk Mode Binding 10 detik
  if (digitalRead(PIN_BTN_L) == LOW && digitalRead(PIN_BTN_R) == LOW) {
    executeBindingProcess();
  }

  // Masuk ke saluran FHSS pertama
  radio.stopListening();
  hopIndex = 0;
  radio.setChannel(fhssChannels[hopIndex]);
  radio.openWritingPipe(activePipe);
  delay(10);

  isArmed = false;
  lastAckTime = millis();
  beepBlocking(200);
  initLCD();
  Serial.println(F("[SYSTEM] TX Siap Tempur. Kondisi: DISARMED."));
}

// ============================================================================
// 9. FUNGSI UTAMA: LOOP (EKSEKUSI REAL-TIME)
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();
  handleAudioVisual();

  // --- A. PERINGATAN TELEMETRI BATERAI RX LEMAH ---
  static unsigned long lastAlarmTime = 0;
  if (isArmed && rxTelem.batteryStatus == 1) {
    if (currentMillis - lastAlarmTime > 2500) {
      beep(500);
      lastAlarmTime = currentMillis;
    }
  }

  // --- B. SIKLUS UTAMA KENDALI & PENGIRIMAN RADIO (TIAP 30ms / 33 Hz) ---
  if (currentMillis - lastTxTime >= TX_INTERVAL) {
    lastTxTime = currentMillis;

    // 1. Baca semua potensio stik & tombol trim
    int rawThr  = analogRead(PIN_THR);
    int rawRud  = analogRead(PIN_RUD);
    int rawEle  = analogRead(PIN_ELE);
    int rawAil  = analogRead(PIN_AIL);
    int rawTrim = analogRead(PIN_TRIM);

    bool btnL = (digitalRead(PIN_BTN_L) == LOW);
    bool btnR = (digitalRead(PIN_BTN_R) == LOW);

    // --- C. LOGIKA TOMBOL R (ARMING / DISARMING / EMERGENCY DISARM) ---
    static unsigned long btnRPressTime = 0;
    static bool btnRWasPressed = false;
    static bool disarmHandled  = false;

    if (btnR && !btnL) {
      if (!btnRWasPressed) {
        btnRPressTime  = currentMillis;
        btnRWasPressed = true;
        disarmHandled  = false;

        if (!isArmed) {
          if (rawThr > 40) {
            // Tolak Arming jika tuas gas tidak di posisi paling bawah
            beepBlocking(80); delay(80); beepBlocking(80);
            Serial.println(F("[WARNING] Ditolak! Turunkan Tuas Gas Dulu!"));
          } else {
            isArmed = true;
            beep(600);
            Serial.println(F("[MOTOR] ARMED!"));
          }
          disarmHandled = true;
        } else if (isArmed && rawThr <= 40) {
          isArmed = false;
          beep(200);
          Serial.println(F("[MOTOR] DISARMED (Klik Instan Darat)"));
          disarmHandled = true;
        }
      }

      // Emergency Disarm saat gas naik: wajib tahan 0.6 detik
      if (isArmed && !disarmHandled && (currentMillis - btnRPressTime > 600)) {
        isArmed = false;
        beep(400);
        Serial.println(F("[MOTOR] EMERGENCY DISARM! (Tahan 0.6s)"));
        disarmHandled = true;
      }
    } else if (!btnR) {
      btnRWasPressed = false;
    }

    // --- D. LOGIKA TOMBOL L (RESET TRIM & MENU KONFIGURASI DARAT) ---
    static bool btnLWasPressed = false;
    static unsigned long btnLPressTime = 0;
    static unsigned long lastMenuTime  = 0;
    static bool menuExecuted = false;
    const unsigned long SHORT_CLICK_MAX = 400;

    if (btnL && !btnR) {
      if (!btnLWasPressed) {
        btnLWasPressed = true;
        btnLPressTime  = currentMillis;
        menuExecuted   = false;

        if (isArmed) {
          // Reset trim instan saat sedang Armed
          throttleTrim = 0; rudderTrim = 0; elevatorTrim = 0; aileronTrim = 0;
          beep(400);
          Serial.println(F("[TRIM] Reset Instan saat Armed!"));
          menuExecuted = true;
        }
      }

      // Menu kombinasi darat (Tahan L + Tombol Trim)
      if (!isArmed && !menuExecuted) {
        if (rawTrim < 900 && (currentMillis - lastMenuTime > 500)) {
          if (rawTrim > 460 && rawTrim < 530) { // Aileron Kanan -> Ganti Slot (1-10)
            currentSlot = (currentSlot % MAX_MODELS) + 1;
            loadModel(currentSlot);
            radio.setChannel(fhssChannels[hopIndex]);
            radio.openWritingPipe(activePipe);
            triggerAnimSlot(currentSlot);
            menuExecuted = true; lastMenuTime = currentMillis;
          } else if (rawTrim > 380 && rawTrim < 450) { // Elevator Bawah -> Ganti ESC/MOSFET
            activeMotorType = (activeMotorType == 0) ? 1 : 0;
            triggerAnimMotor(activeMotorType);
            if (activeMotorType == 0) beep(150);
            else beepDouble();
            menuExecuted = true; lastMenuTime = currentMillis;
          } else if (rawTrim > 300 && rawTrim < 360) { // Elevator Atas -> Ganti Mode (1-4)
            activeMode = (activeMode % 4) + 1;
            beepBlocking(400); delay(200);
            for (int i = 0; i < activeMode; i++) { beepBlocking(100); delay(150); }
            menuExecuted = true; lastMenuTime = currentMillis;
          } else if (rawTrim > 540 && rawTrim < 610) { // Aileron Kiri -> Simpan EEPROM
            saveConfigToSlot();
            menuExecuted = true; lastMenuTime = currentMillis;
          }
        }
      }
    } else if (!btnL) {
      if (btnLWasPressed) {
        if (!isArmed && !menuExecuted) {
          if (currentMillis - btnLPressTime < SHORT_CLICK_MAX) {
            throttleTrim = 0; rudderTrim = 0; elevatorTrim = 0; aileronTrim = 0;
            beep(400);
            Serial.println(F("[TRIM] Reset (Klik Pendek di Darat)!"));
          }
        }
        btnLWasPressed = false;
      }
    }

    // --- E. PEMBACAAN 8 TOMBOL TRIM (RESISTOR LADDER) ---
    static unsigned long lastTrimProcess = 0;
    if (!btnL && rawTrim < 900 && (currentMillis - lastTrimProcess > 150)) {
      bool trimChanged = false;

      if (rawTrim < 40) {
        throttleTrim = constrain(throttleTrim + 2, -127, 127); trimChanged = true;
      } else if (rawTrim > 80 && rawTrim < 130) {
        throttleTrim = constrain(throttleTrim - 2, -127, 127); trimChanged = true;
      } else if (rawTrim > 140 && rawTrim < 200) {
        rudderTrim = constrain(rudderTrim - 2, -127, 127);     trimChanged = true;
      } else if (rawTrim > 230 && rawTrim < 290) {
        rudderTrim = constrain(rudderTrim + 2, -127, 127);     trimChanged = true;
      } else if (rawTrim > 300 && rawTrim < 360) {
        elevatorTrim = constrain(elevatorTrim + 2, -127, 127); trimChanged = true;
      } else if (rawTrim > 380 && rawTrim < 450) {
        elevatorTrim = constrain(elevatorTrim - 2, -127, 127); trimChanged = true;
      } else if (rawTrim > 460 && rawTrim < 530) {
        aileronTrim = constrain(aileronTrim + 2, -127, 127);   trimChanged = true;
      } else if (rawTrim > 540 && rawTrim < 610) {
        aileronTrim = constrain(aileronTrim - 2, -127, 127);   trimChanged = true;
      }

      if (trimChanged) {
        beep(30);
        lastTrimProcess = currentMillis;
      }
    }

    // --- F. KALKULASI STIK & PENYUSUNAN PAKET DATA ---
    int thrMurni = map(rawThr, 15, 680, 0, 255);
    if (rawThr <= 20) thrMurni = 0;
    thrMurni = constrain(thrMurni, 0, 255);
    int thrFinal = constrain(thrMurni + throttleTrim, 0, 255);

    int rudMurni = mapJoystick(rawRud, 11, 504, 1000);
    int eleMurni = mapJoystick(rawEle, 8,  515, 1000);
    int ailMurni = mapJoystick(rawAil, 10, 513, 1012);

    int rudFinal = constrain(rudMurni + rudderTrim, -127, 127);
    int eleFinal = constrain(eleMurni + elevatorTrim, -127, 127);
    int ailFinal = constrain(ailMurni - aileronTrim, -127, 127);

    txData.isArmed   = isArmed ? 1 : 0;
    txData.auxFlag   = activeMode;
    txData.motorType = activeMotorType;

    // Mode 1-3 (Pesawat): Kunci throttle ke 0 saat Disarmed demi keselamatan
    // Mode 4 (Mobil RC) : Izinkan throttle lewat saat Disarmed agar pemicu AUX2 tetap berfungsi
    txData.throttle = (isArmed || activeMode == 4) ? (uint8_t)constrain(thrFinal, 0, 255) : 0;
    txData.rudder   = (int8_t)rudFinal;
    txData.elevator = (int8_t)eleFinal;
    txData.aileron  = (int8_t)ailFinal;

    // --- G. PENGIRIMAN RADIO & LOMPATAN FHSS ---
    if (radio.write(&txData, sizeof(ControlData))) {
      lastAckTime = currentMillis;
      if (radio.isAckPayloadAvailable()) {
        radio.read(&rxTelem, sizeof(TelemetryData));
      }
    } else {
      radio.flush_tx(); // Bersihkan antrean FIFO jika paket gagal di channel ini
    }

    // Lompat ke channel FHSS berikutnya (chip punya waktu 30ms untuk PLL Lock)
    hopIndex = (hopIndex + 1) % FHSS_LEN;
    radio.setChannel(fhssChannels[hopIndex]);

    // Status koneksi RX (toleransi 1 detik tanpa balasan ACK)
    bool isRxConnected = (currentMillis - lastAckTime < 1000);
    if (!isRxConnected) {
      rxTelem.batteryStatus = 0;
    }

    // --- PEMANTAUAN BATERAI INTERNAL TX & STATUS CHARGING TP4056 ---
    analogRead(PIN_BAT_TX); // Baca buang untuk hilangkan crosstalk ADC
    int rawBatTx = analogRead(PIN_BAT_TX);
    int txBatLevel = 0;

    if (rawBatTx > 600)      txBatLevel = 4; // Penuh
    else if (rawBatTx > 570) txBatLevel = 3;
    else if (rawBatTx > 540) txBatLevel = 2;
    else if (rawBatTx > 515) txBatLevel = 1; // Rendah
    else                     txBatLevel = 0; // Kritis

    static bool alarmBatreNyangkut = false;
    if (txBatLevel == 0 && !buzzerActive) {
      unsigned long critTime = currentMillis % 1000;
      if ((critTime < 100) || (critTime > 200 && critTime < 300)) {
        nyalakanBuzzer(true);
        alarmBatreNyangkut = true;
      } else {
        nyalakanBuzzer(false);
        alarmBatreNyangkut = false;
      }
    } else if (alarmBatreNyangkut && txBatLevel > 0) {
      nyalakanBuzzer(false);
      alarmBatreNyangkut = false;
    }

    bool isCharging = (analogRead(PIN_CHRG) < 500);

    // Perbarui tampilan layar LCD V930
    updateLCD(isArmed, thrMurni, txBatLevel, isRxConnected,
              activeMode, activeMotorType,
              throttleTrim, rudderTrim, elevatorTrim, aileronTrim,
              rudMurni, eleMurni, ailMurni, isCharging);

    // --- H. SERIAL MONITOR DEBUG STIK (HANYA DI-COMPILE SAAT AKTIFKAN_SERIAL == 1) ---
    #if AKTIFKAN_SERIAL == 1
      static int lastThr = 0, lastRud = 0, lastEle = 0, lastAil = 0, lastTrmVal = 0;
      static bool lastArmStatus = false;
      bool harusCetak = false;

      if (abs(txData.throttle - lastThr) > 2) harusCetak = true;
      if (abs(txData.rudder - lastRud) > 2)   harusCetak = true;
      if (abs(txData.elevator - lastEle) > 2) harusCetak = true;
      if (abs(txData.aileron - lastAil) > 2)  harusCetak = true;
      if (isArmed != lastArmStatus)           harusCetak = true;
      if (rawTrim < 900 && abs(rawTrim - lastTrmVal) > 10) harusCetak = true;

      if (harusCetak) {
        Serial.print(F("Arm:")); Serial.print(isArmed);
        Serial.print(F(" | Thr:")); Serial.print(txData.throttle);
        Serial.print(F(" Rud:")); Serial.print(txData.rudder);
        Serial.print(F(" Ele:")); Serial.print(txData.elevator);
        Serial.print(F(" Ail:")); Serial.print(txData.aileron);
        Serial.print(F(" | BatRX:")); Serial.println(rxTelem.batteryStatus);

        lastThr = txData.throttle; lastRud = txData.rudder;
        lastEle = txData.elevator; lastAil = txData.aileron;
        lastArmStatus = isArmed;   lastTrmVal = rawTrim;
      }
    #endif
  }
}