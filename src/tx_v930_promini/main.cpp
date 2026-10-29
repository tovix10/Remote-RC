/*
 * ============================================================================
 * FIRMWARE TRANSMITTER (TX) - V930 RC UNIVERSAL HYBRID (PRO MINI 168P)
 * ============================================================================
 * Spesifikasi Hardware & Kelistrikan:
 * - Mikrokontroler : Arduino Pro Mini (ATmega168P - Modif Kristal 8MHz)
 * - Tegangan VCC   : 3.3V Langsung dari LDO Bawaan Board V930
 * - Potensio Stik  : 3.3V (Pin A0, A1, A2, A3)
 * - Tombol Trim    : 3.3V Resistor Ladder 8 Tombol Bawaan V930 (Pin A4)
 * - Modul NRF24L01 : 3.3V Bawaan V930 (CE = Pin 9, CSN = Pin 10)
 * - Tombol L & R   : Isolasi Jalur, Menggunakan Pull-Up Internal (Pin 4 & 5)
 * - Indikator      : LED Hijau (Pin 2), LED Merah (Pin 3), Buzzer (Pin 8)
 * ============================================================================
 */

#include <Arduino.h> // Library utama fungsi dasar Arduino
#include <SPI.h>     // Library komunikasi protokol SPI untuk modul radio
#include <RF24.h>    // Library pengendali modul transceiver NRF24L01
#include <EEPROM.h>  // Library penyimpanan memori permanen di dalam chip Nano
#include "lcd_h8805a.h"

// ============================================================================
// 1. DEFINISI PIN HARDWARE ARDUINO NANO
// ============================================================================
#define PIN_RADIO_CE   9  // Pin Chip Enable untuk mengaktifkan mode kirim/terima NRF24
#define PIN_RADIO_CSN  10 // Pin Chip Select Not untuk jalur seleksi komunikasi SPI NRF24
#define PIN_LED_HIJAU  3  // Pin output LED Hijau (Indikator ARMED / Mesin Hidup)
#define PIN_LED_MERAH  2  // Pin output LED Merah (Indikator DISARM / Mesin Terkunci & Notifikasi)
#define PIN_BTN_L      4  // Pin input Tombol Bahu Kiri (Menu Konfigurasi & Reset Trim)
#define PIN_BTN_R      5  // Pin input Tombol Bahu Kanan (Saklar Arming / Disarm)
#define PIN_BUZZER     8  // Pin output Buzzer aktif untuk alarm dan nada notifikasi

const uint8_t PIN_THR  = A2; // Pin input analog Stik Gas (Throttle)
const uint8_t PIN_RUD  = A3; // Pin input analog Stik Belok Ekor (Rudder)
const uint8_t PIN_ELE  = A1; // Pin input analog Stik Naik/Turun (Elevator)
const uint8_t PIN_AIL  = A0; // Pin input analog Stik Kemiringan Sayap (Aileron)
const uint8_t PIN_TRIM = A4; // Pin input analog Rangkaian 8 Tombol Trim (Resistor Ladder)
const uint8_t PIN_BAT_TX = A6; // Pin analog pembaca tegangan baterai TX (via Resistor Divider)
const uint8_t PIN_CHRG   = A7; // Pin sensor TP4056 (Wajib ada resistor pull-up 10k ke 3.3V)

const byte BIND_ADDR[6]  = "BIND0"; // Alamat pipa khusus saat proses Binding
const uint8_t BIND_CH    = 10;      // Frekuensi channel khusus saat proses Binding
const uint8_t MAX_MODELS = 10;      // Jumlah maksimal slot memori pesawat di EEPROM

// ============================================================================
// 2. STRUKTUR DATA KOMUNIKASI RADIO (WAJIB SAMA PERSIS DENGAN RX)
// ============================================================================
// Struktur paket data untuk mengirim identitas radio saat proses Binding
struct BindPayload {
  byte pipeAddr[5];  // 5 byte alamat pipa unik milik slot yang aktif
  uint8_t opChannel; // 1 byte frekuensi channel operasional milik slot yang aktif
  uint8_t checkSum;  // 1 byte kode validasi keamanan agar tidak salah tangkap sinyal
};

// Struktur paket data utama pengendalian pesawat (Dikirim dari TX ke RX)
struct ControlData {
  uint8_t throttle;  // Nilai gas motor (Rentang: 0 s/d 255)
  int8_t  rudder;    // Nilai belok rudder (Rentang: -127 s/d 127)
  int8_t  elevator;  // Nilai naik/turun elevator (Rentang: -127 s/d 127)
  int8_t  aileron;   // Nilai miring aileron (Rentang: -127 s/d 127)
  uint8_t isArmed;   // Status kunci keamanan motor (0 = Terkunci/Disarm, 1 = Aktif/Armed)
  uint8_t auxFlag;   // Pilihan Mode Terbang (1=Normal, 2=Diferensial, 3=Elevon/V-Tail, 4=Daratan)
  uint8_t motorType; // Pilihan Tipe Penggerak (0 = ESC Brushless, 1 = MOSFET Brushed DC)
};
ControlData txData;  // Deklarasi objek paket kendali

// Struktur paket data balasan telemetri (Diterima TX dari titipan ACK RX)
struct TelemetryData {
  uint8_t batteryStatus;  // Status baterai dari Auto-Cell RX (0 = Aman, 1 = Baterai Lemah)
  uint8_t failsafeActive; // Status darurat RX (0 = Sinyal Normal, 1 = RX sedang Failsafe)
};
TelemetryData rxTelem;    // Deklarasi objek paket telemetri

// ============================================================================
// 3. DEKLARASI OBJEK & VARIABEL GLOBAL SISTEM
// ============================================================================
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN); // Inisialisasi objek radio NRF24L01

uint8_t currentSlot     = 1; // Nomor slot model pesawat yang sedang aktif (1 s/d 10)
byte activePipe[5];          // Menyimpan alamat pipa radio dari slot yang sedang aktif
uint8_t activeChannel   = 76;// Menyimpan channel frekuensi dari slot yang sedang aktif
uint8_t activeMode      = 1; // Menyimpan mode terbang aktif (1 s/d 4)
uint8_t activeMotorType = 0; // Menyimpan tipe motor aktif (0 = ESC, 1 = MOSFET)

// Variabel penyimpan nilai geseran Trim untuk 4 sumbu kendali
int throttleTrim = 0; // Geseran trim gas (-127 s/d 127)
int rudderTrim   = 0; // Geseran trim rudder (-127 s/d 127)
int elevatorTrim = 0; // Geseran trim elevator (-127 s/d 127)
int aileronTrim  = 0; // Geseran trim aileron (-127 s/d 127)

// Variabel pewaktu pengiriman radio dan pemantauan sinyal
unsigned long lastTxTime  = 0;  // Mencatat waktu terakhir paket kendali dikirim
const unsigned long TX_INTERVAL = 30; // Jeda pengiriman paket radio (30ms = ~33 Hz)
unsigned long lastAckTime = 0;  // Mencatat waktu terakhir menerima balasan telemetri dari RX


// Variabel keamanan motor dan penghalus pergerakan tuas gas
bool isArmed           = false; // Status mesin (Default awal wajib false / terkunci)
// float smoothedThr      = 0;     // Nilai hasil filter penghalus EMA pada tuas gas
// const float EMA_ALPHA  = 0.15;  // Koefisien kelembutan filter EMA (0.15 = responsif & halus)

// Variabel manajemen Buzzer & LED Non-Blocking (Tanpa menghentikan laju program)
unsigned long buzzerEndTime = 0;     // Waktu target kapan buzzer harus berhenti berbunyi
bool buzzerActive           = false; // Status apakah buzzer sedang dalam masa aktif bunyi
uint8_t beepState = 0; 
unsigned long beepTimer = 0;
unsigned int beepInterval = 0;
// --- TAMBAHAN UNTUK BUZZER HYBRID ---
const bool PAKE_BUZZER_PASIF = true;  // Set 'true' untuk Buzzer Pasif, 'false' untuk Buzzer Aktif
const int FREKUENSI_BUZZER = 2500;    // Nada frekuensi 2.5kHz (Bisa diubah: 1000 - 4000)
// ------------------------------------

// ============================================================================
// 4. FUNGSI KENDALI AUDIO (BUZZER) & VISUAL (LED MERAH/HIJAU)
// ============================================================================
// Fungsi cerdas penggerak Buzzer (Hybrid)
void nyalakanBuzzer(bool nyala) {
  if (nyala) {
    if (PAKE_BUZZER_PASIF) tone(PIN_BUZZER, FREKUENSI_BUZZER);
    else digitalWrite(PIN_BUZZER, HIGH); 
  } else {
    if (PAKE_BUZZER_PASIF) noTone(PIN_BUZZER);
    else digitalWrite(PIN_BUZZER, LOW);  
  }
}
// Fungsi bip tunggal dengan pelindung Anti-Spam
void beep(unsigned int duration) {
  if (beepState != 0) return; // KUNCI: Tolak perintah masuk jika buzzer masih bunyi!
  beepState = 1;
  beepInterval = duration;
  beepTimer = millis();
  buzzerActive = true; 
}

// Fungsi bip ganda non-blocking (Opsional untuk menu konfigurasi)
void beepDouble() {
  if (beepState != 0) return;
  beepState = 3; 
  beepInterval = 100;
  beepTimer = millis();
  buzzerActive = true; 
}

// Fungsi pengatur nyala/kedip LED dan Buzzer yang dipanggil terus-menerus di loop()
void handleAudioVisual() {
  unsigned long m = millis(); // Ambil waktu sistem saat ini
  bool isBuzzerOn = false;    // Penanda apakah detik ini pin buzzer sedang menyala

  // --- MESIN STATUS BUZZER NON-BLOCKING ---
  if (beepState > 0) {
    // Nyala saat state 1 (Bip Tunggal) atau 3 (Bip Pertama Nada Ganda)
    if (beepState == 1 || beepState == 3) {
      nyalakanBuzzer(true);
      isBuzzerOn = true;
    } else {
      nyalakanBuzzer(false); // Matikan saat state 2 (Fase Jeda)
      isBuzzerOn = false;
    }
    
    // Cek apakah durasi interval saat ini sudah habis
    if (m - beepTimer >= beepInterval) {
      if (beepState == 1) { 
        // Selesai bip tunggal -> Matikan total
        nyalakanBuzzer(false);
        beepState = 0; 
        buzzerActive = false;
        isBuzzerOn = false;
      } 
      else if (beepState == 3) {
        // Selesai bip pertama -> Masuk jeda diam
        nyalakanBuzzer(false);
        beepState = 2; 
        beepInterval = 80; // Jeda diam 80 milidetik
        beepTimer = m;
        isBuzzerOn = false;
      }
      else if (beepState == 2) {
        // Jeda diam selesai -> Mulai bip kedua
        beepState = 1; // Ubah ke logika bip tunggal untuk akhiran
        beepInterval = 100; 
        beepTimer = m;
      }
    }
  } else {
    buzzerActive = false;
  }

  // Logika Indikator LED Utama & Ilusi Kedipan Notifikasi
  if (isArmed) {
    // KONDISI ARMED (Mesin Hidup): LED Hijau nyala konstan.
    // Jika buzzer berbunyi, matikan Hijau sesaat dan nyalakan Merah (efek kedip peringatan)
    digitalWrite(PIN_LED_HIJAU, isBuzzerOn ? LOW : HIGH);
    digitalWrite(PIN_LED_MERAH, isBuzzerOn ? HIGH : LOW);
  } else {
    // KONDISI DISARM (Mesin Mati): LED Merah nyala konstan sebagai tanda power remote ON.
    // Jika buzzer berbunyi, matikan Merah sesaat agar terlihat berkedip mengikuti irama suara
    digitalWrite(PIN_LED_HIJAU, LOW);
    digitalWrite(PIN_LED_MERAH, isBuzzerOn ? LOW : HIGH);
  }
}

// Fungsi bunyi dengan jeda kaku (Hanya dipakai saat Booting atau Menu di darat/Disarm)
void beepBlocking(unsigned int duration) {
  nyalakanBuzzer(true);    // Nyalakan buzzer
  digitalWrite(PIN_LED_MERAH, LOW);  // Kedipkan LED merah (mati sesaat)
  delay(duration);                   // Tahan selama durasi
  nyalakanBuzzer(false);     // Matikan buzzer
  digitalWrite(PIN_LED_MERAH, HIGH); // Nyalakan kembali LED merah
}

// ============================================================================
// 5. FUNGSI MANAJEMEN MEMORI EEPROM (10 SLOT MODEL PESAWAT)
// ============================================================================
// Menghitung alamat awal byte EEPROM untuk masing-masing slot (Tiap slot diberi ruang 15 byte)
int getSlotOffset(uint8_t slot) {
  return 100 + ((slot - 1) * 15); // Slot 1 mulai di alamat 100, Slot 2 di 115, dst.
}

// Menyimpan konfigurasi mode, trim, dan tipe motor ke slot EEPROM yang sedang aktif
void saveConfigToSlot() {
  int offset = getSlotOffset(currentSlot); // Cari alamat EEPROM untuk slot saat ini
  EEPROM.update(offset + 7,  activeMode);                  // Simpan mode terbang
  EEPROM.update(offset + 8,  (uint8_t)(throttleTrim + 128)); // Simpan trim gas (+128 agar positif)
  EEPROM.update(offset + 9,  (uint8_t)(rudderTrim + 128));   // Simpan trim rudder
  EEPROM.update(offset + 10, (uint8_t)(elevatorTrim + 128)); // Simpan trim elevator
  EEPROM.update(offset + 11, (uint8_t)(aileronTrim + 128));  // Simpan trim aileron
  EEPROM.update(offset + 12, activeMotorType);             // Simpan tipe motor (ESC/MOSFET)

  // Bunyikan nada melodi konfirmasi penyimpanan berhasil
  beepBlocking(100); delay(150);
  beepBlocking(100); delay(150);
  beepBlocking(300);

  Serial.print(F("[EEPROM] Konfigurasi Berhasil Disimpan ke Slot: "));
  Serial.println(currentSlot);
}

// Memuat data konfigurasi pesawat dari slot EEPROM yang dipilih
void loadModel(uint8_t slot) {
  int offset = getSlotOffset(slot); // Hitung alamat memori slot tujuan

  // Cek apakah slot ini belum pernah dipakai (Byte pengenal bukan 0xAA)
  if (EEPROM.read(offset) != 0xAA) {
    Serial.print(F("[EEPROM] Memformat Slot Baru: "));
    Serial.println(slot);
    EEPROM.update(offset, 0xAA);                     // Tulis segel pengenal bahwa slot sudah terisi
    EEPROM.update(offset + 1, random(20, 95));       // Buat frekuensi channel acak yang aman (20-94)
    for (int i = 0; i < 5; i++) {
      EEPROM.update(offset + 2 + i, 'A' + random(0, 26)); // Buat 5 huruf alamat pipa acak (A-Z)
    }
    EEPROM.update(offset + 7,  1);   // Default Mode Terbang = 1 (Normal)
    EEPROM.update(offset + 8,  128); // Default Trim Throttle = 0 (128 - 128 = 0)
    EEPROM.update(offset + 9,  128); // Default Trim Rudder   = 0
    EEPROM.update(offset + 10, 128); // Default Trim Elevator = 0
    EEPROM.update(offset + 11, 128); // Default Trim Aileron  = 0
    EEPROM.update(offset + 12, 0);   // Default Tipe Motor    = 0 (ESC Brushless)
  }

  // Baca seluruh parameter dari EEPROM ke variabel aktif di RAM
  activeChannel = EEPROM.read(offset + 1);                // Ambil frekuensi channel
  for (int i = 0; i < 5; i++) {
    activePipe[i] = EEPROM.read(offset + 2 + i);          // Ambil 5 byte alamat pipa
  }
  activeMode      = EEPROM.read(offset + 7);              // Ambil mode terbang
  throttleTrim    = (int)EEPROM.read(offset + 8) - 128;   // Ambil trim gas (kembalikan ke skala minus/plus)
  rudderTrim      = (int)EEPROM.read(offset + 9) - 128;   // Ambil trim rudder
  elevatorTrim    = (int)EEPROM.read(offset + 10) - 128;  // Ambil trim elevator
  aileronTrim     = (int)EEPROM.read(offset + 11) - 128;  // Ambil trim aileron
  activeMotorType = EEPROM.read(offset + 12);             // Ambil tipe motor

  EEPROM.update(50, slot); // Catat nomor slot terakhir yang dipakai di alamat 50
  currentSlot = slot;      // Perbarui variabel slot aktif saat ini

  // Beri tanda bunyi buzzer sebanyak nomor slot yang dimuat
  for (int i = 0; i < slot; i++) {
    beepBlocking(100);
    delay(200);
  }

  // Cetak informasi lengkap slot ke Serial Monitor untuk keperluan debugging
  Serial.print(F("[EEPROM] Berhasil Load Slot: ")); Serial.print(slot);
  Serial.print(F(" | Ch: ")); Serial.print(activeChannel);
  Serial.print(F(" | Mode: ")); Serial.print(activeMode);
  Serial.print(F(" | Motor: ")); Serial.println(activeMotorType == 0 ? F("ESC") : F("MOSFET"));
}

// ============================================================================
// 6. FUNGSI PROSES BINDING (PENCOCOKAN ID TX KE RX SELAMA 10 DETIK)
// ============================================================================
void executeBindingProcess() {
  Serial.println(F("[BIND] Memulai Proses Pancaran Binding 10 Detik..."));
  radio.stopListening();             // 1. WAJIB DIATAS! Ubah jadi pemancar.
  radio.setChannel(BIND_CH);         // 2. Set frekuensi
  radio.openWritingPipe(BIND_ADDR);  // 3. Masukkan alamat "BIND0"

  // Siapkan isi paket identitas yang akan dikirim ke RX
  BindPayload bp;
  memcpy(bp.pipeAddr, activePipe, 5); // Salin 5 byte alamat pipa dari slot aktif saat ini
  bp.opChannel = activeChannel;       // Masukkan nomor frekuensi channel operasional
  bp.checkSum  = activeChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4]; // Buat rumus validasi XOR

  unsigned long bindStartTime = millis(); // Catat waktu dimulainya proses binding

  // Pancarkan paket identitas terus-menerus selama 10.000 milidetik (10 detik)
  while (millis() - bindStartTime < 10000) {
    radio.write(&bp, sizeof(BindPayload)); // Pancarkan paket binding ke udara

    // Nyalakan Buzzer & Kedipkan LED Merah-Hijau secara bergantian (Silang)
    nyalakanBuzzer(true);
    digitalWrite(PIN_LED_MERAH, HIGH);
    digitalWrite(PIN_LED_HIJAU, LOW);
    delay(40);

    nyalakanBuzzer(false);
    digitalWrite(PIN_LED_MERAH, LOW);
    digitalWrite(PIN_LED_HIJAU, HIGH);
    delay(80);
  }

  // Kembalikan kondisi LED dan beri bunyi panjang tanda selesai binding
  digitalWrite(PIN_LED_HIJAU, LOW);
  beepBlocking(500);
  Serial.println(F("[BIND] Selesai! Kembali ke Frekuensi Operasional."));
}

// ============================================================================
// 7. FUNGSI KONVERSI & KALIBRASI STIK ANALOG (-127 S/D 127)
// ============================================================================
int mapJoystick(int val, int lower, int middle, int upper) { 
  val = constrain(val, lower, upper);
  int output = 0;

  if (val < middle - 15) {
    output = map(val, lower, middle - 15, -127, 0);
  } else if (val > middle + 15) {
    output = map(val, middle + 15, upper, 0, 127);
  }

  return output; // Hanya mengembalikan nilai stik murni
}

// ============================================================================
// 8. FUNGSI UTAMA: SETUP (DIJALANKAN 1 KALI SAAT REMOTE DINYALAKAN)
// ============================================================================
void setup() {
  // BOARD NANO WAJIB: Gunakan tegangan 3.3V dari pin AREF sebagai batas atas pembacaan analog (1023 = 3.3V)
  // BOARD PROMINI 8MHZ: Cukup beri komentar
  // analogReference(EXTERNAL);  

  Serial.begin(115200); // Buka komunikasi Serial Monitor pada kecepatan 115200 bps

  // Konfigurasi pin output indikator
  pinMode(PIN_LED_HIJAU, OUTPUT);
  pinMode(PIN_LED_MERAH, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // WAJIB: Gunakan INPUT_PULLUP internal 3.3V Pro Mini untuk tombol L & R (Aman tersambung ke GND)
  pinMode(PIN_BTN_L, INPUT_PULLUP);
  pinMode(PIN_BTN_R, INPUT_PULLUP);

  // Pastikan buzzer mati dan LED Merah menyala di awal booting
  nyalakanBuzzer(false);
  digitalWrite(PIN_LED_MERAH, HIGH);
  digitalWrite(PIN_LED_HIJAU, LOW);

  // Beri jeda 50ms agar tegangan Pull-Up 3.3V pada pin 4 & 5 stabil sepenuhnya sebelum dibaca
  delay(50);

  // Ambil bibit angka acak dari derau pin analog kosong untuk pembuatan ID slot baru
  randomSeed(analogRead(A6) + analogRead(A7));

  // Baca memori slot terakhir yang digunakan dari alamat EEPROM 50
  uint8_t lastSlot = EEPROM.read(50);
  if (lastSlot < 1 || lastSlot > MAX_MODELS) {
    lastSlot = 1; // Jika nilai di luar batas (EEPROM baru), paksa ke Slot 1
  }
  loadModel(lastSlot); // Muat seluruh pengaturan dari slot tersebut

  // Inisialisasi dan periksa koneksi fisik modul NRF24L01
  if (!radio.begin()) {
    Serial.println(F("[FATAL] Modul NRF24L01 Tidak Terdeteksi! Cek Jalur SPI/3.3V!"));
    while (1) {
      digitalWrite(PIN_LED_MERAH, !digitalRead(PIN_LED_MERAH));
      delay(100);
    }
  }

  // Konfigurasi parameter komunikasi radio NRF24L01
  radio.setPALevel(RF24_PA_HIGH); // HIGH agar pancaran radio kuat
  radio.setDataRate(RF24_1MBPS); // 250kbps = Jarak jauh & stabil, 1Mbps = Jarak pendek & cepat
  radio.setRetries(1, 3); // Jeda 500us, dicoba 3x. Total hanya 1.5ms (Aman dari macet & ACK tertangkap!)
  radio.enableDynamicPayloads();  
  radio.enableAckPayload();       

  // Cek apakah pengguna menahan Tombol L dan R bersamaan saat menyalakan remote
  if (digitalRead(PIN_BTN_L) == LOW && digitalRead(PIN_BTN_R) == LOW) {
    executeBindingProcess(); 
  }

  // Setel radio ke frekuensi dan alamat pipa operasional sesuai slot aktif
  radio.stopListening();          // WAJIB dipanggil SEBELUM openWritingPipe!
  radio.setChannel(activeChannel);
  radio.openWritingPipe(activePipe);
  delay(10);                      // Beri waktu 10ms agar register PLL frekuensi NRF terkunci

  isArmed = false;
  lastAckTime = millis(); 
  beepBlocking(200);
  initLCD();      
  Serial.println(F("[SYSTEM] TX V930 Siap Beroperasi. Status: DISARMED (Mesin Terkunci)."));
}

// ============================================================================
// 9. FUNGSI UTAMA: LOOP (DIJALANKAN BERULANG-ULANG TANPA HENTI)
// ============================================================================
void loop() {
  unsigned long currentMillis = millis(); // Ambil waktu sistem berjalan saat ini
  handleAudioVisual();                    // Kelola bunyi Buzzer dan nyala LED secara Non-Blocking

 // --------------------------------------------------------------------------
  // A. PEMANTAUAN ALARM TELEMETRI BATERAI (SABAR)
  // --------------------------------------------------------------------------
  static unsigned long lastAlarmTime = 0; 
  if (isArmed) {
    // KITA HAPUS ALARM KEKUATAN SINYAL, HANYA SISA ALARM BATERAI
    if (rxTelem.batteryStatus == 1) {
      if (currentMillis - lastAlarmTime > 2500) {
        beep(500);
        lastAlarmTime = currentMillis;
      }
    }
  }

  // --------------------------------------------------------------------------
  // B. SIKLUS PEMBACAAN INPUT & PENGIRIMAN RADIO (SETIAP 30 MILIDETIK)
  // --------------------------------------------------------------------------
  if (currentMillis - lastTxTime >= TX_INTERVAL) {
    lastTxTime = currentMillis; // Perbarui waktu eksekusi siklus

    // 1. Baca nilai tegangan analog mentah (0 - 1023) dari keempat potensio stik & tombol trim
    int rawThr  = analogRead(PIN_THR);  // Baca Stik Gas
    int rawRud  = analogRead(PIN_RUD);  // Baca Stik Rudder
    int rawEle  = analogRead(PIN_ELE);  // Baca Stik Elevator
    int rawAil  = analogRead(PIN_AIL);  // Baca Stik Aileron
    int rawTrim = analogRead(PIN_TRIM); // Baca Rangkaian Resistor Ladder 8 Tombol Trim

    // 2. Baca status penekanan Tombol Bahu L dan R (Bernilai true jika ditekan ke GND / LOW)
    bool btnL = (digitalRead(PIN_BTN_L) == LOW);
    bool btnR = (digitalRead(PIN_BTN_R) == LOW);

    // ------------------------------------------------------------------------
    // C. LOGIKA TOMBOL R (ARMING & DISARM AMAN TANPA BLOCKING)
    // ------------------------------------------------------------------------
    // - Saat di darat (Gas Nol): Klik R langsung mengubah Arm <-> Disarm dengan cepat.
    // - Saat terbang (Gas Naik): Wajib tahan R selama 0.6 detik untuk Disarm (Cegah tersenggol).
    static unsigned long btnRPressTime = 0; // Mencatat kapan tombol R mulai ditekan
    static bool btnRWasPressed = false;     // Status apakah tombol R sedang ditahan
    static bool disarmHandled  = false;     // Mencegah pemicuan ganda saat tombol ditahan lama

    if (btnR && !btnL) {
      // Deteksi awal saat jari baru menyentuh tombol R
      if (!btnRWasPressed) {
        btnRPressTime  = currentMillis;
        btnRWasPressed = true;
        disarmHandled  = false;

        // Jika kondisi sedang DISARM -> Proses permintaan ARMING seketika (1 Klik)
        if (!isArmed) {
          if (rawThr > 40) {
            // Tolak Arming jika tuas gas belum berada di posisi paling bawah
            beepBlocking(80); delay(80); beepBlocking(80); // <--- INI UBAHANNYA (2x Bip)
            Serial.println(F("[WARNING] Arming Ditolak! Turunkan Tuas Gas ke Nol!"));
          } else {
            // Izinkan Arming karena gas sudah aman di posisi nol
            isArmed = true;
            beep(600);
            Serial.println(F("[MOTOR] STATUS: ARMED! Motor Siap Berputar!"));
          }
          disarmHandled = true; // Tandai aksi sudah selesai untuk tekanan ini
        }
        // Jika sedang ARMED dan tuas gas sudah di nol (di darat) -> Langsung Disarm dengan 1 Klik!
        else if (isArmed && rawThr <= 40) {
          isArmed = false;
          beep(200);
          Serial.println(F("[MOTOR] STATUS: DISARMED! (Gas Nol - Klik Instan)"));
          disarmHandled = true;
        }
      }

      // Jika sedang ARMED dan gas sedang menyala (di udara) -> Wajib tahan 600ms untuk Disarm Darurat
      if (isArmed && !disarmHandled && (currentMillis - btnRPressTime > 600)) {
        isArmed = false;
        beep(400);
        Serial.println(F("[MOTOR] STATUS: EMERGENCY DISARM! (Tahan 0.6 Detik)"));
        disarmHandled = true;
      }
    } else if (!btnR) {
      btnRWasPressed = false; // Reset status ketika jari dilepas dari tombol R
    }

    // ------------------------------------------------------------------------
    // D. LOGIKA TOMBOL L (MENU KOMBINASI & RESET TRIM CERDAS)
    // ------------------------------------------------------------------------
    static bool btnLWasPressed = false;
    static unsigned long btnLPressTime = 0;
    static unsigned long lastMenuTime = 0;
    static bool menuExecuted = false;
    const unsigned long SHORT_CLICK_MAX = 400; // Batas waktu klik pendek (400 milidetik)

    if (btnL && !btnR) {
      // Deteksi awal saat jari baru menyentuh tombol L
      if (!btnLWasPressed) {
        btnLWasPressed = true;
        btnLPressTime = currentMillis;
        menuExecuted = false;

        // 1. TERBANG (ARMED): Reset Instan detik itu juga! (Menu tidak bisa diakses)
        if (isArmed) {
          throttleTrim = 0; rudderTrim = 0; elevatorTrim = 0; aileronTrim = 0;
          beep(400);
          Serial.println(F("[TRIM] Reset Instan (Mode Terbang)!"));
          menuExecuted = true; // Tandai selesai agar tidak memicu apapun saat dilepas
        }
      }

      // 2. DARAT (DISARM): L Ditahan, Menunggu Kombinasi Trim
      if (!isArmed && !menuExecuted) {
        if (rawTrim < 900 && (currentMillis - lastMenuTime > 500)) {
          if (rawTrim > 460 && rawTrim < 530) { // Aileron Kanan -> Pindah Slot
            currentSlot = (currentSlot % MAX_MODELS) + 1;
            loadModel(currentSlot);
            radio.setChannel(activeChannel); radio.openWritingPipe(activePipe);
            triggerAnimSlot(currentSlot);
            menuExecuted = true; lastMenuTime = currentMillis;
          } 
          else if (rawTrim > 380 && rawTrim < 450) { // Elevator Bawah -> Tipe Motor
            activeMotorType = (activeMotorType == 0) ? 1 : 0;
            triggerAnimMotor(activeMotorType);
            if (activeMotorType == 0) beep(150); // Bip tunggal aman
            else beepDouble();                   // Bip ganda tanpa delay
            menuExecuted = true; lastMenuTime = currentMillis;
          }
          else if (rawTrim > 300 && rawTrim < 360) { // Elevator Atas -> Mode Terbang
            activeMode = (activeMode % 4) + 1;
            beepBlocking(400); delay(200);
            for (int i = 0; i < activeMode; i++) { beepBlocking(100); delay(150); }
            menuExecuted = true; lastMenuTime = currentMillis;
          } 
          else if (rawTrim > 540 && rawTrim < 610) { // Aileron Kiri -> Simpan EEPROM
            saveConfigToSlot();
            menuExecuted = true; lastMenuTime = currentMillis;
          }
        }
      }
    } 
    else if (!btnL) {
      // 3. Saat tombol L DILEPAS
      if (btnLWasPressed) {
        // Jika statusnya masih Disarm dan Menu belum tereksekusi
        if (!isArmed && !menuExecuted) {
          // Hitung berapa lama tombol L tadi ditahan
          unsigned long holdDuration = currentMillis - btnLPressTime;
          
          if (holdDuration < SHORT_CLICK_MAX) {
            // Aksi 3A: Klik Cepat (< 400ms) -> RESET TRIM
            throttleTrim = 0; rudderTrim = 0; elevatorTrim = 0; aileronTrim = 0;
            beep(400);
            Serial.println(F("[TRIM] Reset (Klik Pendek di Darat)!"));
          } else {
            // Aksi 3B: Ditahan Lama (>= 400ms) tapi Lupa Pencet Trim -> ABAIKAN
            Serial.println(F("[MENU] Batal. Tombol L dilepas tanpa kombinasi. Trim Aman."));
            // Tidak ada bunyi atau reset. Sistem kembali normal.
          }
        }
        btnLWasPressed = false; // Bersihkan status memori
      }
    }

    // ------------------------------------------------------------------------
    // E. PEMBACAAN 8 TOMBOL TRIM RESISTOR LADDER PIN A4 (NON-BLOCKING)
    // ------------------------------------------------------------------------
    static unsigned long lastTrimProcess = 0; // Pewaktu jeda pembacaan trim beruntun
    // Jika nilai ADC di bawah 900 (berarti ada salah satu tombol trim yang ditekan)
    // Syarat Mutlak: Trim HANYA berfungsi jika Tombol L TIDAK DITEKAN (!btnL)
    if (!btnL && rawTrim < 900 && (currentMillis - lastTrimProcess > 150)) {
      bool trimChanged = false; // Penanda apakah nilai trim berubah

      if (rawTrim < 40) {
        throttleTrim = constrain(throttleTrim + 2, -127, 127); // Tombol 1: Trim Gas Naik
        trimChanged = true;
      } else if (rawTrim > 80 && rawTrim < 130) {
        throttleTrim = constrain(throttleTrim - 2, -127, 127); // Tombol 2: Trim Gas Turun
        trimChanged = true;
      } else if (rawTrim > 140 && rawTrim < 200) {
        rudderTrim = constrain(rudderTrim - 2, -127, 127);     // Tombol 3: Trim Rudder Kiri
        trimChanged = true;
      } else if (rawTrim > 230 && rawTrim < 290) {
        rudderTrim = constrain(rudderTrim + 2, -127, 127);     // Tombol 4: Trim Rudder Kanan
        trimChanged = true;
      } else if (rawTrim > 300 && rawTrim < 360) {
        elevatorTrim = constrain(elevatorTrim + 2, -127, 127); // Tombol 5: Trim Elevator Atas
        trimChanged = true;
      } else if (rawTrim > 380 && rawTrim < 450) {
        elevatorTrim = constrain(elevatorTrim - 2, -127, 127); // Tombol 6: Trim Elevator Bawah
        trimChanged = true;
      } else if (rawTrim > 460 && rawTrim < 530) {
        aileronTrim = constrain(aileronTrim + 2, -127, 127);   // Tombol 7: Trim Aileron Kanan
        trimChanged = true;
      } else if (rawTrim > 540 && rawTrim < 610) {
        aileronTrim = constrain(aileronTrim - 2, -127, 127);   // Tombol 8: Trim Aileron Kiri
        trimChanged = true;
      }

      // Jika terdeteksi tombol trim valid ditekan, bunyikan klik pendek & catat waktunya
      if (trimChanged) {
        beep(30); // Bunyi klik pendek non-blocking
        lastTrimProcess = currentMillis;
        Serial.print(F("[TRIM] T:")); Serial.print(throttleTrim);
        Serial.print(F(" R:")); Serial.print(rudderTrim);
        Serial.print(F(" E:")); Serial.print(elevatorTrim);
        Serial.print(F(" A:")); Serial.println(aileronTrim);
      }
    }

    // ------------------------------------------------------------------------
    // F. KALKULASI STIK MURNI & PENYUSUNAN PAKET FINAL
    // ------------------------------------------------------------------------
    // 1. Olah Stik Throttle (Respons Instan & Presisi)
    int thrMurni = map(rawThr, 15, 680, 0, 255); // Langsung petakan nilai mentah
    if (rawThr <= 20) thrMurni = 0;              // Ambang batas bawah diturunkan
    thrMurni = constrain(thrMurni, 0, 255);      // Pastikan nilai murni terkunci aman 0-255
    
    int thrFinal = constrain(thrMurni + throttleTrim, 0, 255); // Nilai untuk NRF
    
    // 2. Olah Stik Rudder, Elevator, dan Aileron (Nilai MURNI untuk LCD)
    int rudMurni = mapJoystick(rawRud, 11, 504, 1000);
    int eleMurni = mapJoystick(rawEle, 8,  515, 1000);
    int ailMurni = mapJoystick(rawAil, 10, 513, 1012);

    // 3. Kalkulasi Nilai FINAL (Stik Murni + Trim) untuk Pesawat
    int rudFinal = constrain(rudMurni - rudderTrim, -127, 127);
    int eleFinal = constrain(eleMurni + elevatorTrim, -127, 127);
    int ailFinal = constrain(ailMurni - aileronTrim, -127, 127);

    // 4. Masukkan status sistem & nilai FINAL ke paket data TX (Untuk RX)
    txData.isArmed   = isArmed ? 1 : 0;
    txData.auxFlag   = activeMode;
    txData.motorType = activeMotorType;

    // WAJIB: Pesawat menerima rxData yang sudah FINAL (sudah ditambah trim)
    txData.throttle = isArmed ? (uint8_t)constrain(thrFinal, 0, 255) : 0; 
    txData.rudder   = (int8_t)rudFinal;
    txData.elevator = (int8_t)eleFinal;
    txData.aileron  = (int8_t)ailFinal;

    // ------------------------------------------------------------------------
    // G. PENGIRIMAN RADIO SINKRONUS (STABIL & PASTI MENANGKAP ACK)
    // ------------------------------------------------------------------------
    if (radio.write(&txData, sizeof(ControlData))) {
      // Sinyal sukses menembus antena RX, segera intip apakah RX menitipkan data baterai
      if (radio.isAckPayloadAvailable()) {
        radio.read(&rxTelem, sizeof(TelemetryData)); 
        lastAckTime = currentMillis; 
      }
    } else {
      // Sinyal tidak mendapat balasan (jarak asimetris/terhalang pohon), buang memori macet!
      radio.flush_tx(); 
    }

    // Kalkulasi koneksi LCD (Toleransi ikon hilang jika putus maksimal 1 detik)
    bool isRxConnected = (currentMillis - lastAckTime < 1000);
    
    // Jika lebih dari 1 detik tidak ada balasan, lupakan status baterai klien!
    if (!isRxConnected) {
      rxTelem.batteryStatus = 0; // Paksa nol agar alarm RX di TX langsung bungkam
    }

    // ========================================================================
    // --- LOGIKA BACA BATERAI TX (DENGAN DUMMY READ ANTI-CROSSTALK) ---
    // ========================================================================
    analogRead(PIN_BAT_TX); // DUMMY READ: Buang sisa tegangan dari pin A4 (Trim)
    int rawBatTx = analogRead(PIN_BAT_TX); // Baca tegangan baterai yang asli & bersih
    int txBatLevel = 0; 
    
    if (rawBatTx > 600) txBatLevel = 4;      // > 3.85V (Full)
    else if (rawBatTx > 570) txBatLevel = 3; // > 3.65V
    else if (rawBatTx > 540) txBatLevel = 2; // > 3.50V
    else if (rawBatTx > 515) txBatLevel = 1; // > 3.35V (Low)
    else txBatLevel = 0;                     // < 3.35V (Kritis Parah)

    // --- ALARM KRITIS BATERAI TX DENGAN ANTI-NYANGKUT ---
    static bool alarmBatreNyangkut = false; // Penanda untuk mematikan paksa
    
    if (txBatLevel == 0 && !buzzerActive) {
      unsigned long critTime = currentMillis % 1000; 
      if ((critTime < 100) || (critTime > 200 && critTime < 300)) {
        nyalakanBuzzer(true);
        alarmBatreNyangkut = true;
      } else {
        nyalakanBuzzer(false);
        alarmBatreNyangkut = false;
      }
    } 
    // JIKA BATERAI KEMBALI NORMAL TAPI BUZZER MASIH TERIAK, MATIKAN PAKSA!
    else if (alarmBatreNyangkut && txBatLevel > 0) {
      nyalakanBuzzer(false);
      alarmBatreNyangkut = false;
    }
    
    // --- LOGIKA DETEKSI CHARGING (A7) ---
    // Jika ditarik IC TP4056 ke Ground = mengecas (< 500). Jika mengambang = penuh/dicabut (> 500).
    bool isCharging = (analogRead(PIN_CHRG) < 500);

    // Panggil updateLCD dan masukkan txBatLevel ke dalamnya
    updateLCD(isArmed, thrMurni, txBatLevel, isRxConnected, 
              activeMode, activeMotorType, 
              throttleTrim, rudderTrim, elevatorTrim, aileronTrim, 
              rudMurni, eleMurni, ailMurni, isCharging);

    // ------------------------------------------------------------------------
    // H. SERIAL SOPAN (Hanya cetak jika tuas digeser atau tombol ditekan)
    // ------------------------------------------------------------------------
    static int lastThr = 0, lastRud = 0, lastEle = 0, lastAil = 0, lastTrmVal = 0;
    static bool lastArmStatus = false;
    bool harusCetak = false;

    // Cek apakah ada pergerakan stik atau status arming
    if (abs(txData.throttle - lastThr) > 2) harusCetak = true;
    if (abs(txData.rudder - lastRud) > 2) harusCetak = true;
    if (abs(txData.elevator - lastEle) > 2) harusCetak = true;
    if (abs(txData.aileron - lastAil) > 2) harusCetak = true;
    if (isArmed != lastArmStatus) harusCetak = true;
    if (rawTrim < 900 && abs(rawTrim - lastTrmVal) > 10) harusCetak = true;

    if (harusCetak) {
      Serial.print(F("Arm:")); Serial.print(isArmed);
      Serial.print(F(" | Thr:")); Serial.print(txData.throttle);
      Serial.print(F(" Rud:")); Serial.print(txData.rudder);
      Serial.print(F(" Ele:")); Serial.print(txData.elevator);
      Serial.print(F(" Ail:")); Serial.print(txData.aileron);
      Serial.print(F(" | Bat:")); Serial.println(rxTelem.batteryStatus);
      
      lastThr = txData.throttle; lastRud = txData.rudder;
      lastEle = txData.elevator; lastAil = txData.aileron;
      lastArmStatus = isArmed; lastTrmVal = rawTrim;
    }
  }
}
// =================================== AKHIR KODE ===================================