/*
 * ============================================================================
 * FIRMWARE TRANSMITTER (TX) - V930 RC UNIVERSAL HYBRID (PRO MINI)
 * ============================================================================
 * Spesifikasi Hardware & Kelistrikan:
 * - Mikrokontroler : Arduino Pro Mini (ATmega168P/328P - 3.3V 8MHz)
 * - Tegangan VCC   : 3.3V (Wajib, bisa dari LDO bawaan board V930)
 * - Potensio Stik  : 3.3V (Pin A0, A1, A2, A3)
 * - Tombol Trim    : 3.3V Resistor Ladder 8 Tombol (Pin A4)
 * - Modul NRF24L01 : 3.3V (CE = Pin 9, CSN = Pin 10)
 * - Tombol L & R   : Isolasi Jalur, Menggunakan Pull-Up Internal (Pin 4 & 5)
 * - Indikator      : LED Hijau (Pin 3), LED Merah (Pin 2), Buzzer (Pin 8)
 * 
 * ============================================================================
 * PANDUAN PENGGUNAAN & FITUR TX:
 * ============================================================================
 * [TOMBOL R] - ARMING/DISARMING (Menghidupkan/Mematikan Motor):
 * - Saat di Darat (Gas Nol): Tekan 1x untuk Arm/Disarm cepat.
 * - Saat di Udara (Gas Naik): Tolak Arming. Wajib tahan 0.6 detik untuk 
 *                             Emergency Disarm.
 * 
 * [TOMBOL L] - MENU KOMBINASI DARAT (Harus kondisi Disarmed):
 * - Tahan L + Trim Aileron Kanan = Pindah Slot Memori Pesawat (1-10)
 * - Tahan L + Trim Elevator Bawah= Ganti Tipe Motor (ESC Brushless / MOSFET)
 * - Tahan L + Trim Elevator Atas = Ganti Mode Terbang (Normal/Glider/Jet/Mobil)
 * - Tahan L + Trim Aileron Kiri  = Simpan Pengaturan (Save EEPROM)
 * 
 * [TOMBOL L] - RESET TRIM INSTAN:
 * - Saat di Darat (Disarmed): Klik L dengan cepat (<400ms).
 * - Saat di Udara (Armed): Klik L kapan saja, trim langsung kembali nol.
 * ============================================================================
 */

#include <Arduino.h> // Library utama fungsi dasar Arduino
#include <SPI.h>     // Library komunikasi protokol SPI untuk modul radio
#include <RF24.h>    // Library pengendali modul transceiver NRF24L01
#include <EEPROM.h>  // Library penyimpanan memori permanen di dalam chip
#include "lcd_h8805a.h" // Library khusus untuk mengontrol layar LCD V930

// ============================================================================
// 1. DEFINISI PIN HARDWARE ARDUINO PRO MINI
// ============================================================================
#define PIN_RADIO_CE   9  // Pin Chip Enable untuk mengaktifkan mode kirim/terima NRF24
#define PIN_RADIO_CSN  10 // Pin Chip Select Not untuk jalur seleksi komunikasi SPI NRF24
#define PIN_LED_HIJAU  3  // Pin output LED Hijau (Indikator ARMED / Mesin Hidup)
#define PIN_LED_MERAH  2  // Pin output LED Merah (Indikator DISARM / Mesin Terkunci)
#define PIN_BTN_L      4  // Pin input Tombol Bahu Kiri (Menu Konfigurasi & Reset Trim)
#define PIN_BTN_R      5  // Pin input Tombol Bahu Kanan (Saklar Arming / Disarm)
#define PIN_BUZZER     8  // Pin output Buzzer untuk alarm dan nada notifikasi

const uint8_t PIN_THR  = A2; // Pin input analog Stik Gas (Throttle)
const uint8_t PIN_RUD  = A3; // Pin input analog Stik Belok Ekor (Rudder)
const uint8_t PIN_ELE  = A1; // Pin input analog Stik Naik/Turun (Elevator)
const uint8_t PIN_AIL  = A0; // Pin input analog Stik Kemiringan Sayap (Aileron)
const uint8_t PIN_TRIM = A4; // Pin input analog Rangkaian 8 Tombol Trim (Resistor Ladder)
const uint8_t PIN_BAT_TX = A6; // Pin analog pembaca tegangan baterai internal Remote (TX)
const uint8_t PIN_CHRG   = A7; // Pin sensor IC Charger TP4056 (Wajib ada resistor pull-up 10k ke 3.3V)

const byte BIND_ADDR[6]  = "BIND0"; // Alamat pintu darurat khusus saat proses Binding
const uint8_t BIND_CH    = 10;      // Frekuensi saluran darurat khusus saat proses Binding
const uint8_t MAX_MODELS = 10;      // Jumlah maksimal kapasitas garasi pesawat di EEPROM

// ============================================================================
// 2. STRUKTUR DATA KOMUNIKASI RADIO (WAJIB 100% SAMA DENGAN RX)
// ============================================================================
// Struktur paket perkenalan (Hanya dipakai 10 detik pertama saat kombinasi tombol L+R)
struct BindPayload {
  byte pipeAddr[5];  // 5 huruf acak sebagai password unik untuk RX
  uint8_t opChannel; // 1 angka saluran frekuensi (20-95) tempat TX/RX akan ngobrol
  uint8_t checkSum;  // 1 angka gembok verifikasi agar RX tidak menangkap sinyal nyasar
};

// Struktur paket data kendali pesawat terbang (Dikirim 33x per detik ke RX)
struct ControlData {
  uint8_t throttle;  // Nilai gas murni + trim (0 s/d 255)
  int8_t  rudder;    // Nilai belok ekor murni + trim (-127 s/d 127)
  int8_t  elevator;  // Nilai naik turun murni + trim (-127 s/d 127)
  int8_t  aileron;   // Nilai miring sayap murni + trim (-127 s/d 127)
  uint8_t isArmed;   // Bendera keamanan motor (0 = Kunci, 1 = Buka)
  uint8_t auxFlag;   // Mode Mixer Aerodinamika (1=Pesawat Biasa, 2=Glider, 3=Jet, 4=Mobil)
  uint8_t motorType; // Jalur distribusi listrik RX (0 = Pin ESC, 1 = Pin MOSFET)
};
ControlData txData;  // Membuat kotak nyata penampung paket kendali

// Struktur paket laporan kesehatan pesawat (Disedot oleh TX dari dalam RX)
struct TelemetryData {
  uint8_t batteryStatus;  // Laporan Voltase Lipo RX (0 = Baterai Penuh/Aman, 1 = Baterai Hampir Habis)
  uint8_t failsafeActive; // Laporan Status Sinyal RX (0 = Terkendali, 1 = RX sedang panik hilang arah)
};
TelemetryData rxTelem;    // Membuat kotak nyata penampung paket laporan

// ============================================================================
// 3. DEKLARASI OBJEK & VARIABEL GLOBAL SISTEM
// ============================================================================
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN); // Menghidupkan mesin penggerak NRF24L01

uint8_t currentSlot     = 1; // Posisi aktif memori pesawat (Slot 1 sampai 10)
byte activePipe[5];          // Password komunikasi pesawat yang sedang aktif
uint8_t activeChannel   = 76;// Saluran frekuensi pesawat yang sedang aktif
uint8_t activeMode      = 1; // Mode terbang yang sedang aktif
uint8_t activeMotorType = 0; // Tipe distribusi motor yang sedang aktif

// Memori Geseran Tuas Trim (-127 s/d 127)
int throttleTrim = 0; 
int rudderTrim   = 0; 
int elevatorTrim = 0; 
int aileronTrim  = 0; 

unsigned long lastTxTime  = 0; // Stopwatch ritme pengiriman data radio
const unsigned long TX_INTERVAL = 30; // 30ms = Remote mengirim perintah 33 kali dalam satu detik
unsigned long lastAckTime = 0; // Stopwatch ritme balasan kesehatan pesawat (Untuk deteksi sinyal putus)

bool isArmed           = false; // Bendera utama keselamatan pilot: Wajib false (Terkunci) saat mesin menyala

// --- SISTEM MANAJEMEN BUZZER CERDAS (NON-BLOCKING) ---
unsigned long buzzerEndTime = 0; 
bool buzzerActive           = false; 
uint8_t beepState = 0; 
unsigned long beepTimer = 0;
unsigned int beepInterval = 0;

// Opsi Hardware Buzzer:
const bool PAKE_BUZZER_PASIF = true;  // Ubah ke 'false' jika modul buzzer Anda adalah tipe aktif (2 kaki biasa)
const int FREKUENSI_BUZZER = 2500;    // Tingkat lengkingan suara (Hanya berefek jika pakai pasif)

// ============================================================================
// 4. FUNGSI KENDALI AUDIO (BUZZER) & VISUAL (LED MERAH/HIJAU)
// ============================================================================
// Pengendali fisik tegangan ke kaki Buzzer
void nyalakanBuzzer(bool nyala) {
  if (nyala) {
    if (PAKE_BUZZER_PASIF) tone(PIN_BUZZER, FREKUENSI_BUZZER);
    else digitalWrite(PIN_BUZZER, HIGH); 
  } else {
    if (PAKE_BUZZER_PASIF) noTone(PIN_BUZZER);
    else digitalWrite(PIN_BUZZER, LOW);  
  }
}

// Bip Cepat: Bunyi sekali lalu otomatis mati (Sangat berguna saat menerbangkan agar kode tidak macet)
void beep(unsigned int duration) {
  if (beepState != 0) return; // Anti-Spam: Tolak jika mulut buzzer masih penuh menyuarakan perintah lain
  beepState = 1;
  beepInterval = duration;
  beepTimer = millis();
  buzzerActive = true; 
}

// Bip Ganda Cepat: Khusus digunakan sebagai notifikasi penggantian menu
void beepDouble() {
  if (beepState != 0) return;
  beepState = 3; 
  beepInterval = 100;
  beepTimer = millis();
  buzzerActive = true; 
}

// Mesin pemutar suara dan lampu di latar belakang (Dipanggil setiap putaran loop)
void handleAudioVisual() {
  unsigned long m = millis(); 
  bool isBuzzerOn = false;    

  // --- PEMROSES SUARA DI LATAR BELAKANG ---
  if (beepState > 0) {
    if (beepState == 1 || beepState == 3) {
      nyalakanBuzzer(true);
      isBuzzerOn = true;
    } else {
      nyalakanBuzzer(false); // Mode 2: Diam sesaat sebelum lanjut bunyi lagi
      isBuzzerOn = false;
    }
    
    // Cek apakah waktu berbunyi (atau waktu diam) sudah habis
    if (m - beepTimer >= beepInterval) {
      if (beepState == 1) { 
        // Akhir dari bunyi tunggal, bersihkan memori
        nyalakanBuzzer(false);
        beepState = 0; 
        buzzerActive = false;
        isBuzzerOn = false;
      } 
      else if (beepState == 3) {
        // Bunyi pertama selesai, mulai masuk jeda diam
        nyalakanBuzzer(false);
        beepState = 2; 
        beepInterval = 80; // Diam selama 80 milidetik
        beepTimer = m;
        isBuzzerOn = false;
      }
      else if (beepState == 2) {
        // Diam selesai, mulai mainkan bunyi kedua
        beepState = 1; 
        beepInterval = 100; 
        beepTimer = m;
      }
    }
  } else {
    buzzerActive = false;
  }

  // --- PEMROSES CAHAYA INDIKATOR (EFEK SILANG) ---
  if (isArmed) {
    // Mode Bahaya (Hijau): Jika buzzer nyala, hijau mati, merah nyala. (Efek alarm kelap-kelip merah cepat)
    digitalWrite(PIN_LED_HIJAU, isBuzzerOn ? LOW : HIGH);
    digitalWrite(PIN_LED_MERAH, isBuzzerOn ? HIGH : LOW);
  } else {
    // Mode Darat (Merah): Jika buzzer nyala, merah mati, hijau mati. (Efek merah berkedip kalem)
    digitalWrite(PIN_LED_HIJAU, LOW);
    digitalWrite(PIN_LED_MERAH, isBuzzerOn ? LOW : HIGH);
  }
}

// Bip Klasik (Blocking): Menghentikan total laju kode TX selama durasi (Hanya aman dipanggil di Setup/Darat)
void beepBlocking(unsigned int duration) {
  nyalakanBuzzer(true);    
  digitalWrite(PIN_LED_MERAH, LOW);  // Padamkan merah sebentar
  delay(duration);                   // Gembok Arduino total!
  nyalakanBuzzer(false);     
  digitalWrite(PIN_LED_MERAH, HIGH); // Nyalakan kembali
}

// ============================================================================
// 5. FUNGSI MANAJEMEN MEMORI EEPROM (RUANG 10 SLOT PESAWAT)
// ============================================================================
// Mencari tahu alamat laci penyimpanan untuk slot tertentu
int getSlotOffset(uint8_t slot) {
  return 100 + ((slot - 1) * 15); // Laci Slot 1 dimulai di byte ke-100, tiap laci muat 15 byte.
}

// Mengunci settingan pesawat saat ini ke dalam memori chip agar tidak hilang saat baterai dicabut
void saveConfigToSlot() {
  int offset = getSlotOffset(currentSlot); 
  EEPROM.update(offset + 7,  activeMode);                  // Mengamankan Mode Terbang
  EEPROM.update(offset + 8,  (uint8_t)(throttleTrim + 128)); // Mengamankan Trim Gas (Disuntik +128 agar menghindari minus)
  EEPROM.update(offset + 9,  (uint8_t)(rudderTrim + 128));   // Mengamankan Trim Ekor
  EEPROM.update(offset + 10, (uint8_t)(elevatorTrim + 128)); // Mengamankan Trim Sayap Ekor
  EEPROM.update(offset + 11, (uint8_t)(aileronTrim + 128));  // Mengamankan Trim Sayap Utama
  EEPROM.update(offset + 12, activeMotorType);             // Mengamankan Tipe Distribusi Listrik

  // Konser mini 3 nada sukses menyimpan
  beepBlocking(100); delay(150);
  beepBlocking(100); delay(150);
  beepBlocking(300);

  Serial.print(F("[EEPROM] Konfigurasi Berhasil Disimpan ke Slot: "));
  Serial.println(currentSlot);
}

// Memanggil/menarik memori pesawat tertentu dari EEPROM untuk digunakan terbang
void loadModel(uint8_t slot) {
  int offset = getSlotOffset(slot); 

  // Pengecekan Segel (0xAA). Jika segel belum ada, berarti ini slot baru dari pabrik
  if (EEPROM.read(offset) != 0xAA) {
    Serial.print(F("[EEPROM] Memformat Slot Baru: "));
    Serial.println(slot);
    EEPROM.update(offset, 0xAA);                     // Segel bahwa laci ini sudah ada yang punya
    EEPROM.update(offset + 1, random(20, 95));       // Ciptakan frekuensi channel dadakan (20-94)
    for (int i = 0; i < 5; i++) {
      EEPROM.update(offset + 2 + i, 'A' + random(0, 26)); // Ciptakan 5 huruf acak (Misal: "XZGHY") untuk alamat
    }
    EEPROM.update(offset + 7,  1);   // Isi dengan Normal Mode
    EEPROM.update(offset + 8,  128); // Nolkan Trim (128 berarti 0)
    EEPROM.update(offset + 9,  128); 
    EEPROM.update(offset + 10, 128); 
    EEPROM.update(offset + 11, 128); 
    EEPROM.update(offset + 12, 0);   // Isi dengan ESC Brushless Mode
  }

  // Tumpahkan isi EEPROM ke RAM untuk segera digunakan
  activeChannel = EEPROM.read(offset + 1);                
  for (int i = 0; i < 5; i++) {
    activePipe[i] = EEPROM.read(offset + 2 + i);          
  }
  activeMode      = EEPROM.read(offset + 7);              
  throttleTrim    = (int)EEPROM.read(offset + 8) - 128;   // Obati hasil suntikan +128 tadi agar bisa minus kembali
  rudderTrim      = (int)EEPROM.read(offset + 9) - 128;   
  elevatorTrim    = (int)EEPROM.read(offset + 10) - 128;  
  aileronTrim     = (int)EEPROM.read(offset + 11) - 128;  
  activeMotorType = EEPROM.read(offset + 12);             

  EEPROM.update(50, slot); // Catat di buku tamu (Alamat 50) pesawat mana yang terakhir kali dipakai
  currentSlot = slot;      // Perbarui posisi memori RAM saat ini

  // Konser pemberitahuan: Titik-titik suara sejumlah urutan slot memori
  for (int i = 0; i < slot; i++) {
    beepBlocking(100);
    delay(200);
  }

  Serial.print(F("[EEPROM] Berhasil Load Slot: ")); Serial.print(slot);
  Serial.print(F(" | Ch: ")); Serial.print(activeChannel);
  Serial.print(F(" | Mode: ")); Serial.print(activeMode);
  Serial.print(F(" | Motor: ")); Serial.println(activeMotorType == 0 ? F("ESC") : F("MOSFET"));
}

// ============================================================================
// 6. FUNGSI PROSES BINDING (MENJODOHKAN RX BARU SELAMA 10 DETIK)
// ============================================================================
void executeBindingProcess() {
  Serial.println(F("[BIND] Memulai Proses Pancaran Binding 10 Detik..."));
  radio.stopListening();             // Wajib kosongkan telinga sebelum jadi penyiar radio
  radio.setChannel(BIND_CH);         // Geser ke frekuensi siaran darurat (Channel 10)
  radio.openWritingPipe(BIND_ADDR);  // Siarkan dengan label amplop "BIND0"

  // Susun data perkenalan untuk dilempar ke udara
  BindPayload bp;
  memcpy(bp.pipeAddr, activePipe, 5); // Tawarkan RX untuk memakai password acak yang tadi dibuat di loadModel()
  bp.opChannel = activeChannel;       // Tawarkan RX untuk bertemu di frekuensi acak tersebut
  bp.checkSum  = activeChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4]; // Segel pakai XOR Math

  unsigned long bindStartTime = millis(); 

  // Pesta siaran berulang-ulang tanpa henti selama 10 Detik!
  while (millis() - bindStartTime < 10000) {
    radio.write(&bp, sizeof(BindPayload)); // Lempar data perkenalan

    // Efek Disko Silang Merah Hijau sambil Buzzer bernyanyi
    nyalakanBuzzer(true);
    digitalWrite(PIN_LED_MERAH, HIGH);
    digitalWrite(PIN_LED_HIJAU, LOW);
    delay(40);

    nyalakanBuzzer(false);
    digitalWrite(PIN_LED_MERAH, LOW);
    digitalWrite(PIN_LED_HIJAU, HIGH);
    delay(80);
  }

  // Waktu habis, kembalikan kosmetik LED
  digitalWrite(PIN_LED_HIJAU, LOW);
  beepBlocking(500);
  Serial.println(F("[BIND] Selesai! Kembali ke Frekuensi Operasional."));
}

// ============================================================================
// 7. FUNGSI KONVERSI STIK ANALOG JADUL MENJADI PRESISI DIGITAL 
// ============================================================================
// Fungsi ini membuang "deadzone" (ruang goyang kosong) di tengah tuas stik V930
int mapJoystick(int val, int lower, int middle, int upper) { 
  val = constrain(val, lower, upper);
  int output = 0;

  // Jika stik digeser menjauhi tengah ke arah bawah/kiri (Melewati batas 15 angka getar tangan)
  if (val < middle - 15) {
    output = map(val, lower, middle - 15, -127, 0); // Konversi murni jadi -127 s.d 0
  } 
  // Jika stik digeser menjauhi tengah ke arah atas/kanan 
  else if (val > middle + 15) {
    output = map(val, middle + 15, upper, 0, 127);  // Konversi murni jadi 0 s.d 127
  }

  return output; 
}

// ============================================================================
// 8. FUNGSI UTAMA: SETUP (HANYA DIPANGGIL 1x SAAT REMOTE DICETEK ON)
// ============================================================================
void setup() {
  // PENGAMANAN PENGUKURAN LISTRIK
  // Arduino Pro Mini 3.3V otomatis memakai rel VCC (3.3V) sebagai penggaris 1023 analogRead.
  // Jangan aktifkan EXTERNAL karena akan mengacaukan pembacaan ADC dari potensio stik (kecuali pin AREF disambung ke VCC).
  // analogReference(EXTERNAL);  

  Serial.begin(115200); 

  pinMode(PIN_LED_HIJAU, OUTPUT);
  pinMode(PIN_LED_MERAH, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // MENGAMANKAN TOMBOL DARAT:
  // Tombol fisik dipasang dari Pin 4 dan 5 lalu ke Kabel Ground (Tanpa tegangan positif luar).
  // Fungsi INPUT_PULLUP memaksa pin mengeluarkan 3.3V gaib dari dalam chip.
  // Ketika dipencet, listrik 3.3V gaib itu bocor ke tanah (Ground/LOW). 
  pinMode(PIN_BTN_L, INPUT_PULLUP);
  pinMode(PIN_BTN_R, INPUT_PULLUP);

  nyalakanBuzzer(false);
  digitalWrite(PIN_LED_MERAH, HIGH);
  digitalWrite(PIN_LED_HIJAU, LOW);

  delay(50); // Tarik nafas, tunggu listrik 3.3V gaib stabil merambat di tembaga tombol L & R

  // Semburkan angka acak dari derau kosmik (listrik liar) di udara pada dua pin analog nganggur
  // Digunakan sebagai bibit pembangkit alamat/frekuensi untuk pesawat baru.
  randomSeed(analogRead(A6) + analogRead(A7));

  // Mengintip buku tamu, pesawat mana yang kemarin siang terakhir diterbangkan
  uint8_t lastSlot = EEPROM.read(50);
  if (lastSlot < 1 || lastSlot > MAX_MODELS) {
    lastSlot = 1; // Jika ini chip perawan (isinya 255), paksa mulai dari pesawat nomer 1
  }
  loadModel(lastSlot); 

  // PENGUJIAN JANTUNG NRF24L01
  if (!radio.begin()) {
    Serial.println(F("[FATAL] Modul NRF24L01 Tidak Terdeteksi! Cek Kabel MISO/MOSI/SCK & Power 3.3V!"));
    while (1) {
      // Jika NRF rusak, matikan total remote. Beri kedipan horor tanda berkabung.
      digitalWrite(PIN_LED_MERAH, !digitalRead(PIN_LED_MERAH));
      delay(100);
    }
  }

  radio.setPALevel(RF24_PA_HIGH); // Bakar daya baterai agak besar agar antena memancar kuat
  radio.setDataRate(RF24_1MBPS);  // 1MBPS optimal menembus tembok hobi RC jarak pandang mata
  
  // ANTI TABRAKAN SINYAL:
  // Tunggu 500 mikrodetik, jika pesawat gagal membalas, tembak lagi. Coba sampai 3 kali maksimal.
  radio.setRetries(1, 3); 
  radio.enableDynamicPayloads();  // Izinkan kantong data melar memendek dinamis
  radio.enableAckPayload();       // AKTIFKAN TELEMETRI: Intip apakah pesawat menitipkan rapor kesehatan

  // TINDAKAN RAHASIA 10 DETIK (Tahan Bahu L & R Kiri Kanan serentak)
  if (digitalRead(PIN_BTN_L) == LOW && digitalRead(PIN_BTN_R) == LOW) {
    executeBindingProcess(); 
  }

  // Setelah siap tempur, putar siaran TX ke frekuensi spesifik milik pesawat yang dipanggil
  radio.stopListening();          // Tutup telinga sebelum memutar frekuensi
  radio.setChannel(activeChannel);
  radio.openWritingPipe(activePipe);
  delay(10);                      // Biarkan chip radio mengunci (PLL Lock) agar tidak bocor frekuensi

  isArmed = false;
  lastAckTime = millis(); 
  beepBlocking(200);
  initLCD();      // Nyalakan layar peninggalan pesawat V930
  Serial.println(F("[SYSTEM] TX Siap Tempur. Kondisi: DISARMED."));
}

// ============================================================================
// 9. FUNGSI UTAMA: LOOP (BERJALAN SEPERTI MESIN MOBIL MENDERU)
// ============================================================================
void loop() {
  unsigned long currentMillis = millis(); 
  handleAudioVisual(); // Urus suara buzzer dan nyala lampu di latar belakang tanpa mengganggu

  // --------------------------------------------------------------------------
  // A. SISTEM PERINGATAN BAHAYA DARI PESAWAT (TELEMETRI BATERAI LIPO RX)
  // --------------------------------------------------------------------------
  static unsigned long lastAlarmTime = 0; 
  if (isArmed) { // Hanya perdulikan teriakan pesawat jika mesin sedang menyala terbang
    if (rxTelem.batteryStatus == 1) { // Baterai Pesawat Lemah!
      if (currentMillis - lastAlarmTime > 2500) {
        beep(500); // Bip mengerikan panjang setiap 2.5 detik
        lastAlarmTime = currentMillis;
      }
    }
  }

  // --------------------------------------------------------------------------
  // B. DETAK JANTUNG PENGENDALIAN PESAWAT (EKSEKUSI TIAP 30 MILIDETIK)
  // --------------------------------------------------------------------------
  if (currentMillis - lastTxTime >= TX_INTERVAL) {
    lastTxTime = currentMillis; 

    // 1. Sedot tegangan voltase riil dari gerakan tangan Anda di atas potensio tuas V930 (0 - 1023)
    int rawThr  = analogRead(PIN_THR);  
    int rawRud  = analogRead(PIN_RUD);  
    int rawEle  = analogRead(PIN_ELE);  
    int rawAil  = analogRead(PIN_AIL);  
    int rawTrim = analogRead(PIN_TRIM); // Deteksi pencetan resistor dari papan tombol TRIM

    // 2. Baca apakah jari Anda meremas pundak remote (Tombol bahu L/R)
    bool btnL = (digitalRead(PIN_BTN_L) == LOW);
    bool btnR = (digitalRead(PIN_BTN_R) == LOW);

    // ------------------------------------------------------------------------
    // C. LOGIKA TOMBOL R KANAN (TOMBOL MAUT: ARMING / DISARMING MOTOR)
    // ------------------------------------------------------------------------
    static unsigned long btnRPressTime = 0; 
    static bool btnRWasPressed = false;     
    static bool disarmHandled  = false;     

    if (btnR && !btnL) {
      if (!btnRWasPressed) {
        btnRPressTime  = currentMillis;
        btnRWasPressed = true;
        disarmHandled  = false;

        if (!isArmed) {
          // SYARAT MUTLAK ARMING: Tuas Gas Wajib Mentok Bawah (Nol)
          if (rawThr > 40) {
            // Jika ada orang rese yang menghidupkan mesin sambil menekan gas, TOLAK!
            beepBlocking(80); delay(80); beepBlocking(80); 
            Serial.println(F("[WARNING] Ditolak! Tangan Nakal, Turunkan Tuas Gas Dulu!"));
          } else {
            isArmed = true; // Buka gerbang listrik motor di pesawat
            beep(600);      // Bip panjang tanda Bahaya dimulai
            Serial.println(F("[MOTOR] ARMED!"));
          }
          disarmHandled = true; 
        }
        else if (isArmed && rawThr <= 40) {
          // Klik Disarm di Darat: Matikan instan
          isArmed = false;
          beep(200);
          Serial.println(F("[MOTOR] DISARMED (Klik Instan Darat)"));
          disarmHandled = true;
        }
      }

      // EMERGENCY DISARM UDARA: Jika mesin menyala dan gas sedang ditarik tinggi, 
      // klik pelan tombol R akan diabaikan (anti kesenggol jari). 
      // Wajib tahan dengan kuat 0.6 detik berturut-turut untuk mematikan motor!
      if (isArmed && !disarmHandled && (currentMillis - btnRPressTime > 600)) {
        isArmed = false;
        beep(400);
        Serial.println(F("[MOTOR] EMERGENCY DISARM! (Tahan 0.6s)"));
        disarmHandled = true;
      }
    } else if (!btnR) {
      btnRWasPressed = false; 
    }

    // ------------------------------------------------------------------------
    // D. LOGIKA TOMBOL L KIRI (TOMBOL CERDAS: RESET TRIM & MASUK MENU SISTEM)
    // ------------------------------------------------------------------------
    static bool btnLWasPressed = false;
    static unsigned long btnLPressTime = 0;
    static unsigned long lastMenuTime = 0;
    static bool menuExecuted = false;
    const unsigned long SHORT_CLICK_MAX = 400; // Kalau ditahan < 400ms artinya dia mau klik cepat (Reset)

    if (btnL && !btnR) {
      if (!btnLWasPressed) {
        btnLWasPressed = true;
        btnLPressTime = currentMillis;
        menuExecuted = false;

        if (isArmed) {
          // Skenario Darurat Udara: Pilot panik pesawatnya miring parah karena salah pencet trim.
          // Pilot memencet tombol L -> Seketika semua trim dikembalikan ke titik Nol!
          throttleTrim = 0; rudderTrim = 0; elevatorTrim = 0; aileronTrim = 0;
          beep(400);
          Serial.println(F("[TRIM] Diselamatkan! Reset Instan di Udara!"));
          menuExecuted = true; 
        }
      }

      // Skenario Mekanik di Darat: Tahan L sambil Pencet Trim
      if (!isArmed && !menuExecuted) {
        if (rawTrim < 900 && (currentMillis - lastMenuTime > 500)) {
          if (rawTrim > 460 && rawTrim < 530) { // Aileron Kanan
            // Menu Ganti Pesawat (Memanggil memori slot berikutnya)
            currentSlot = (currentSlot % MAX_MODELS) + 1;
            loadModel(currentSlot);
            radio.setChannel(activeChannel); radio.openWritingPipe(activePipe);
            triggerAnimSlot(currentSlot);
            menuExecuted = true; lastMenuTime = currentMillis;
          } 
          else if (rawTrim > 380 && rawTrim < 450) { // Elevator Bawah
            // Menu Ganti Jalur Listrik (Mosfet / ESC)
            activeMotorType = (activeMotorType == 0) ? 1 : 0;
            triggerAnimMotor(activeMotorType);
            if (activeMotorType == 0) beep(150); 
            else beepDouble();                   
            menuExecuted = true; lastMenuTime = currentMillis;
          }
          else if (rawTrim > 300 && rawTrim < 360) { // Elevator Atas
            // Menu Ganti Aerodinamika (Mode 1-4)
            activeMode = (activeMode % 4) + 1;
            beepBlocking(400); delay(200);
            for (int i = 0; i < activeMode; i++) { beepBlocking(100); delay(150); }
            menuExecuted = true; lastMenuTime = currentMillis;
          } 
          else if (rawTrim > 540 && rawTrim < 610) { // Aileron Kiri
            // Simpan paksa modifikasi saat ini ke dalam EEPROM
            saveConfigToSlot();
            menuExecuted = true; lastMenuTime = currentMillis;
          }
        }
      }
    } 
    else if (!btnL) {
      if (btnLWasPressed) {
        if (!isArmed && !menuExecuted) {
          unsigned long holdDuration = currentMillis - btnLPressTime;
          
          if (holdDuration < SHORT_CLICK_MAX) {
            // L hanya diklik pelan di darat -> Eksekusi Reset Trim
            throttleTrim = 0; rudderTrim = 0; elevatorTrim = 0; aileronTrim = 0;
            beep(400);
            Serial.println(F("[TRIM] Reset (Klik Pendek di Darat)!"));
          } 
        }
        btnLWasPressed = false; 
      }
    }

    // ------------------------------------------------------------------------
    // E. DETEKSI DINI 8 TOMBOL TRIM PENYUMBANG ARUS LADDER
    // ------------------------------------------------------------------------
    static unsigned long lastTrimProcess = 0; 
    // Murni hanya boleh pencet Trim kalau L tidak ditahan (!btnL)
    if (!btnL && rawTrim < 900 && (currentMillis - lastTrimProcess > 150)) {
      bool trimChanged = false; 

      if (rawTrim < 40) {
        throttleTrim = constrain(throttleTrim + 2, -127, 127); // Trim Naik (Dibatasi jangan sampai jebol +128)
        trimChanged = true;
      } else if (rawTrim > 80 && rawTrim < 130) {
        throttleTrim = constrain(throttleTrim - 2, -127, 127); 
        trimChanged = true;
      } else if (rawTrim > 140 && rawTrim < 200) {
        rudderTrim = constrain(rudderTrim - 2, -127, 127);     
        trimChanged = true;
      } else if (rawTrim > 230 && rawTrim < 290) {
        rudderTrim = constrain(rudderTrim + 2, -127, 127);     
        trimChanged = true;
      } else if (rawTrim > 300 && rawTrim < 360) {
        elevatorTrim = constrain(elevatorTrim + 2, -127, 127); 
        trimChanged = true;
      } else if (rawTrim > 380 && rawTrim < 450) {
        elevatorTrim = constrain(elevatorTrim - 2, -127, 127); 
        trimChanged = true;
      } else if (rawTrim > 460 && rawTrim < 530) {
        aileronTrim = constrain(aileronTrim + 2, -127, 127);   
        trimChanged = true;
      } else if (rawTrim > 540 && rawTrim < 610) {
        aileronTrim = constrain(aileronTrim - 2, -127, 127);   
        trimChanged = true;
      }

      if (trimChanged) {
        beep(30); // Bunyi krik krik halus saat memijat tombol trim
        lastTrimProcess = currentMillis;
      }
    }

    // ------------------------------------------------------------------------
    // F. MATEMATIKA STIK GIMBAL (BERAT-BERSIH-FINAL)
    // ------------------------------------------------------------------------
    // 1. Gas murni
    int thrMurni = map(rawThr, 15, 680, 0, 255); 
    if (rawThr <= 20) thrMurni = 0;              
    thrMurni = constrain(thrMurni, 0, 255);      
    
    int thrFinal = constrain(thrMurni + throttleTrim, 0, 255); 
    
    // 2. Tiga sayap murni dengan pengikisan area tengah agar tidak gemetar di LCD
    int rudMurni = mapJoystick(rawRud, 11, 504, 1000);
    int eleMurni = mapJoystick(rawEle, 8,  515, 1000);
    int ailMurni = mapJoystick(rawAil, 10, 513, 1012);

    // 3. Suntikkan nilai trim hasil pekerjaan mekanik, lalu potong mentok jika > 127 (Overflow Protection)
    int rudFinal = constrain(rudMurni - rudderTrim, -127, 127);
    int eleFinal = constrain(eleMurni + elevatorTrim, -127, 127);
    int ailFinal = constrain(ailMurni - aileronTrim, -127, 127);

    // 4. Susun kedalam bungkus paket rapi bernama "txData" siap lempar
    txData.isArmed   = isArmed ? 1 : 0;
    txData.auxFlag   = activeMode;
    txData.motorType = activeMotorType;

    // Paksa mesin di pesawat tidak menerima sinyal gas jika remote masih Disarmed (Amankan Nyawa Pesawat)
    txData.throttle = isArmed ? (uint8_t)constrain(thrFinal, 0, 255) : 0; 
    txData.rudder   = (int8_t)rudFinal;
    txData.elevator = (int8_t)eleFinal;
    txData.aileron  = (int8_t)ailFinal;

    // ------------------------------------------------------------------------
    // G. MENGUDARA! (TEMBAKKAN SINYAL RADIO KE LANGIT BEBAS)
    // ------------------------------------------------------------------------
    if (radio.write(&txData, sizeof(ControlData))) {
      // BINGO! Sinyal kendali berhasil menyentuh kumparan antena pesawat.
      
      // Jika ternyata pesawat merespon dengan membawa paket titipan berisi kesehatan baterai:
      if (radio.isAckPayloadAvailable()) {
        radio.read(&rxTelem, sizeof(TelemetryData)); 
        lastAckTime = currentMillis; 
      }
    } else {
      // YAH PUTUS! (Pesawat terhalang pohon bata, atau sudah sangat jauh)
      // Buang antrian kemacetan memori NRF agar proses tembak 30ms berikutnya lancar!
      radio.flush_tx(); 
    }

    // Hukum Kejam Hilang Sinyal (Hanya toleransi 1 detik / 1000ms tanpa balasan)
    bool isRxConnected = (currentMillis - lastAckTime < 1000);
    
    if (!isRxConnected) {
      rxTelem.batteryStatus = 0; // Mulut pesawat tertutup (Bungkam alarm palsu yang nyangkut di remote)
    }

    // ========================================================================
    // --- KESEHATAN LISTRIK DALAM REMOTE TX SENDIRI (TP4056 LIPO) ---
    // ========================================================================
    // Trik Cerdas Crosstalk (Sedot dulu voltase pin bekas trim ke tanah A6, lalu buang)
    analogRead(PIN_BAT_TX); 
    // Baru kemudian sedot tegangan baterai TX murni dan bersih!
    int rawBatTx = analogRead(PIN_BAT_TX); 
    int txBatLevel = 0; 
    
    // Klasifikasi daya baterai TX sisa berapa batangan
    if (rawBatTx > 600) txBatLevel = 4;      // Hijau Penuh
    else if (rawBatTx > 570) txBatLevel = 3; 
    else if (rawBatTx > 540) txBatLevel = 2; 
    else if (rawBatTx > 515) txBatLevel = 1; // Merah Kritis
    else txBatLevel = 0;                     // Pingsan!

    // --- ALARM REMOTE TX MAU MATI ---
    static bool alarmBatreNyangkut = false; 
    
    if (txBatLevel == 0 && !buzzerActive) {
      unsigned long critTime = currentMillis % 1000; 
      // Ritme Kepanikan Bip..Bip.......Bip..Bip
      if ((critTime < 100) || (critTime > 200 && critTime < 300)) {
        nyalakanBuzzer(true);
        alarmBatreNyangkut = true;
      } else {
        nyalakanBuzzer(false);
        alarmBatreNyangkut = false;
      }
    } 
    else if (alarmBatreNyangkut && txBatLevel > 0) {
      // Cabut charger, batre waras kembali. Hentikan tangisan buzzer.
      nyalakanBuzzer(false);
      alarmBatreNyangkut = false;
    }
    
    // --- INTIP PROSES CHARGING TP4056 ---
    // Jika kabel USB dicolok, TP4056 akan menarik pin A7 menempel ke Ground (Listrik < 500)
    bool isCharging = (analogRead(PIN_CHRG) < 500);

    // KETUK PINTU LCD H8805A, LALU TUMPAHKAN SEMUA DATA UNTUK DIGAMBAR
    updateLCD(isArmed, thrMurni, txBatLevel, isRxConnected, 
              activeMode, activeMotorType, 
              throttleTrim, rudderTrim, elevatorTrim, aileronTrim, 
              rudMurni, eleMurni, ailMurni, isCharging);

    // ------------------------------------------------------------------------
    // H. SERIAL MONITOR SOPAN (HEMAT MEMORI DI UDARA - EDITAN BERHASIL!)
    // ------------------------------------------------------------------------
    static int lastThr = 0, lastRud = 0, lastEle = 0, lastAil = 0, lastTrmVal = 0;
    static bool lastArmStatus = false;
    bool harusCetak = false;

    // Jika terjadi guncangan perubahan posisi yang masif pada stik...
    if (abs(txData.throttle - lastThr) > 2) harusCetak = true;
    if (abs(txData.rudder - lastRud) > 2) harusCetak = true;
    if (abs(txData.elevator - lastEle) > 2) harusCetak = true;
    if (abs(txData.aileron - lastAil) > 2) harusCetak = true;
    if (isArmed != lastArmStatus) harusCetak = true;
    if (rawTrim < 900 && abs(rawTrim - lastTrmVal) > 10) harusCetak = true;
    
    // Syarat ini membuat cetakan tulisan Serial dinonaktifkan total (False Mutlak).
    // Trik "&& false" ini adalah teknik master C++ yang memaksa compiler IDE Arduino 
    // membuang (mengabaikan total / mengeliminasi mati) barisan kode Print memori.
    // Hasilnya: Kode mengudara dengan sempurna dan RAM Arduino menjadi sangat lega dan ringan!
    if (harusCetak && false) { 
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