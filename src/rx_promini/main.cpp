/*
 * ============================================================================
 * FIRMWARE RECEIVER (RX) - RC UNIVERSAL HYBRID (ESC & MOSFET)
 * ============================================================================
 * Fitur: Auto-Binding, No-Pulses Failsafe (Motor Beep), Slew-Rate Limiter, 
 *        Throttle Deadband Lock, 4 Mode Terbang, Penggerak Motor Hybrid,
 *        dan AUTO-CELL DETECT (Deteksi Baterai 1S/2S/3S Otomatis).
 * 
 * ============================================================================
 * PANDUAN INSTALASI HARDWARE (WIRING GUIDE)
 * ============================================================================
 * 1. KONEKSI MODUL NRF24L01:
 *    - VCC  -> 3.3V (WAJIB 3.3V, NRF akan jebol jika masuk 5V!)
 *    - GND  -> GND Arduino
 *    - CE   -> Pin 9  Arduino
 *    - CSN  -> Pin 10 Arduino
 *    - SCK  -> Pin 13 Arduino
 *    - MOSI -> Pin 11 Arduino
 *    - MISO -> Pin 12 Arduino
 * 
 * 2. KONEKSI SENSOR BATERAI UNIVERSAL (1S, 2S, 3S)
 *    Skema Pemasangan (Voltage Divider):
 *    Kutub Positif Baterai (+) ---> [Resistor 1] ---> Pin A0 ---> [Resistor 2] ---> GND
 * 
 *    KARENA MENGGUNAKAN PENGGARIS INTERNAL 1.1V:
 *    Nilai resistor untuk Arduino Pro Mini 3.3V maupun 5V adalah SAMA:
 *    - R1 (Baterai ke A0) : 120k Ohm (Kode SMD: 124 atau 1203)
 *    - R2 (A0 ke GND)     : 10k Ohm  (Kode SMD: 103)
 *    
 *    (Catatan: Konfigurasi ini aman membaca tegangan baterai hingga 14.3 Volt.
 *     Dan kebal terhadap voltage-drop saat memakai baterai 1S).
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>       // Library jalur komunikasi serial berkecepatan tinggi untuk NRF
#include <RF24.h>      // Library khusus otak pengendali modul radio NRF24L01
#include <Servo.h>     // Library pembuat sinyal PWM presisi (1000us-2000us) untuk ESC dan Servo Sayap
#include <EEPROM.h>    // Library untuk menulis/membaca data ke memori permanen Arduino
#include <avr/wdt.h>   // Panggil Anjing Penjaga (Watchdog)

// ============================================================================
// 1. KONFIGURASI PIN HARDWARE (PRO MINI)
// ============================================================================
// Catatan: Pin 5 dan 6 memiliki fitur Hardware PWM bawaan ATmega328P, 
// sangat krusial untuk fitur analogWrite() jika menggunakan MOSFET.
#define PIN_RADIO_CE   9  // Pin Chip Enable: Menghidupkan/mematikan pemancar NRF
#define PIN_RADIO_CSN  10 // Pin Chip Select Not: Memilih NRF sebagai lawan bicara SPI
#define PIN_MOTOR_1    6  // Output Motor Kiri / Utama (Wajib pin ber-PWM Hardware ~)
#define PIN_MOTOR_2    5  // Output Motor Kanan (Wajib pin ber-PWM Hardware ~)
#define PIN_SERVO_3    4  // Output Servo Digital (Bebas, bisa pin digital biasa)
#define PIN_SERVO_4    7  // Output Servo Digital (Bebas, bisa pin digital biasa)
#define PIN_LED        3  // Indikator Status RX (Menyala jika Armed, Mati jika Disarmed)
#define PIN_BUZZER     8  // Buzzer aktif pencari pesawat (Lost Model Finder)
#define PIN_VBATT      A0 // Sensor Tegangan Baterai (Membaca voltase analog 0-1.1V)

const byte BIND_ADDR[6] = "BIND0"; // "Pintu Darurat" statis untuk pertama kali berkenalan dengan TX
const uint8_t BIND_CH   = 10;      // Saluran frekuensi darurat (Channel 10) khusus untuk proses jabat tangan

// ============================================================================
// 2. STRUKTUR DATA (TEMPLATE BUNGKUSAN DATA RADIO)
// ============================================================================
// Paket Data untuk Binding (Proses Jabat Tangan Pertama)
struct BindPayload {
  byte pipeAddr[5]; // TX memberikan RX alamat permanen yang baru (5 karakter rahasia)
  uint8_t opChannel; // TX memberi tahu RX frekuensi permanen untuk dipakai terbang
  uint8_t checkSum;  // Angka verifikasi anti-hacker/anti-glitch agar RX tidak salah bind
};

// Paket Data Kendali Utama dari TX (Wajib 100% identik ukurannya dengan di TX)
struct ControlData {
  uint8_t throttle;  // Tipe uint8_t: Angka hanya bisa 0 sampai 255 (pas untuk PWM)
  int8_t  rudder;    // Tipe int8_t: Angka bisa MINUS (-127 sampai +127) untuk belok Kiri/Kanan
  int8_t  elevator;  // Tipe int8_t: Angka bisa MINUS (-127 sampai +127) untuk Naik/Turun
  int8_t  aileron;   // Tipe int8_t: Angka bisa MINUS (-127 sampai +127) untuk Putar Sayap
  uint8_t isArmed;   // 0 = Kunci Motor (Aman), 1 = Motor Hidup (Siap Terbang)
  uint8_t auxFlag;   // Mode Terbang: 1=Normal, 2=Diferensial, 3=V-Tail, 4=Mobil
  uint8_t motorType; // Tipe Penggerak: 0 = ESC Brushless (Servo signal), 1 = MOSFET (PWM signal)
};
ControlData rxData; // Membuat kotak penampung fisik di RAM dengan cetakan ControlData

// Paket Data Telemetri (Data yang dititipkan RX untuk diambil oleh TX)
struct TelemetryData {
  uint8_t batteryStatus;  // 0 = Normal, 1 = Low Batt (Memicu alarm di remote)
  uint8_t failsafeActive; // 0 = Aman, 1 = Failsafe (Melapor ke TX jika RX hilang arah)
};
TelemetryData txTelem;

// ============================================================================
// 3. DEKLARASI OBJEK & VARIABEL GLOBAL (STATUS SISTEM)
// ============================================================================
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN); // Menciptakan objek radio
Servo servo3; // Menciptakan mesin pembuat sinyal untuk Servo Elevator
Servo servo4; // Menciptakan mesin pembuat sinyal untuk Servo Aileron
Servo esc1;   // Menciptakan mesin pembuat sinyal khusus untuk menipu ESC Brushless 1
Servo esc2;   // Menciptakan mesin pembuat sinyal khusus untuk menipu ESC Brushless 2

byte myPipe[5];       // Variabel penyimpan memori alamat dari EEPROM
uint8_t myChannel;    // Variabel penyimpan memori channel dari EEPROM
bool isBound = false; // Bendera status: Apakah RX ini sudah punya pasangan hidup?

unsigned long lastRecvTime = 0; // Stopwatch: Mencatat kapan terakhir kali RX disapa TX
bool failsafeActive = true;     // Bendera keselamatan: Sengaja dibuat 'true' saat baru nyala agar motor tidak langsung loncat
bool escsAttached = false;      // Bendera saklar: Memantau siapa yang sedang menguasai Pin 5 & 6 (Servo.h atau analogWrite?)
int currentThrottle = 0;        // Memori posisi gas riil saat ini (Digunakan oleh Slew-Rate Limiter)

uint16_t lowBatteryThreshold = 0; // Target batas bawah (mV) pemicu alarm, ditentukan otomatis di setup
uint8_t detectedCells = 0;        // Menyimpan status jumlah sel baterai yang dicolok (1, 2, atau 3)

// ============================================================================
// 4. FUNGSI BINDING (PROSES JABAT TANGAN PERTAMA KALI)
// ============================================================================
void enterBindingMode() {
  Serial.println(F("[BIND] Masuk Mode Binding. Menunggu TX..."));
  radio.setChannel(BIND_CH);          // Pindah ke saluran darurat (Channel 10)
  radio.openReadingPipe(1, BIND_ADDR); // Buka pintu menggunakan kunci statis "BIND0"
  radio.startListening();             // Pasang telinga, tunggu TX berteriak

  // Sistem akan "Terkunci" di dalam loop ini selama isBound masih false
  while (!isBound) {
    if (radio.available()) {
      BindPayload bp; // Buat kotak kosong sementara
      radio.read(&bp, sizeof(BindPayload)); // Sedot isi data dari udara ke dalam kotak
      
      // LOGIKA CHECKSUM (Keamanan Tingkat Tinggi)
      // XOR (^) digunakan untuk mengacak data. Jika TX dan RX menggunakan rumus XOR yang sama,
      // hasilnya harus cocok. Jika meleset, berarti itu sinyal nyasar atau corrupt.
      uint8_t calcSum = bp.opChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4];
      
      if (bp.checkSum == calcSum) { // Jika verifikasi data cocok
        myChannel = bp.opChannel;   // Simpan channel permanen ke RAM
        memcpy(myPipe, bp.pipeAddr, 5); // Copy 5 byte alamat rahasia ke RAM
        
        // SIMPAN KE MEMORI PERMANEN (EEPROM)
        // EEPROM tidak akan hilang walaupun baterai dicabut
        EEPROM.update(0, 0xBB); // Byte ke-0 diisi 0xBB (187) sebagai tanda/segel bahwa RX sudah di-bind
        EEPROM.update(1, myChannel); // Byte ke-1 menyimpan saluran terbang
        for(int i=0; i<5; i++) EEPROM.update(2+i, myPipe[i]); // Byte 2 s.d 6 menyimpan 5 huruf alamat
        
        isBound = true; // Buka kunci loop
        Serial.println(F("[BIND] Sukses! Data tersimpan di EEPROM."));
        
        // Feedback suara ke pengguna
        tone(PIN_BUZZER, 2000, 1000); delay(1000);
      }
    }
    // Indikator visual: LED berkedip sangat cepat menandakan RX butuh pasangan
    digitalWrite(PIN_LED, millis() % 200 < 100);
  }
}

// ============================================================================
// 5. FUNGSI UTAMA: SETUP (DIJALANKAN SEKALI SAAT DINYALAKAN)
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Trik Rahasia Pengukuran Presisi 1S
  // Memutus rujukan pembacaan voltase dari VCC yang bisa drop, 
  // dan menguncinya ke komponen internal ATmega328P yang paten di 1.1 Volt.
  analogReference(INTERNAL); 
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);

  // Inisialisasi NRF24L01 (Pemeriksaan Hardware)
  if (!radio.begin()) { 
    Serial.println(F("[FATAL] Radio NRF24 Rusak atau Kabel Lepas!")); 
    while(1); // Sistem mengunci diri mati total jika NRF tidak merespon SPI
  }
  radio.setPALevel(RF24_PA_HIGH);  // Setel kekuatan pancaran (Low, High, Max)
  radio.setDataRate(RF24_1MBPS);   // Setel kecepatan transfer. 1MBPS optimal untuk jarak dan kecepatan
  radio.enableDynamicPayloads();   // Izinkan panjang data radio bervariasi (tidak kaku)
  radio.enableAckPayload();        // AKTIFKAN TELEMETRI PINTAR (Titip pesan ke sinyal balik otomatis)

  // --- CEK MEMORI BINDING & FITUR RESET BINDING ---
  pinMode(2, INPUT_PULLUP); // Siapkan Pin D2. Secara default terbaca HIGH karena resistor internal

  // Cek jika Segel EEPROM kosong (!= 0xBB) ATAU mekanik menjumper Pin D2 ke Ground (LOW)
  if (EEPROM.read(0) != 0xBB || digitalRead(2) == LOW) {
    enterBindingMode(); // Masuk mode pencarian jodoh
  } 
  else {
    // Jika segel utuh, bongkar isi brankas EEPROM ke RAM
    myChannel = EEPROM.read(1);
    for(int i=0; i<5; i++) myPipe[i] = EEPROM.read(2+i);
    isBound = true;
    Serial.println(F("[SYSTEM] Load Data Binding dari EEPROM."));
  }

  // --- KECERDASAN AUTO-CELL DETECT (Deteksi Voltase Otomatis Awal) ---
  // Membaca voltase pertama kali colok baterai. 
  // Angka 14800 didapat dari (1.1 Volt x Rasio Resistor 120k/10k)
  analogRead(PIN_VBATT); // Baca sekali untuk menstabilkan ADC
  delay(10); // Jeda sebentar agar ADC stabil
  uint16_t initialMilliVolt = map(analogRead(PIN_VBATT), 0, 1023, 0, 14300);
  
  // Logika Pemisahan Sel (Threshold)
  if (initialMilliVolt < 4600) {
    detectedCells = 1; // Jika voltase < 4.35V, pasti baterai 1S
    lowBatteryThreshold = 3500; // Alarm bunyi saat 1S menyentuh 3.5V
  } else if (initialMilliVolt < 8500) {
    detectedCells = 2; // Jika voltase < 8.5V, pasti baterai 2S
    lowBatteryThreshold = 7000; // Alarm bunyi saat 2S menyentuh 7.0V (3.5V/sel)
  } else {
    detectedCells = 3; // Sisanya pasti baterai 3S
    lowBatteryThreshold = 10500; // Alarm bunyi saat 3S menyentuh 10.5V (3.5V/sel)
  }

  Serial.print(F("[BATT DETECT] Lipo "));
  Serial.print(detectedCells);
  Serial.print(F("S Terdeteksi. Set Alarm: "));
  Serial.println(lowBatteryThreshold);

  // TRANSISI FREKUENSI DARI DARURAT KE PERMANEN
  radio.stopListening(); // Wajib tutup telinga sebelum pindah saluran
  delay(10);             // Jeda agar chip NRF bernapas sesaat

  radio.setChannel(myChannel);      // Pindah ke channel rahasia
  radio.openReadingPipe(1, myPipe); // Gembok pintu dengan password alamat rahasia
  radio.startListening();           // Pasang telinga kembali
  
  // Titipkan data telemetri pertama agar tidak error saat TX pertama nanya
  txTelem.batteryStatus = 0;
  txTelem.failsafeActive = 1; 
  radio.writeAckPayload(1, &txTelem, sizeof(TelemetryData));
  
  // Aktifkan Servo Sayap
  servo3.attach(PIN_SERVO_3);
  servo4.attach(PIN_SERVO_4);
  
  wdt_enable(WDTO_250MS); 
  
  Serial.println(F("[SYSTEM] RX Siap Terbang (Watchdog Aktif)."));
}

// ============================================================================
// 6. FUNGSI UTAMA: LOOP (OTAK REAL-TIME PENERBANGAN)
// ============================================================================
void loop() {
  wdt_reset(); // Beri makan anjing setiap putaran agar tidak me-reset Arduino!
  unsigned long currentMillis = millis(); // Baca stopwatch utama Arduino

  // --- A. PENERIMAAN DATA DARI TX ---
  if (radio.available()) {
    radio.read(&rxData, sizeof(ControlData)); 
    lastRecvTime = currentMillis;             
    
    // --- TAMBAHAN ANTI-GHOST PACKET (NRF LOCKUP DETECTOR) ---
    static int packetCount = 0;
    static unsigned long lastCountTime = 0;
    packetCount++; // Hitung setiap paket yang masuk
    
    // Evaluasi setiap 1 detik
    if (currentMillis - lastCountTime >= 1000) {
      if (packetCount > 100) {
        // MUSTAHIL! TX hanya kirim 33 paket per detik. NRF pasti MACET!
        Serial.println(F("[FATAL] NRF HANG TERDETEKSI! Memaksa Watchdog Reset..."));
        while(1); // Terjebak di sini agar Watchdog menggigit dan mereset sistem secara fisik!
      }
      packetCount = 0; // Reset hitungan untuk detik berikutnya
      lastCountTime = currentMillis;
    }
    // ---------------------------------------------------------

    // PEMULIHAN DARI FAILSAFE 
    if (failsafeActive) { 
      failsafeActive = false; 
      rxData.isArmed = 0;     
      Serial.println(F("[RADIO] Link Tersambung Kembali. Motor Terkunci Otomatis."));
    }
    
    // --- EVALUASI BATERAI SECARA REAL-TIME ---
    // Diulang terus menerus sambil terbang untuk menangkap jika tegangan anjlok
    uint16_t currentMilliVolt = map(analogRead(PIN_VBATT), 0, 1023, 0, 14300);
    
    // Jika voltase anjlok melewati batas alarm yang ditetapkan saat setup()
    if (currentMilliVolt <= lowBatteryThreshold) {
      txTelem.batteryStatus = 1; // Nyalakan bendera peringatan baterai
    } else {
      txTelem.batteryStatus = 0; 
    }

    txTelem.failsafeActive = 0; // Lapor ke TX bahwa RX sedang baik-baik saja
    radio.flush_tx(); // Buang sisa antrean lama agar tidak macet
    // Masukkan data telemetri ke dalam "ruang tunggu", agar otomatis terkirim balik saat TX ngobrol lagi
    radio.writeAckPayload(1, &txTelem, sizeof(TelemetryData)); 
  }

  // --- B. LOGIKA FAILSAFE (PENGAMAN KEHILANGAN SINYAL) ---
  // Jika selisih waktu sekarang dengan detak jantung terakhir melebihi 500 milidetik (0.5 Detik)
  if (currentMillis - lastRecvTime > 500) {
    if (!failsafeActive) {
      Serial.println(F("[FAILSAFE] Sinyal Hilang! Memicu No-Pulses Failsafe."));
      failsafeActive = true;
      radio.flush_tx();
      rxData.isArmed = 0;  // Kunci perizinan putaran motor
      rxData.throttle = 0; // Tarik gas kembali ke titik nol secara memori
      
      // NO-PULSES FAILSAFE UNTUK ESC:
      // Cabut total hak library Servo dari pin motor. 
      // ESC Brushless membaca ini sebagai "Kabel Sinyal Putus". 
      // Reaksi ESC pabrikan: Mematikan motor, lalu membunyikan dinamo "Bip.. Bip.. Bip.." 
      esc1.detach(); esc2.detach(); 
      escsAttached = false;
      
      // Matikan listrik total untuk mode MOSFET
      digitalWrite(PIN_MOTOR_1, LOW); 
      digitalWrite(PIN_MOTOR_2, LOW);
      
      // Netralkan posisi sayap ke derajat 90 (Tengah).
      // Tujuannya agar pesawat tidak memutar/menukik tajam, melainkan melayang bebas (Gliding) turun perlahan.
      servo3.write(90); servo4.write(90);
    }
    txTelem.failsafeActive = 1; // Update status telemetri gagal (Bersiap jika sinyal masuk sedikit)

    // LOST MODEL FINDER: Alarm pencarian jika hilang > 15 detik (15000 ms)
    if (currentMillis - lastRecvTime > 15000) {
      // Modulo 2000: Bikin pola putaran 2 detik. 
      // Jika sisa bagi < 300ms, bunyi! Sisanya 1700ms, diam! (Menghasilkan bunyi Bip... putus-putus)
      if ((currentMillis % 2000) < 300) tone(PIN_BUZZER, 2500);
      else noTone(PIN_BUZZER);
    } else {
      noTone(PIN_BUZZER);
    }
  }

  // --- C. EKSEKUSI PENGGERAK (HANYA BERJALAN JIKA SINYAL AMAN) ---
  if (!failsafeActive) {
    
    // 1. THROTTLE SLEW-RATE LIMITER (Peredam Lonjakan Berbasis Waktu Mutlak)
    int targetThrottle = rxData.isArmed ? rxData.throttle : 0;
    
    if (targetThrottle > currentThrottle) {
      // Hanya naikkan putaran mesin jika interval waktu sudah terpenuhi
      static unsigned long lastSlewTime = 0;
      const unsigned int SLEW_INTERVAL = 5; // Jeda waktu (5 milidetik)
      const int SLEW_STEP = 2;              // Ukuran pijakan gas (naik 2 angka)
      
      if (currentMillis - lastSlewTime >= SLEW_INTERVAL) {
         currentThrottle += SLEW_STEP; 
         if (currentThrottle > targetThrottle) currentThrottle = targetThrottle; // Cegah kebablasan
         lastSlewTime = currentMillis; // Reset stopwatch akselerasi
      }
    } else {
       // KESELAMATAN MUTLAK: Jika gas diturunkan, WAJIB instan seketika itu juga!
       currentThrottle = targetThrottle; 
    }

    int outM1 = 0;  // Variabel ember penampung hasil hitung akhir Motor 1
    int outM2 = 0;  // Variabel ember penampung hasil hitung akhir Motor 2
    int outS3 = 90; // Variabel ember penampung derajat Servo 3
    int outS4 = 90; // Variabel ember penampung derajat Servo 4

    // 2. THROTTLE DEADBAND LOCK (Gembok Pengaman Bawah)
    // Jika tuas gas posisi nyaris tertutup (< 10), paksa motor murni berhenti.
    // Ini mencegah motor baling-baling ikut berputar saat Anda sedang ngetes belok sayap di darat (karena efek mixer).
    if (currentThrottle < 10) {
      outM1 = 0;
      outM2 = 0;
      // Sayap (Elevator/Aileron) tetap boleh merespon
      outS3 = map(rxData.elevator, -127, 127, 45, 135);
      outS4 = map(rxData.aileron, -127, 127, 45, 135);
    } else {
      
      // 3. MIXER AERODINAMIKA (Kalkulasi Otak Pesawat Berdasarkan Mode)
      switch (rxData.auxFlag) {
        case 1: // NORMAL (Pesawat Konvensional: Baling depan, sayap belakang)
          outM1 = currentThrottle;
          outM2 = currentThrottle;
          // map(): Mengubah data remote (-127 s.d 127) menjadi derajat putar servo fisik (45 s.d 135 derajat)
          outS3 = map(rxData.elevator, -127, 127, 45, 135);
          outS4 = map(rxData.aileron, -127, 127, 45, 135);
          break;

        case 2: // DIFERENSIAL (Glider Twin Motor Busa Tanpa Servo)
          // Berbelok Kiri/Kanan dengan cara membedakan kecepatan putar motor 1 dan 2
          outM1 = currentThrottle + rxData.aileron; 
          outM2 = currentThrottle - rxData.aileron; 
          outS3 = 90; // Servo dikunci netral
          outS4 = 90; // Servo dikunci netral
          break;

        case 3: // V-TAIL / ELEVON (Pesawat Sayap Delta/JetTempur)
          outM1 = currentThrottle;
          outM2 = currentThrottle;
          // Pergerakan elevator (Naik/Turun) dan Aileron (Guling) dicampur secara matematis 
          // lalu dikirim membagi ke 2 Servo Kiri dan Kanan sayap secara bersamaan.
          outS3 = map(rxData.elevator + rxData.aileron, -254, 254, 45, 135); 
          outS4 = map(rxData.elevator - rxData.aileron, -254, 254, 45, 135); 
          break;
          
        case 4: // DARAT / AIR (Mobil RC atau Kapal)
          outM1 = currentThrottle; // Motor Penggerak Utama
          outS3 = map(rxData.rudder, -127, 127, 45, 135); // Setir Kemudi
          break;
      }
    }

    // constrain(): Hukum wajib pembatasan nilai. 
    // Jika efek Mixer di atas menghasilkan nilai melebihi batas (misal gas 200 + aileron 100 = 300),
    // constrain akan memangkas/mencekiknya kembali mentok di 255 agar perintah PWM/ESC tidak error (Overflow).
    outM1 = constrain(outM1, 0, 255);
    outM2 = constrain(outM2, 0, 255);

    // 4. SISTEM DISTRIBUSI OUTPUT (PINTU TOL HYBRID: ESC vs MOSFET)
    if (rxData.motorType == 0) {
      
      // --- JALUR A: OUTPUT ESC BRUSHLESS ---
      if (!escsAttached) { 
        // Rebut hak kontrol Pin 5 & 6 dari Arduino, serahkan ke library Servo.h
        esc1.attach(PIN_MOTOR_1); esc2.attach(PIN_MOTOR_2); 
        escsAttached = true; // Tandai bahwa pin sedang dikuasai library
      }
      
      if (!rxData.isArmed) {
        // Jika status disarmed, kirim pulsa 1000us mutlak (Titik 0% pada ESC)
        esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000);
      } else {
        // Konversi nilai mentah (0 - 255) menjadi lebar pulsa standar ESC pesawat (1000us - 2000us)
        esc1.writeMicroseconds(map(outM1, 0, 255, 1000, 2000));
        esc2.writeMicroseconds(map(outM2, 0, 255, 1000, 2000));
      }
      
    } else {
      
      // --- JALUR B: OUTPUT MOSFET BRUSHED DC (N30) ---
      if (escsAttached) { 
        // Wajib mengusir library Servo.h dari Pin 5 & 6! 
        // Jika tidak di-detach, fungsi analogWrite (PWM) di bawahnya tidak akan mau bekerja.
        esc1.detach(); esc2.detach(); 
        pinMode(PIN_MOTOR_1, OUTPUT); pinMode(PIN_MOTOR_2, OUTPUT);
        escsAttached = false; 
      }
      
      // Tembakkan sinyal kedip cepat Hardware PWM (0 - 255) langsung ke kaki Gate MOSFET
      analogWrite(PIN_MOTOR_1, outM1);
      analogWrite(PIN_MOTOR_2, outM2);
    }

    // 5. UPDATE SERVO FISIK & INDIKATOR KESELAMATAN
    // Constrain kembali untuk menjaga umur dinamo servo agar tidak memaksa berputar melebihi limit gear-nya (45-135 derajat)
    servo3.write(constrain(outS3, 45, 135));
    servo4.write(constrain(outS4, 45, 135));
    
  } 
  // --- TAMBAHAN 3B: SMART LED HEARTBEAT (DI LUAR BLOK FAILSAFE) ---
  int cycle = currentMillis % 1000; // Membuat putaran waktu 1 detik (0 - 999 ms)
  
  if (failsafeActive) {
    // 1. FAILSAFE: Kedip santai (Setengah detik nyala, setengah detik mati)
    digitalWrite(PIN_LED, (cycle < 500) ? HIGH : LOW);
  } else {
    if (rxData.isArmed) {
      // 2. ARMED (Mesin Bahaya): LED Nyala Terang, tapi berkedut MATI sekejap setiap 1 detik
      digitalWrite(PIN_LED, (cycle < 50) ? LOW : HIGH); 
    } else {
      // 3. DISARMED (Mesin Aman): LED Mati Gelap, tapi berkedut NYALA sekejap setiap 1 detik
      digitalWrite(PIN_LED, (cycle < 50) ? HIGH : LOW);
    }
  }
} 