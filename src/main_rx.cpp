/*
 * ============================================================================
 * FIRMWARE RECEIVER (RX) - RC UNIVERSAL HYBRID (ESC & MOSFET)
 * ============================================================================
 * Fitur: Auto-Binding, No-Pulses Failsafe (Motor Beep), Slew-Rate Limiter, 
 *        Throttle Deadband Lock, 4 Mode Terbang, dan Penggerak Motor Hybrid.
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>

// ============================================================================
// 1. KONFIGURASI PIN HARDWARE (PRO MINI)
// ============================================================================
#define PIN_RADIO_CE   9  // Pin CE Modul NRF24L01
#define PIN_RADIO_CSN  10 // Pin CSN Modul NRF24L01
#define PIN_MOTOR_1    5  // Output Motor Kiri / Utama (Support Hardware PWM)
#define PIN_MOTOR_2    6  // Output Motor Kanan (Support Hardware PWM)
#define PIN_SERVO_3    4  // Output Servo Digital (Elevator)
#define PIN_SERVO_4    7  // Output Servo Digital (Aileron / Rudder)
#define PIN_LED        3  // Indikator Status RX
#define PIN_BUZZER     8  // Buzzer Pencari Pesawat (Lost Model Finder)
#define PIN_VBATT      A0 // Sensor Tegangan Baterai (Maks 12.6V via Resistor Pembagi)

const byte BIND_ADDR[6] = "BIND0"; // Alamat Pipa Darurat untuk proses Binding
const uint8_t BIND_CH   = 10;      // Channel Darurat untuk proses Binding

// ============================================================================
// 2. STRUKTUR DATA (WAJIB SAMA PERSIS DENGAN TX)
// ============================================================================
// Paket Data untuk Binding
struct BindPayload {
  byte pipeAddr[5];
  uint8_t opChannel;
  uint8_t checkSum;
};

// Paket Data Kendali Utama dari TX
struct ControlData {
  uint8_t throttle;  // Tuas Gas
  int8_t  rudder;    // Belok Kiri/Kanan
  int8_t  elevator;  // Naik/Turun
  int8_t  aileron;   // Putar Sayap
  uint8_t isArmed;   // Status Kunci Motor (0=Kunci, 1=Hidup)
  uint8_t auxFlag;   // Mode Terbang
  uint8_t motorType; // 0 = ESC Brushless, 1 = MOSFET DC
};
ControlData rxData;

// Paket Data Telemetri (Dikirim balik ke TX)
struct TelemetryData {
  uint16_t batteryMilliVolt; // Tegangan baterai pesawat
  uint8_t  failsafeActive;   // Melapor ke TX jika RX sedang mode darurat
};
TelemetryData txTelem;

// ============================================================================
// 3. DEKLARASI OBJEK & VARIABEL GLOBAL
// ============================================================================
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
Servo servo3; // Servo Elevator
Servo servo4; // Servo Aileron
Servo esc1;   // Objek Servo pembentuk sinyal ESC (Motor 1)
Servo esc2;   // Objek Servo pembentuk sinyal ESC (Motor 2)

byte myPipe[5];       // Alamat pipa yang didapat setelah binding
uint8_t myChannel;    // Frekuensi radio yang didapat setelah binding
bool isBound = false; // Status apakah RX sudah punya pasangan TX

// Variabel Stabilitas & Keamanan
unsigned long lastRecvTime = 0; // Catatan waktu data terakhir masuk
bool failsafeActive = true;     // Status hilang sinyal (default true saat baru nyala)
bool escsAttached = false;      // Memantau apakah pin motor sedang dikuasai library Servo
int currentThrottle = 0;        // Memori gas saat ini (untuk fitur Slew-Rate Limiter)

// ============================================================================
// 4. FUNGSI BINDING (MENERIMA IDENTITAS DARI TX)
// ============================================================================
void enterBindingMode() {
  Serial.println(F("[BIND] Masuk Mode Binding. Menunggu TX..."));
  radio.setChannel(BIND_CH);
  radio.openReadingPipe(1, BIND_ADDR);
  radio.startListening();

  while (!isBound) {
    if (radio.available()) {
      BindPayload bp;
      radio.read(&bp, sizeof(BindPayload));
      
      // Verifikasi keaslian paket (Checksum)
      uint8_t calcSum = bp.opChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4];
      
      if (bp.checkSum == calcSum) {
        myChannel = bp.opChannel;
        memcpy(myPipe, bp.pipeAddr, 5);
        
        // Simpan permanen ke EEPROM Pro Mini
        EEPROM.update(0, 0xBB); // Tanda bahwa sudah ada data binding
        EEPROM.update(1, myChannel);
        for(int i=0; i<5; i++) EEPROM.update(2+i, myPipe[i]);
        
        isBound = true;
        Serial.println(F("[BIND] Sukses! Data tersimpan di EEPROM."));
        
        // Nada sukses
        tone(PIN_BUZZER, 2000, 1000); delay(1000);
      }
    }
    // Kedipkan LED cepat saat menunggu TX
    digitalWrite(PIN_LED, millis() % 200 < 100);
  }
}

// ============================================================================
// 5. FUNGSI UTAMA: SETUP (DIJALANKAN SEKALI SAAT DINYALAKAN)
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);

  // Inisialisasi NRF24L01
  if (!radio.begin()) { 
    Serial.println(F("[FATAL] Radio NRF24 Rusak atau Kabel Lepas!")); 
    while(1); // Sistem berhenti total jika modul tidak ada
  }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.enableDynamicPayloads();
  radio.enableAckPayload(); // Aktifkan balasan telemetri

  // --- CEK MEMORI BINDING ---
  // Jika di EEPROM sudah ada tanda 0xBB, berarti pernah di-bind
  if (EEPROM.read(0) == 0xBB) {
    myChannel = EEPROM.read(1);
    for(int i=0; i<5; i++) myPipe[i] = EEPROM.read(2+i);
    isBound = true;
    Serial.println(F("[SYSTEM] Load Data Binding dari EEPROM."));
  } else {
    // Jika masih perawan/kosong, paksa masuk mode binding
    enterBindingMode();
  }

  // Aktifkan Radio sesuai data dari EEPROM
  radio.setChannel(myChannel);
  radio.openReadingPipe(1, myPipe);
  radio.startListening();
  
  // Titipkan data telemetri pertama
  radio.writeAckPayload(1, &txTelem, sizeof(TelemetryData));
  
  // Aktifkan Servo Sayap
  servo3.attach(PIN_SERVO_3);
  servo4.attach(PIN_SERVO_4);
  
  Serial.println(F("[SYSTEM] RX Siap Terbang."));
}

// ============================================================================
// 6. FUNGSI UTAMA: LOOP (DIJALANKAN BERULANG-ULANG TERUS MENERUS)
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // --- A. CEK KONEKSI & PENERIMAAN DATA DARI TX ---
  if (radio.available()) {
    radio.read(&rxData, sizeof(ControlData)); // Baca paket dari TX
    lastRecvTime = currentMillis;             // Catat waktu data diterima
    
    // PEMULIHAN DARI FAILSAFE (Sinyal kembali normal)
    if (failsafeActive) { 
      failsafeActive = false;
      rxData.isArmed = 0; // AUTO-DISARM: Paksa motor terkunci (jangan kaget nyala)
      Serial.println(F("[RADIO] Link Tersambung Kembali. Motor Terkunci Otomatis."));
    }
    
    // Update data Telemetri Baterai untuk membalas TX di loop berikutnya
    txTelem.batteryMilliVolt = map(analogRead(PIN_VBATT), 0, 1023, 0, 12600); // Asumsi maks 3S Lipo (12.6V)
    txTelem.failsafeActive = 0;
    radio.writeAckPayload(1, &txTelem, sizeof(TelemetryData));
  }

  // --- B. LOGIKA FAILSAFE (TIDAK ADA DATA MASUK > 1 DETIK) ---
  if (currentMillis - lastRecvTime > 1000) {
    if (!failsafeActive) {
      Serial.println(F("[FAILSAFE] Sinyal Hilang! Memicu No-Pulses Failsafe."));
      failsafeActive = true;
      rxData.isArmed = 0; // Kunci motor
      rxData.throttle = 0; // Matikan gas
      
      // NO-PULSES FAILSAFE: Cabut total kendali servo pada pin ESC
      // Ini membuat ESC Brushless mengenali kondisi "Signal Lost" dan membunyikan alarm Beep-Beep dari dinamo
      esc1.detach(); esc2.detach(); 
      escsAttached = false;
      digitalWrite(PIN_MOTOR_1, LOW); 
      digitalWrite(PIN_MOTOR_2, LOW);
      
      // Netralkan posisi permukaan sayap agar pesawat meluncur lurus (Gliding)
      servo3.write(90); servo4.write(90);
    }
    txTelem.failsafeActive = 1;

    // LOST MODEL FINDER: Alarm pencarian jika hilang > 15 detik
    if (currentMillis - lastRecvTime > 15000) {
      // Buzzer bunyi terputus-putus
      if ((currentMillis % 2000) < 300) tone(PIN_BUZZER, 2500);
      else noTone(PIN_BUZZER);
    } else {
      noTone(PIN_BUZZER);
    }
  }

  // --- C. EKSEKUSI PENGGERAK (JIKA SINYAL AMAN) ---
  if (!failsafeActive) {
    
    // 1. THROTTLE SLEW-RATE LIMITER (Peredam Lonjakan Arus Baterai)
    int targetThrottle = rxData.isArmed ? rxData.throttle : 0;
    
    // Jika gas naik, jangan langsung meloncat (naikkan maksimal 15 angka per siklus)
    if (targetThrottle > currentThrottle) {
       currentThrottle += 15; 
       if (currentThrottle > targetThrottle) currentThrottle = targetThrottle;
    } else {
       // Jika gas turun, langsung turunkan instan demi keamanan!
       currentThrottle = targetThrottle; 
    }

    int outM1 = 0;  // Nilai Akhir Motor 1 (Kiri)
    int outM2 = 0;  // Nilai Akhir Motor 2 (Kanan)
    int outS3 = 90; // Posisi Akhir Servo 3
    int outS4 = 90; // Posisi Akhir Servo 4

    // 2. THROTTLE DEADBAND LOCK (Mencegah Motor Terputar Akibat Mixer Aileron)
    // Jika tuas gas posisi paling bawah (< 10), paksa mutlak mematikan motor!
    if (currentThrottle < 10) {
      outM1 = 0;
      outM2 = 0;
      // Permukaan sayap servo tetap merespon untuk ngetes sayap di darat
      outS3 = map(rxData.elevator, -127, 127, 45, 135);
      outS4 = map(rxData.aileron, -127, 127, 45, 135);
    } else {
      // 3. MIXER AERODINAMIKA (Kalkulasi Berdasarkan Mode Terbang)
      switch (rxData.auxFlag) {
        case 1: // NORMAL (Pesawat Biasa: 1 Motor, Elevator, Aileron/Rudder)
          outM1 = currentThrottle;
          outM2 = currentThrottle;
          outS3 = map(rxData.elevator, -127, 127, 45, 135);
          outS4 = map(rxData.aileron, -127, 127, 45, 135);
          break;

        case 2: // DIFERENSIAL (Glider Busa Kerdus: 2 Motor, Tanpa Servo)
          // Berbelok dengan cara menaik-turunkan salah satu putaran motor (Aileron TX)
          outM1 = currentThrottle + rxData.aileron; 
          outM2 = currentThrottle - rxData.aileron; 
          outS3 = 90; // Servo tak terpakai, netralkan
          outS4 = 90; // Servo tak terpakai, netralkan
          break;

        case 3: // V-TAIL / ELEVON (Sayap Delta/Jet)
          outM1 = currentThrottle;
          outM2 = currentThrottle;
          // Pergerakan elevator (Naik) dan Aileron (Roll) dicampur pada 2 Servo Sayap Kiri-Kanan
          outS3 = map(rxData.elevator + rxData.aileron, -254, 254, 45, 135); 
          outS4 = map(rxData.elevator - rxData.aileron, -254, 254, 45, 135); 
          break;
          
        case 4: // MOBIL / KAPAL (Daratan)
          outM1 = currentThrottle; // Motor Maju
          outS3 = map(rxData.rudder, -127, 127, 45, 135); // Servo Setir Belok Kiri/Kanan
          break;
      }
    }

    // Pastikan nilai motor tidak bocor melebihi batas 0-255
    outM1 = constrain(outM1, 0, 255);
    outM2 = constrain(outM2, 0, 255);

    // 4. SISTEM DISTRIBUSI OUTPUT (HYBRID ESC vs MOSFET)
    if (rxData.motorType == 0) {
      // --- JIKA MODE: ESC BRUSHLESS ---
      if (!escsAttached) { 
        // Pasang fungsi pembangkit pulsa Servo ke Pin Motor
        esc1.attach(PIN_MOTOR_1); esc2.attach(PIN_MOTOR_2); 
        escsAttached = true; 
      }
      
      if (!rxData.isArmed) {
        // Disarmed OPSI A (Standby Tenang) -> Kirim 1000us agar ESC Diam
        esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000);
      } else {
        // Armed -> Konversi nilai 0-255 menjadi lebar pulsa 1000us - 2000us
        esc1.writeMicroseconds(map(outM1, 0, 255, 1000, 2000));
        esc2.writeMicroseconds(map(outM2, 0, 255, 1000, 2000));
      }
    } else {
      // --- JIKA MODE: MOSFET BRUSHED DC ---
      if (escsAttached) { 
        // Lepas fungsi Servo dari Pin Motor (Wajib! Karena tabrakan dengan analogWrite)
        esc1.detach(); esc2.detach(); 
        pinMode(PIN_MOTOR_1, OUTPUT); pinMode(PIN_MOTOR_2, OUTPUT);
        escsAttached = false; 
      }
      
      // Kirim sinyal Hardware PWM konvensional (0 - 255)
      analogWrite(PIN_MOTOR_1, outM1);
      analogWrite(PIN_MOTOR_2, outM2);
    }

    // 5. UPDATE SERVO & LED INDIKATOR
    servo3.write(constrain(outS3, 45, 135));
    servo4.write(constrain(outS4, 45, 135));
    
    // LED RX Nyala = ARMED, LED RX Mati = DISARMED
    digitalWrite(PIN_LED, rxData.isArmed ? HIGH : LOW);
  }
}
// =================================== AKHIR KODE ===================================