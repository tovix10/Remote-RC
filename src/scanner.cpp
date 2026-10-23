/*
 * ============================================================================
 * FIRMWARE SIMULATOR LCD H-8805A - VERSI 4 (FULL UI & ANIMASI KOMBINASI)
 * ============================================================================
 */

#include <Arduino.h>

#define CS_PIN   10
#define WR_PIN   11
#define DATA_PIN 12

uint8_t lcd_ram[32]; 

// ============================================================================
// VARIABEL STATUS SISTEM UTAMA
// ============================================================================
bool sim_armed = false;
int sim_throttle = 0;   
int sim_battery = 4;    
bool sim_rx_conn = true;

int activeMode = 1;       // Mode Terbang (1 s/d 4)
int activeMotorType = 0;  // 0 = ESC, 1 = Brushed
int currentSlot = 1;      // Slot EEPROM (1 s/d 10)

// Variabel Input Stik & Trim (-4 s/d 4)
int tr_rud = 0, tr_ail = 0, tr_ele = 0, tr_thr = 0; 
int st_ail = 0, st_ele = 0, st_rud = 0; 

// ============================================================================
// MESIN STATUS ANIMASI (NON-BLOCKING)
// ============================================================================
enum AnimState { ANIM_NORMAL, ANIM_MOTOR, ANIM_SLOT };
AnimState uiState = ANIM_NORMAL;
unsigned long animStartTime = 0;

// ============================================================================
// DATA MAPPING ARRAY LCD
// ============================================================================
const int ARR_TR_RUD_L[4] = {48, 49, 50, 51};
const int ARR_TR_RUD_R[4] = {52, 53, 54, 55};
const int ARR_TR_AIL_L[4] = {84, 85, 86, 87};
const int ARR_TR_AIL_R[4] = {88, 89, 90, 91};
const int ARR_TR_ELE_D[4] = {104, 105, 106, 107}; 
const int ARR_TR_ELE_U[4] = {100, 101, 102, 103}; 
const int ARR_TR_THR_D[4] = {116, 117, 118, 119}; 
const int ARR_TR_THR_U[4] = {120, 121, 122, 123}; 

const int ARR_ST_AIL_L[4] = {56, 57, 58, 59};
const int ARR_ST_AIL_R[4] = {64, 65, 66, 67};
const int ARR_ST_ELE_D[4] = {60, 61, 62, 63}; 
const int ARR_ST_ELE_U[4] = {68, 69, 70, 71}; 

const int ARR_ST_RUD_L[4] = {80, 81, 82, 83}; 
const int ARR_ST_RUD_R[4] = {76, 77, 78, 79}; 

// ============================================================================
// FUNGSI KOMUNIKASI HT1621 & RENDER DASAR
// ============================================================================
void ht1621_send_bits(uint16_t data, uint8_t bits) {
  for (int i = bits - 1; i >= 0; i--) {
    digitalWrite(WR_PIN, LOW); delayMicroseconds(50);
    digitalWrite(DATA_PIN, (data & (1 << i)) ? HIGH : LOW);
    delayMicroseconds(50); digitalWrite(WR_PIN, HIGH); delayMicroseconds(50);
  }
}

void ht1621_command(uint8_t cmd) {
  digitalWrite(CS_PIN, LOW); delayMicroseconds(50);
  ht1621_send_bits(0x04, 3); ht1621_send_bits(cmd, 8); ht1621_send_bits(0, 1);
  digitalWrite(CS_PIN, HIGH); delay(1);
}

void ht1621_write_ram() {
  digitalWrite(CS_PIN, LOW); delayMicroseconds(50);
  ht1621_send_bits(0x05, 3); ht1621_send_bits(0, 6);    
  for (int i = 0; i < 32; i++) {
    for (int b = 0; b < 4; b++) {
      digitalWrite(WR_PIN, LOW); delayMicroseconds(50);
      digitalWrite(DATA_PIN, (lcd_ram[i] & (1 << b)) ? HIGH : LOW);
      delayMicroseconds(50); digitalWrite(WR_PIN, HIGH); delayMicroseconds(50);
    }
  }
  digitalWrite(CS_PIN, HIGH);
}

void setIkon(int id, bool nyala) {
  if(id < 0 || id > 127) return;
  uint8_t addr = id / 4; uint8_t bitPos = id % 4;
  if (nyala) lcd_ram[addr] |= (1 << bitPos); else lcd_ram[addr] &= ~(1 << bitPos);       
}

void setAngkaMode(int angka) {
  setIkon(126, angka == 1); // ID Teks 1
  setIkon(125, angka == 2); // ID Teks 2
  setIkon(97,  angka == 3); // ID Teks 3
  setIkon(98,  angka == 4); // ID Teks 4
}

// Fungsi Pembangun Garis (Bar Builder) untuk Trim & Stick
void renderBar(int val, const int arrNeg[4], const int arrPos[4]) {
  // Matikan semua 8 titik di sumbu tersebut
  for(int i=0; i<4; i++) { setIkon(arrNeg[i], 0); setIkon(arrPos[i], 0); }
  
  // Nyalakan bertahap sesuai nilai
  if (val < 0) {
    int steps = min(-val, 4);
    for(int i=0; i<steps; i++) setIkon(arrNeg[i], 1);
  } else if (val > 0) {
    int steps = min(val, 4);
    for(int i=0; i<steps; i++) setIkon(arrPos[i], 1);
  }
}

// ============================================================================
// SETUP 
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(CS_PIN, OUTPUT); pinMode(WR_PIN, OUTPUT); pinMode(DATA_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); digitalWrite(WR_PIN, HIGH); digitalWrite(DATA_PIN, HIGH);
  delay(200);

  ht1621_command(0x01); ht1621_command(0x05); 
  ht1621_command(0x03); ht1621_command(0x29); 
  memset(lcd_ram, 0, sizeof(lcd_ram)); ht1621_write_ram(); 
  
  Serial.println("\n=== SIMULATOR LCD V930 (V4 - FULL UI) ===");
  Serial.println("[A] Arm/Disarm   | [M] Ganti Mode Terbang (1-4)");
  Serial.println("[T] Ganti Motor Type (Simulasi L + Trim Ele Dwn)");
  Serial.println("[C] Ganti Slot CH (Simulasi L + Trim Ail Rgt)");
  Serial.println("---------------------------------");
  Serial.println("[U/O] Stik Rudder L/R | [J/L] Stik Aileron L/R");
  Serial.println("[I/K] Stik Ele U/D    | [W/S] Gas Naik/Turun");
  Serial.println("[1..8] Tombol Trim");
}

// ============================================================================
// LOOP UTAMA
// ============================================================================
void loop() {
  unsigned long currentMillis = millis();

  if (Serial.available() > 0) {
    char in = Serial.read();
    if (in == '\n' || in == '\r') return; 

    // Menu Dasar
    if (in == 'a' || in == 'A') sim_armed = !sim_armed;
    if (in == 'm' || in == 'M') { 
      activeMode++; if(activeMode > 4) activeMode = 1; 
      uiState = ANIM_NORMAL; // Kembali normal jika tombol dipencet
    }
    
    // PEMICU ANIMASI KOMBINASI L + TRIM
    if (in == 't' || in == 'T') {
      activeMotorType = (activeMotorType == 0) ? 1 : 0;
      uiState = ANIM_MOTOR;
      animStartTime = currentMillis;
      Serial.print("Motor Type Berubah: "); Serial.println(activeMotorType == 0 ? "ESC" : "Brushed");
    }
    if (in == 'c' || in == 'C') {
      currentSlot++; if(currentSlot > 10) currentSlot = 1;
      uiState = ANIM_SLOT;
      animStartTime = currentMillis;
      Serial.print("Slot Channel Berubah: "); Serial.println(currentSlot);
    }

    // Input Gas & Stik
    if (in == 'w') { sim_throttle += 20; if(sim_throttle > 100) sim_throttle = 100; }
    if (in == 's') { sim_throttle -= 20; if(sim_throttle < 0) sim_throttle = 0; }
    if (in == 'u') { st_rud--; if(st_rud < -4) st_rud = -4; }
    if (in == 'o') { st_rud++; if(st_rud > 4) st_rud = 4; }
    if (in == 'j' || in == 'J') { st_ail--; if(st_ail < -4) st_ail = -4; }
    if (in == 'l' || in == 'L') { st_ail++; if(st_ail > 4) st_ail = 4; }
    if (in == 'i' || in == 'I') { st_ele++; if(st_ele > 4) st_ele = 4; } 
    if (in == 'k' || in == 'K') { st_ele--; if(st_ele < -4) st_ele = -4; }
    // Tes Trim
    if (in == '1') { tr_rud--; if(tr_rud < -4) tr_rud = -4; }
    if (in == '2') { tr_rud++; if(tr_rud > 4) tr_rud = 4; }
    if (in == '3') { tr_ail--; if(tr_ail < -4) tr_ail = -4; }
    if (in == '4') { tr_ail++; if(tr_ail > 4) tr_ail = 4; }
    if (in == '5') { tr_ele--; if(tr_ele < -4) tr_ele = -4; }
    if (in == '6') { tr_ele++; if(tr_ele > 4) tr_ele = 4; }
    if (in == '7') { tr_thr--; if(tr_thr < -4) tr_thr = -4; }
    if (in == '8') { tr_thr++; if(tr_thr > 4) tr_thr = 4; } 
  }
  // ==========================================================================
  // RENDER GRAFIS (NON-BLOCKING)
  // ==========================================================================
  
  // 1. Elemen Statis, Sinyal & Baterai
  setIkon(74, 1); setIkon(75, 1); setIkon(96, 1); 
  setIkon(124, sim_rx_conn);
  setIkon(95, sim_battery >= 1); setIkon(94, sim_battery >= 2);
  setIkon(93, sim_battery >= 3); setIkon(92, sim_battery >= 4);

  // 2. Indikator ARM / DISARM (ID 72 di tengah)
  if (sim_armed) {
    setIkon(72, 1); // Nyala diam
  } else {
    setIkon(72, (currentMillis / 500) % 2); // Kedip tiap 500ms
  }

  // 3. Logika Animasi Angka & Teks (State Machine UI)
  setIkon(127, 0); // Matikan teks MODE secara default
  setIkon(99, 0);  // Matikan teks CH secara default
  setAngkaMode(0); // Matikan semua angka terlebih dahulu

  if (uiState == ANIM_NORMAL) {
    // Tampilan Standar: Tulisan MODE nyala, Angka Mode nyala statis
    setIkon(127, 1); 
    setAngkaMode(activeMode);
  } 
  else if (uiState == ANIM_MOTOR) {
    // Animasi Ganti Motor: Kedip angka mode 3x (Total durasi 1200ms)
    setIkon(127, 1); // Tulisan MODE tetap nyala
    unsigned long elapsed = currentMillis - animStartTime;
    if (elapsed < 1200) {
      // Pembagian waktu per 200ms (On-Off-On-Off-On-Off)
      bool isBlinkOn = ((elapsed / 200) % 2) == 0;
      if (isBlinkOn) setAngkaMode(activeMotorType == 0 ? 1 : 2);
    } else {
      uiState = ANIM_NORMAL; // Selesai, kembali ke normal
    }
  }
  else if (uiState == ANIM_SLOT) {
    // Animasi Ganti Slot: Tampilkan tulisan CH dan rentetan angka penjumlahan
    setIkon(99, 1); // Tulisan CH nyala
    unsigned long elapsed = currentMillis - animStartTime;
    
    // Algoritma Pemecah Angka Slot (Contoh: Slot 7 -> Tampil '4' lalu '3')
    int sisaSlot = currentSlot;
    int digitTampil = 0;
    
    // Tentukan tahap kedipan berdasarkan waktu yang sudah berlalu (Setiap tahap 600ms: 400ms On, 200ms Off)
    int tahapSekarang = elapsed / 600; 
    bool isNyala = (elapsed % 600) < 400; // Nyala 400ms pertama di tiap tahap
    
    // Hitung matematis digit apa yang harus tampil di tahap ini
    for (int i = 0; i <= tahapSekarang; i++) {
      if (sisaSlot >= 4) { digitTampil = 4; sisaSlot -= 4; }
      else { digitTampil = sisaSlot; sisaSlot = 0; }
    }

    if (digitTampil > 0 && isNyala) {
      setAngkaMode(digitTampil);
    } else if (digitTampil == 0 && sisaSlot == 0) {
      // Jika semua angka sudah selesai ditampilkan, beri jeda lalu kembali normal
      if ((elapsed % 600) > 400) uiState = ANIM_NORMAL;
    }
  }

  // 4. Bar & Stik
  setIkon(115, sim_throttle > 0); setIkon(114, sim_throttle >= 25);
  setIkon(113, sim_throttle >= 50); setIkon(112, sim_throttle >= 75); setIkon(73, sim_throttle >= 100);

  renderBar(st_rud, ARR_ST_RUD_L, ARR_ST_RUD_R);
  renderBar(st_ail, ARR_ST_AIL_L, ARR_ST_AIL_R);
  renderBar(st_ele, ARR_ST_ELE_D, ARR_ST_ELE_U);

  // 5. Update Layar (50Hz)
  static unsigned long lastRender = 0;
  if (currentMillis - lastRender >= 50) {
    lastRender = currentMillis;
    ht1621_write_ram(); 
  }
}