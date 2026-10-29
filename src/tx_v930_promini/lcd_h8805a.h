/*
 * ============================================================================
 * LIBRARY KHUSUS LCD H-8805A - UNTUK TX V930
 * ============================================================================
 */

#ifndef LCD_H8805A_H
#define LCD_H8805A_H

#include <Arduino.h>

// ============================================================================
// PIN LCD 
// ============================================================================
#define LCD_CS_PIN   6
#define LCD_WR_PIN   7
#define LCD_DATA_PIN A5 // Pin analog A5 bisa berfungsi sebagai digital

uint8_t lcd_ram[32]; 

// ============================================================================
// MESIN STATUS ANIMASI (NON-BLOCKING)
// ============================================================================
enum AnimState { ANIM_NORMAL, ANIM_MOTOR, ANIM_SLOT };
AnimState uiState = ANIM_NORMAL;
unsigned long animStartTime = 0;
int animTargetMotor = 0;
int animTargetSlot = 1;

// ============================================================================
// DATA MAPPING ARRAY
// ============================================================================
const int ARR_TR_RUD_L[4] = {48, 49, 50, 51};
const int ARR_TR_RUD_R[4] = {52, 53, 54, 55};
const int ARR_TR_AIL_L[4] = {84, 85, 86, 87};
const int ARR_TR_AIL_R[4] = {88, 89, 90, 91};
const int ARR_TR_ELE_D[4] = {104, 105, 106, 107}; 
const int ARR_TR_ELE_U[4] = {100, 101, 102, 103}; 
const int ARR_TR_THR_D[4] = {116, 117, 118, 119}; 
const int ARR_TR_THR_U[4] = {120, 121, 122, 123}; 

const int ARR_ST_AIL_L[4] = {64, 65, 66, 67}; 
const int ARR_ST_AIL_R[4] = {56, 57, 58, 59};
const int ARR_ST_ELE_D[4] = {60, 61, 62, 63}; 
const int ARR_ST_ELE_U[4] = {68, 69, 70, 71}; 
const int ARR_ST_RUD_L[4] = {76, 77, 78, 79}; 
const int ARR_ST_RUD_R[4] = {80, 81, 82, 83}; 

// ============================================================================
// FUNGSI KOMUNIKASI HT1621 TINGKAT RENDAH (DIPERCEPAT 10x LIPAT)
// ============================================================================
void ht1621_send_bits(uint16_t data, uint8_t bits) {
  for (int i = bits - 1; i >= 0; i--) {
    digitalWrite(LCD_WR_PIN, LOW); delayMicroseconds(5); // Ubah dari 50 ke 5
    digitalWrite(LCD_DATA_PIN, (data & (1 << i)) ? HIGH : LOW);
    delayMicroseconds(5); digitalWrite(LCD_WR_PIN, HIGH); delayMicroseconds(5); // Ubah dari 50 ke 5
  }
}

void ht1621_command(uint8_t cmd) {
  digitalWrite(LCD_CS_PIN, LOW); delayMicroseconds(5); // Ubah dari 50 ke 5
  ht1621_send_bits(0x04, 3); ht1621_send_bits(cmd, 8); ht1621_send_bits(0, 1);
  digitalWrite(LCD_CS_PIN, HIGH); delay(1);
}

void ht1621_write_ram() {
  digitalWrite(LCD_CS_PIN, LOW); delayMicroseconds(5); // Ubah dari 50 ke 5
  ht1621_send_bits(0x05, 3); ht1621_send_bits(0, 6);    
  for (int i = 0; i < 32; i++) {
    for (int b = 0; b < 4; b++) {
      digitalWrite(LCD_WR_PIN, LOW); delayMicroseconds(5); // Ubah dari 50 ke 5
      digitalWrite(LCD_DATA_PIN, (lcd_ram[i] & (1 << b)) ? HIGH : LOW);
      delayMicroseconds(5); digitalWrite(LCD_WR_PIN, HIGH); delayMicroseconds(5); // Ubah dari 50 ke 5
    }
  }
  digitalWrite(LCD_CS_PIN, HIGH);
}

void setIkon(int id, bool nyala) {
  if(id < 0 || id > 127) return;
  uint8_t addr = id / 4; uint8_t bitPos = id % 4;
  if (nyala) lcd_ram[addr] |= (1 << bitPos); else lcd_ram[addr] &= ~(1 << bitPos);       
}

void setAngkaMode(int angka) {
  setIkon(126, angka == 1); 
  setIkon(125, angka == 2); 
  setIkon(97,  angka == 3); 
  setIkon(98,  angka == 4); 
}

void renderBar(int val, const int arrNeg[4], const int arrPos[4]) {
  for(int i=0; i<4; i++) { setIkon(arrNeg[i], 0); setIkon(arrPos[i], 0); }
  if (val < 0) {
    int steps = min(-val, 4); for(int i=0; i<steps; i++) setIkon(arrNeg[i], 1);
  } else if (val > 0) {
    int steps = min(val, 4); for(int i=0; i<steps; i++) setIkon(arrPos[i], 1);
  }
}

// ============================================================================
// FUNGSI INISIALISASI (Panggil 1x di setup)
// ============================================================================
void initLCD() {
  pinMode(LCD_CS_PIN, OUTPUT); pinMode(LCD_WR_PIN, OUTPUT); pinMode(LCD_DATA_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH); digitalWrite(LCD_WR_PIN, HIGH); digitalWrite(LCD_DATA_PIN, HIGH);
  delay(200);
  ht1621_command(0x01); ht1621_command(0x05); 
  ht1621_command(0x03); ht1621_command(0x29); 
  memset(lcd_ram, 0, sizeof(lcd_ram)); 
  ht1621_write_ram(); 
}

// ============================================================================
// FUNGSI PEMICU ANIMASI (Panggil saat tombol kombinasi L + Trim ditekan)
// ============================================================================
void triggerAnimMotor(int motorType) {
  uiState = ANIM_MOTOR;
  animTargetMotor = motorType;
  animStartTime = millis();
}

void triggerAnimSlot(int slot) {
  uiState = ANIM_SLOT;
  animTargetSlot = slot;
  animStartTime = millis();
}

// ============================================================================
// FUNGSI UPDATE LAYAR (Panggil terus menerus di loop utama firmware)
// ============================================================================
void updateLCD(bool isArmed, uint8_t rawThrottle, int txBatLevel, bool rxConnected, 
               int activeMode, int activeMotorType, 
               int thrTrim, int rudTrim, int eleTrim, int ailTrim, 
               int stRud, int stEle, int stAil, bool isCharging) {
               
  unsigned long currentMillis = millis();

  // 1. Konversi Nilai TX (Skala Besar) ke Skala LCD (Skala Kecil)
  int lcd_thr = map(rawThrottle, 0, 255, 0, 100); 
  
  // Trim TX (-127 s/d 127) -> LCD (-4 s/d 4)
  int tr_thr_lcd = map(thrTrim, -127, 127, -4, 4);
  int tr_rud_lcd = map(rudTrim, -127, 127, -4, 4);
  int tr_ele_lcd = map(eleTrim, -127, 127, -4, 4);
  int tr_ail_lcd = map(ailTrim, -127, 127, -4, 4);

  // Stik TX (-127 s/d 127) -> LCD (-4 s/d 4)
  int s_rud_lcd = map(stRud, -127, 127, -4, 4);
  int s_ele_lcd = map(stEle, -127, 127, -4, 4);
  int s_ail_lcd = map(stAil, -127, 127, -4, 4);

  

  // ==========================================================================
  // 2. RENDER GRAFIS KE RAM
  // ==========================================================================
  setIkon(74, 1); setIkon(75, 1); 
  setIkon(124, rxConnected);

  // BLOK RENDER BATERAI ---
  bool blinkCharge = (currentMillis / 500) % 2; // Kedip tiap 500ms

  if (isCharging) {
    // Sedang Dicas: Bar target berkedip, bar di bawahnya nyala statis
    setIkon(95, txBatLevel >= 1 ? (txBatLevel == 1 ? blinkCharge : 1) : 0);
    setIkon(94, txBatLevel >= 2 ? (txBatLevel == 2 ? blinkCharge : 1) : 0);
    setIkon(93, txBatLevel >= 3 ? (txBatLevel == 3 ? blinkCharge : 1) : 0);
    setIkon(92, txBatLevel >= 4 ? (txBatLevel == 4 ? blinkCharge : 1) : 0);
  } else {
    // Normal (Baterai Dipakai Terbang / Cas Penuh & Dicabut)
    setIkon(95, txBatLevel >= 1); setIkon(94, txBatLevel >= 2);
    setIkon(93, txBatLevel >= 3); setIkon(92, txBatLevel >= 4);
  }

  // Logika Alarm Kritis (Hanya berkedip jika baterai habis DAN sedang TIDAK dicas)
  if (txBatLevel == 0 && !isCharging) {
    setIkon(96, (currentMillis / 400) % 2); 
  } else {
    setIkon(96, 1); // Nyala statis
  }
  // --------------------------------------------------------

  if (isArmed) setIkon(72, 1);
  else setIkon(72, (currentMillis / 500) % 2); 

  // Logika State Machine UI
  setIkon(127, 0); setIkon(99, 0); setAngkaMode(0); 

  if (uiState == ANIM_NORMAL) {
    setIkon(127, 1); 
    setAngkaMode(activeMode);
  } 
  else if (uiState == ANIM_MOTOR) {
    setIkon(127, 1); 
    unsigned long elapsed = currentMillis - animStartTime;
    if (elapsed < 1200) {
      bool isBlinkOn = ((elapsed / 200) % 2) == 0;
      // Koreksi 1: Kedip angka 1 (ESC) atau angka 2 (Brushed)
      if (isBlinkOn) setAngkaMode(animTargetMotor == 0 ? 1 : 2);
    } else {
      uiState = ANIM_NORMAL; 
    }
  }
  else if (uiState == ANIM_SLOT) {
    setIkon(99, 1); 
    unsigned long elapsed = currentMillis - animStartTime;
    int sisaSlot = animTargetSlot;
    int digitTampil = 0;
    int tahapSekarang = elapsed / 600; 
    bool isNyala = (elapsed % 600) < 400; 
    
    for (int i = 0; i <= tahapSekarang; i++) {
      if (sisaSlot >= 4) { digitTampil = 4; sisaSlot -= 4; }
      else { digitTampil = sisaSlot; sisaSlot = 0; }
    }

    if (digitTampil > 0 && isNyala) {
      setAngkaMode(digitTampil);
    } else if (digitTampil == 0 && sisaSlot == 0) {
      if ((elapsed % 600) > 400) uiState = ANIM_NORMAL;
    }
  }

  // Bar Throttle Halus (Koreksi 2: 5 Tahapan)
  setIkon(115, lcd_thr > 0);  setIkon(114, lcd_thr >= 40);
  setIkon(113, lcd_thr >= 60); setIkon(112, lcd_thr >= 80); 
  setIkon(73,  lcd_thr >= 100);

  // Render Animasi Baling-baling
  if (lcd_thr == 0) {
    setIkon(108, 1); setIkon(110, 1); setIkon(109, 0); setIkon(111, 0);
  } else {
    bool propBlink = (currentMillis / 100) % 2;
    setIkon(108, propBlink);  setIkon(110, propBlink);
    setIkon(109, !propBlink); setIkon(111, !propBlink);
  }

  // Render Trim & Stik
  renderBar(tr_thr_lcd, ARR_TR_THR_D, ARR_TR_THR_U);
  renderBar(tr_rud_lcd, ARR_TR_RUD_L, ARR_TR_RUD_R);
  renderBar(tr_ele_lcd, ARR_TR_ELE_D, ARR_TR_ELE_U);
  renderBar(tr_ail_lcd, ARR_TR_AIL_L, ARR_TR_AIL_R);
  
  renderBar(s_rud_lcd, ARR_ST_RUD_L, ARR_ST_RUD_R);
  renderBar(s_ele_lcd, ARR_ST_ELE_D, ARR_ST_ELE_U);
  renderBar(s_ail_lcd, ARR_ST_AIL_L, ARR_ST_AIL_R);

  // 3. Pompa ke Layar LCD (50Hz)
  static unsigned long lastRender = 0;
  if (currentMillis - lastRender >= 50) {
    lastRender = currentMillis;
    ht1621_write_ram(); 
  }
}

#endif