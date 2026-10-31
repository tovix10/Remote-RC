#include <Arduino.h>
#include <EEPROM.h>

void setup() {
  Serial.begin(115200);
  Serial.println(F("Memulai Hard Reset EEPROM..."));
  
  // ATmega168P memiliki 512 Byte EEPROM (0 s/d 511)
  // Untuk memastikan, kita perlebar sedikit tidak masalah
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0xFF); 
  }
  
  Serial.println(F("Hard Reset Selesai! EEPROM kembali perawan (0xFF)."));
}

void loop() {
  // Biarkan kosong
}