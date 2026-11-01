#include <Arduino.h>

#define LED_PIN 13 // Pin LED bawaan Pro Mini

void setup() {
  // 1. UJI GPIO (General Purpose Input/Output)
  pinMode(LED_PIN, OUTPUT);
  
  // 2. UJI JALUR TX (Transmit)
  // Menggunakan baudrate 115200 sesuai standar monitor PlatformIO
  Serial.begin(115200);
  
  // Memberi jeda sebentar agar serial terminal siap
  delay(500); 
  
  Serial.println("\n========================================");
  Serial.println("   DIAGNOSTIK CHIP ATMEGA328P DIMULAI   ");
  Serial.println("========================================");
  Serial.println("Status: Flash & Bootloader Normal.");
  Serial.println("Clock : 16MHz (Verifikasi via waktu ms)");
  Serial.println("Ketik sesuatu di terminal untuk uji RX...");
  Serial.println("========================================\n");
}

void loop() {
  // Menyalakan LED
  digitalWrite(LED_PIN, HIGH);
  
  // 3. UJI TIMER & OSILATOR 16MHz
  // Fungsi millis() mencetak waktu hidup chip. Jika delay terasa pas 1 detik
  // tetapi angka millis() melompat tidak wajar, berarti ada masalah di kristal/osilator.
  Serial.print("[");
  Serial.print(millis()); 
  Serial.println(" ms] LED ON  (GPIO 13 HIGH)");
  delay(1000);
  
  // Mematikan LED
  digitalWrite(LED_PIN, LOW);
  Serial.print("[");
  Serial.print(millis());
  Serial.println(" ms] LED OFF (GPIO 13 LOW)");
  delay(1000);

  // 4. UJI JALUR RX (Receive)
  // Mengetes apakah IC bisa menerima perintah dari luar (Modul FTDI)
  if (Serial.available() > 0) {
    String dataMasuk = Serial.readStringUntil('\n');
    dataMasuk.trim(); // Menghapus spasi atau enter berlebih
    
    if (dataMasuk.length() > 0) {
      Serial.println("\n>>> [JALUR RX NORMAL] Chip menerima pesan: " + dataMasuk + " <<<\n");
    }
  }
}