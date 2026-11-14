#include <Arduino.h>
#include <LiquidCrystal.h>

// Inisialisasi pin LCD (RS, E, D4, D5, D6, D7)
// Sesuai dengan wiring tabel di atas
LiquidCrystal lcd(12, 11, 5, 6, 7, 8);

void setup() {
  // Deklarasi ukuran LCD: 16 kolom, 2 baris
  lcd.begin(16, 2);

  // Tampilan awal saat booting
  lcd.setCursor(0, 0);
  lcd.print("Test LCD 1602A");
  lcd.setCursor(0, 1);
  lcd.print("Status: OK!");
  
  delay(2000);
  lcd.clear();
}

void loop() {
  // Menampilkan teks statis di baris pertama
  lcd.setCursor(0, 0);
  lcd.print("LCD Siap Dipakai");

  // Menampilkan waktu berjalan (non-blocking style) di baris kedua
  lcd.setCursor(0, 1);
  lcd.print("Uptime: ");
  lcd.print(millis() / 1000); 
  lcd.print(" dtk "); // Spasi ekstra di belakang untuk menghapus sisa karakter lama
  
  delay(100); 
}