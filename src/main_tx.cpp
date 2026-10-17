#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <PS2X_lib.h>
#include <EEPROM.h>

#define PIN_RADIO_CE   9
#define PIN_RADIO_CSN  10
#define PS2_DAT        4
#define PS2_CMD        5
#define PS2_ATT        6
#define PS2_CLK        7
#define PIN_LED_HIJAU  2
#define PIN_LED_MERAH  3
#define PIN_BUZZER     8

const byte BIND_ADDR[6] = "BIND0";
const uint8_t BIND_CH   = 10;
const uint8_t MAX_MODELS = 10;

struct BindPayload {
  byte pipeAddr[5];
  uint8_t opChannel;
  uint8_t checkSum;
};

struct ControlData {
  uint8_t throttle;
  int8_t  rudder;
  int8_t  elevator;
  int8_t  aileron;
  uint8_t isArmed;
  uint8_t auxFlag;
};
ControlData txData;

struct TelemetryData {
  uint16_t batteryMilliVolt;
  uint8_t  failsafeActive;
};
TelemetryData rxTelem;

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
PS2X ps2x;

uint8_t currentSlot = 1; 
byte activePipe[5];
uint8_t activeChannel;
uint8_t activeMode = 1; 
int throttleTrim = 0;
int steeringTrim = 0;

unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL = 30;
bool isArmed = false;
bool ps2Initialized = false;

// --- Timer Non-Blocking untuk Buzzer ---
unsigned long buzzerEndTime = 0;
bool buzzerActive = false;

void beep(unsigned int duration) {
  digitalWrite(PIN_BUZZER, HIGH);
  buzzerActive = true;
  buzzerEndTime = millis() + duration;
}

void handleBuzzer() {
  if (buzzerActive && millis() >= buzzerEndTime) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerActive = false;
  }
}

// --- FUNGSI EEPROM ---
int getSlotOffset(uint8_t slot) {
  return 100 + ((slot - 1) * 10);
}

void saveTrimToSlot() {
  int offset = getSlotOffset(currentSlot);
  EEPROM.update(offset + 8, (uint8_t)(throttleTrim + 128)); 
  EEPROM.update(offset + 9, (uint8_t)(steeringTrim + 128));
  // Nada konfirmasi simpan: Tit-Tit-Tit
  beep(100); delay(150); beep(100); delay(150); beep(200);
}

void loadModel(uint8_t slot) {
  int offset = getSlotOffset(slot);
  
  if (EEPROM.read(offset) != 0xAA) {
    EEPROM.update(offset, 0xAA);
    EEPROM.update(offset + 1, random(20, 95));
    for (int i = 0; i < 5; i++) EEPROM.update(offset + 2 + i, 'A' + random(0, 26));
    EEPROM.update(offset + 7, 1); 
    EEPROM.update(offset + 8, 128); 
    EEPROM.update(offset + 9, 128); 
  }

  activeChannel = EEPROM.read(offset + 1);
  for (int i = 0; i < 5; i++) activePipe[i] = EEPROM.read(offset + 2 + i);
  activeMode = EEPROM.read(offset + 7);
  throttleTrim = (int)EEPROM.read(offset + 8) - 128;
  steeringTrim = (int)EEPROM.read(offset + 9) - 128;

  EEPROM.update(50, slot); 
  currentSlot = slot;

  for (int i = 0; i < slot; i++) {
    beep(100); delay(200);
  }
}

void executeBindingProcess() {
  Serial.print(F("=== BINDING MODEL SLOT: ")); Serial.print(currentSlot); Serial.println(F(" ==="));
  radio.setChannel(BIND_CH);
  radio.openWritingPipe(BIND_ADDR);
  radio.stopListening();

  BindPayload bp;
  memcpy(bp.pipeAddr, activePipe, 5);
  bp.opChannel = activeChannel;
  bp.checkSum = activeChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4];

  unsigned long bindStartTime = millis();
  while (millis() - bindStartTime < 10000) {
    radio.write(&bp, sizeof(BindPayload));
    digitalWrite(PIN_BUZZER, HIGH); delay(40); digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED_HIJAU, HIGH); delay(80); digitalWrite(PIN_LED_HIJAU, LOW);
    delay(150);
  }
  beep(500);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_HIJAU, OUTPUT);
  pinMode(PIN_LED_MERAH, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  randomSeed(analogRead(A6) + analogRead(A7));

  uint8_t lastSlot = EEPROM.read(50);
  if (lastSlot < 1 || lastSlot > MAX_MODELS) lastSlot = 1;
  loadModel(lastSlot);

  int error = -1; byte retry = 0;
  while (error != 0 && retry < 15) {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
    if (error == 0) { ps2Initialized = true; break; }
    delay(300); retry++;
  }

  if (!radio.begin()) { Serial.println(F("NRF24 Fail!")); while(1); }
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  if (ps2Initialized) {
    ps2x.read_gamepad(false, 0);
    if (ps2x.Button(PSB_L1) && ps2x.Button(PSB_R1)) {
      executeBindingProcess();
    }
  }

  radio.setChannel(activeChannel);
  radio.openWritingPipe(activePipe);
  radio.stopListening();
  Serial.println(F("TX Siap."));
}

void loop() {
  unsigned long currentMillis = millis();
  handleBuzzer(); // Urus matikan buzzer tanpa block delay

  if (currentMillis - lastTxTime >= TX_INTERVAL) {
    lastTxTime = currentMillis;

    int thrVal = 0, rudVal = 0, eleVal = 0, ailVal = 0;

    if (ps2Initialized) {
      if (ps2x.read_gamepad(false, 0)) {
        
        if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
           uint8_t next = currentSlot + 1;
           if(next > MAX_MODELS) next = 1;
           loadModel(next);
           radio.setChannel(activeChannel);
           radio.openWritingPipe(activePipe);
        }

        if (ps2x.Button(PSB_SELECT) && ps2x.ButtonPressed(PSB_TRIANGLE)) {
           activeMode++;
           if (activeMode > 4) activeMode = 1;
           EEPROM.update(getSlotOffset(currentSlot) + 7, activeMode);
           beep(400); delay(450); // Nada awal panjang
           for(int i=0; i<activeMode; i++){ beep(100); delay(200); }
        }

        int rawLY = ps2x.Analog(PSS_LY);
        int rawLX = ps2x.Analog(PSS_LX);
        int rawRY = ps2x.Analog(PSS_RY);
        int rawRX = ps2x.Analog(PSS_RX);

        if ((rawLY == 0 && rawRX == 0 && rawLX == 0 && rawRY == 0) || (rawLY == 255 && rawRX == 255 && rawLX == 255 && rawRY == 255)) {
           if (isArmed) { isArmed = false; beep(200); }
        } else {
          
          // --- KONTROL TRIM (Tahan & Jalan) ---
          // Menggunakan ps2x.Button (Bukan ButtonPressed) agar berulang saat ditahan
          // Ditambah logika delay kecil (Timer) agar tidak melesat terlalu cepat
          static unsigned long lastTrimTime = 0;
          if (currentMillis - lastTrimTime > 150) { // Kecepatan luncur trim = 150ms per step
            bool trimChanged = false;
            if (ps2x.Button(PSB_PAD_RIGHT) && !ps2x.Button(PSB_SELECT)) { steeringTrim = constrain(steeringTrim + 2, -127, 127); trimChanged = true; }
            if (ps2x.Button(PSB_PAD_LEFT))  { steeringTrim = constrain(steeringTrim - 2, -127, 127); trimChanged = true; }
            if (ps2x.Button(PSB_PAD_UP))    { throttleTrim = constrain(throttleTrim + 3, -127, 127); trimChanged = true; }
            if (ps2x.Button(PSB_PAD_DOWN))  { throttleTrim = constrain(throttleTrim - 3, -127, 127); trimChanged = true; }
            
            if (trimChanged) {
              beep(30); // Bunyi bip super pendek setiap kali nilai trim bergeser
              lastTrimTime = currentMillis;
            }
          }

          // Reset Trim ke 0 (Tombol Silang)
          if (ps2x.ButtonPressed(PSB_CROSS)) { 
            steeringTrim = 0; throttleTrim = 0; 
            beep(250); // Bunyi agak panjang menandakan netral
          }
          
          // Simpan Permanen (Tombol Bulat)
          if (ps2x.ButtonPressed(PSB_CIRCLE)) saveTrimToSlot();

          // Kalkulasi murni: Stik + Trim
          thrVal = (rawLY < 240) ? constrain(map(rawLY, 240, 0, 0, 255) + throttleTrim, 0, 255) : 0;
          rudVal = constrain(map(rawLX, 0, 255, -127, 127) + steeringTrim, -127, 127);
          ailVal = map(rawRX, 0, 255, -127, 127);
          eleVal = map(rawRY, 0, 255, 127, -127);

          if (abs(rudVal) < 10) rudVal = 0;
          if (abs(ailVal) < 10) ailVal = 0;
          if (abs(eleVal) < 10) eleVal = 0;

          if (ps2x.ButtonPressed(PSB_START)) {
            isArmed = !isArmed;
            if(isArmed) beep(600); else { beep(100); delay(150); beep(100); }
          }
        }
      }
    }

    txData.isArmed = isArmed ? 1 : 0;
    txData.auxFlag = activeMode;

    if (!isArmed) {
      txData.throttle = 0; txData.rudder = 0; txData.elevator = 0; txData.aileron = 0;
    } else {
      txData.throttle = (uint8_t)thrVal;
      txData.rudder   = (int8_t)rudVal;
      txData.elevator = (int8_t)eleVal;
      txData.aileron  = (int8_t)ailVal;
    }

    bool radioSuccess = radio.write(&txData, sizeof(ControlData));
    if (radioSuccess && radio.isAckPayloadAvailable()) radio.read(&rxTelem, sizeof(TelemetryData));

    // --- MODE DEBUG SERIAL (Pantau Trim) ---
    static unsigned long lastPrint = 0;
    if (currentMillis - lastPrint > 500) { 
       lastPrint = currentMillis;
       Serial.print(F("Slot: ")); Serial.print(currentSlot);
       Serial.print(F(" | Mode: ")); Serial.print(activeMode);
       Serial.print(F(" | TrimSteer: ")); Serial.print(steeringTrim);
       Serial.print(F(" | TrimThr: ")); Serial.println(throttleTrim);
    }
  }
}