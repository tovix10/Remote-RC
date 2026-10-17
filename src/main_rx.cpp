#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>

const uint8_t PIN_BIND_JUMPER = 2;
const uint8_t PIN_LED_STATUS  = 3; 
const uint8_t PIN_OUT_1       = 4; // Tergantung Mode
const uint8_t PIN_OUT_2       = 5; // Tergantung Mode
const uint8_t PIN_OUT_3       = 6; // Tergantung Mode
const uint8_t PIN_OUT_4       = 7; // Tergantung Mode
const uint8_t PIN_BUZZER_RX   = 8; 
const uint8_t PIN_BATTERY     = A0;

const byte BIND_ADDR[6] = "BIND0";
const uint8_t BIND_CH   = 10;

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
  uint8_t auxFlag; // Mode: 1=Normal, 2=Diff, 3=Delta, 4=Car
};
ControlData rxData;

struct TelemetryData {
  uint16_t batteryMilliVolt;
  uint8_t  failsafeActive;
};
TelemetryData telemData;

Servo out1, out2, out3, out4;
RF24 radio(9, 10);

byte opPipe[5] = "DEF01";
uint8_t opChannel = 76;
unsigned long lastRecvTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 500; 
uint8_t lastArmedState = 0;
unsigned long lastLedBlink = 0;
bool ledState = false;

uint16_t readBatteryVoltage() {
  return (uint16_t)(((uint32_t)analogRead(PIN_BATTERY) * 5000UL) / 1023UL);
}

void applyFailsafe() {
  out1.writeMicroseconds(1000); out2.writeMicroseconds(1000); // Motor Mati (Bawah)
  out3.writeMicroseconds(1500); out4.writeMicroseconds(1500); // Servo Tengah
  telemData.failsafeActive = 1;
}

void processControlMixing() {
  if (rxData.isArmed == 0) { applyFailsafe(); return; }

  // ==== MODE 1: PESAWAT NORMAL ====
  if (rxData.auxFlag == 1) {
    out1.writeMicroseconds(map(rxData.throttle, 0, 255, 1000, 2000)); // Gas Kiri-Y
    out2.writeMicroseconds(map(rxData.rudder, -127, 127, 1000, 2000));  // Servo Rud Kiri-X
    out3.writeMicroseconds(map(rxData.elevator, -127, 127, 1000, 2000)); // Servo Ele Kanan-Y
    out4.writeMicroseconds(map(rxData.aileron, -127, 127, 1000, 2000));  // Servo Ail Kanan-X
  }
  
  // ==== MODE 2: DIFERENSIAL 2 MOTOR ====
  // Menggunakan Analog Kanan Kiri/Kanan (aileron) untuk belok!
  else if (rxData.auxFlag == 2) {
    int16_t mLeft  = constrain(rxData.throttle + rxData.aileron, 0, 255);
    int16_t mRight = constrain(rxData.throttle - rxData.aileron, 0, 255);
    out1.writeMicroseconds(map(mLeft, 0, 255, 1000, 2000));
    out2.writeMicroseconds(map(mRight, 0, 255, 1000, 2000));
    out3.writeMicroseconds(1500); out4.writeMicroseconds(1500); // Nonaktif
  }

  // ==== MODE 3: DELTA WING / JET ====
  // Motor gas kiri, Aileron & Elevator mix di Kanan
  else if (rxData.auxFlag == 3) {
    out1.writeMicroseconds(map(rxData.throttle, 0, 255, 1000, 2000));
    int16_t mixL = rxData.elevator + rxData.aileron;
    int16_t mixR = rxData.elevator - rxData.aileron;
    out3.writeMicroseconds(map(constrain(mixL, -127, 127), -127, 127, 1000, 2000));
    out4.writeMicroseconds(map(constrain(mixR, -127, 127), -127, 127, 1000, 2000));
    out2.writeMicroseconds(1500);
  }

  // ==== MODE 4: RC MOBIL / BOAT ====
  // FULL KONTROL DI ANALOG KANAN (Aileron & Elevator)
  else if (rxData.auxFlag == 4) {
    // Elevator untuk maju mundur ESC Mobil (Tengah = 1500)
    out1.writeMicroseconds(map(rxData.elevator, -127, 127, 1000, 2000));
    // Aileron untuk servo belok roda depan
    out2.writeMicroseconds(map(rxData.aileron, -127, 127, 1000, 2000));
    out3.writeMicroseconds(1500); out4.writeMicroseconds(1500);
  }
}

void executeRxBinding() {
  radio.setChannel(BIND_CH); radio.openReadingPipe(1, BIND_ADDR); radio.startListening();
  bool bound = false; unsigned long startBind = millis();
  while (!bound && (millis() - startBind < 30000)) {
    digitalWrite(PIN_LED_STATUS, HIGH); digitalWrite(PIN_BUZZER_RX, HIGH); delay(30); 
    digitalWrite(PIN_LED_STATUS, LOW); digitalWrite(PIN_BUZZER_RX, LOW); delay(100);
    if (radio.available()) {
      BindPayload bp; radio.read(&bp, sizeof(BindPayload));
      if (bp.checkSum == (bp.opChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4])) {
        EEPROM.update(10, 0x55); EEPROM.update(11, bp.opChannel);
        for (int i = 0; i < 5; i++) EEPROM.update(12 + i, bp.pipeAddr[i]);
        bound = true;
      }
    }
  }
  if (bound) {
    digitalWrite(PIN_LED_STATUS, HIGH); digitalWrite(PIN_BUZZER_RX, HIGH); delay(1000); 
    digitalWrite(PIN_LED_STATUS, LOW); digitalWrite(PIN_BUZZER_RX, LOW);
    while (1) { delay(1000); }
  }
}

void setup() {
  pinMode(PIN_BIND_JUMPER, INPUT_PULLUP);
  pinMode(PIN_BUZZER_RX, OUTPUT); pinMode(PIN_LED_STATUS, OUTPUT);
  out1.attach(PIN_OUT_1); out2.attach(PIN_OUT_2); out3.attach(PIN_OUT_3); out4.attach(PIN_OUT_4);
  applyFailsafe();

  if (!radio.begin()) while(1);
  radio.setPALevel(RF24_PA_HIGH); radio.setDataRate(RF24_1MBPS);
  radio.enableDynamicPayloads(); radio.enableAckPayload();

  if (digitalRead(PIN_BIND_JUMPER) == LOW) executeRxBinding();

  if (EEPROM.read(10) == 0x55) {
    opChannel = EEPROM.read(11);
    for (int i = 0; i < 5; i++) opPipe[i] = EEPROM.read(12 + i);
  }
  radio.setChannel(opChannel); radio.openReadingPipe(1, opPipe); radio.startListening();
  digitalWrite(PIN_BUZZER_RX, HIGH); delay(150); digitalWrite(PIN_BUZZER_RX, LOW);
}

void loop() {
  unsigned long now = millis();
  if (radio.available()) {
    radio.read(&rxData, sizeof(ControlData));
    lastRecvTime = now;
    digitalWrite(PIN_LED_STATUS, HIGH);
    
    if (rxData.isArmed != lastArmedState) {
      digitalWrite(PIN_BUZZER_RX, HIGH); delay(rxData.isArmed ? 450 : 90); digitalWrite(PIN_BUZZER_RX, LOW);
      if(!rxData.isArmed){ delay(90); digitalWrite(PIN_BUZZER_RX, HIGH); delay(90); digitalWrite(PIN_BUZZER_RX, LOW); }
      lastArmedState = rxData.isArmed;
    }
    processControlMixing();
    telemData.failsafeActive = 0; telemData.batteryMilliVolt = readBatteryVoltage();
    radio.writeAckPayload(1, &telemData, sizeof(TelemetryData));
  }

  if (now - lastRecvTime > FAILSAFE_TIMEOUT) {
    applyFailsafe(); lastArmedState = 0;
    if (now - lastLedBlink >= 500) { lastLedBlink = now; ledState = !ledState; digitalWrite(PIN_LED_STATUS, ledState); }
    if (now - lastRecvTime > 3000) { digitalWrite(PIN_BUZZER_RX, HIGH); delay(100); digitalWrite(PIN_BUZZER_RX, LOW); delay(900); }
  }
}