/**
 * This sketch is for the arduino near the auger bin to control and report status
 */

#include <SPI.h>
#include <RH_RF95.h>
#define TXPOWER 5 // TX power in dbm

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define MAX_MESSAGE_LENGTH 3

uint8_t buf[MAX_MESSAGE_LENGTH];
uint8_t len = sizeof(buf);

// Button/LED pins
#define SYSTEM_ENABLE_BUTTON_PIN 5
#define SYSTEM_STATUS_PIN 6
#define AUGER_STATUS_PIN 7

uint8_t lastMid = 0;
bool augerStatus = false;
bool systemEnabled = false;
bool systemStatus = false;

void setup() 
{ 
  pinMode(AUGER_STATUS_PIN, INPUT_PULLUP);
  pinMode(SYSTEM_STATUS_PIN, INPUT_PULLUP);
  pinMode(SYSTEM_ENABLE_BUTTON_PIN, OUTPUT);
  digitalWrite(SYSTEM_ENABLE_BUTTON_PIN, systemEnabled);
  Serial.begin(115200);
  delay(100);

  Serial.println(F("Arduino Bin Running"));

  radio_init();
}

void loop()
{
  Serial.println(F("Waiting for message..."));
  if (rf95.waitAvailableTimeout(2000)) {
    
    if (rf95.recv(buf, &len)) {
      processMessage((char *) buf); 
    } else {
      Serial.println(F("Receive failed"));
    }
    
  } else {
    Serial.println(F("Nothing to process"));
  }
  
  systemStatus = digitalRead(SYSTEM_STATUS_PIN);
  augerStatus = digitalRead(AUGER_STATUS_PIN);

  sendRadioPacket();
}

void processMessage(char * data) {

  Serial.println(F("Received message with"));
  Serial.print(F("MID: "));Serial.println(data[0], DEC);
  Serial.print(F("SysEn: "));Serial.println(data[1], DEC);

  lastMid = data[0];
  if (!augerStatus) { // Only allow change if auger is OFF
    systemEnabled = data[1];
    digitalWrite(SYSTEM_ENABLE_BUTTON_PIN, systemEnabled);
  } else {
    Serial.println(F("Auger ON. Not accepting input"));
  }
}

void sendRadioPacket() {
  
  buf[0] = lastMid;
  buf[1] = systemStatus;
  buf[2] = augerStatus;
 
  rf95.send((uint8_t *)buf, sizeof(buf));
  rf95.waitPacketSent(500);
  
}

void radio_init() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1) {
      delay(100);
    }
  }
  Serial.println(F("LoRa radio init OK!"));

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1) {
      delay(100);
    }
  }
  Serial.print(F("Set Freq to: ")); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(TXPOWER, false);
}
