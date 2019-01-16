/**
 * This sketch is for the Arduino Server Box in the blynk LoRa project for status and control of on auger.
 */
#include <SPI.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#define TXPOWER 5 // TX power in dbm

#define AUGER_ENABLE_BUTTON_PIN 5 // 
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#include <Ethernet.h>
#include <BlynkSimpleEthernet.h>
#define W5100_CS  10
#define SDCARD_CS 4
#define SEND_STATUS_LED A0
#define UNIT_ONLINE_LED_PIN 9
#define SYSTEM_ENABLED_LED_PIN 6 
#define AUGER_STATUS_LED_PIN 8 

#define BUTTON_PRESS_TIME 2000
#define BLINK_TIME 1000
#define ONLINE_TIMEOUT 10000

// Blynk virtual pins
#define AUGER_ENABLE_BUTTON_VPIN V0
#define UNIT_ONLINE_LED_VPIN V1
#define SYSTEM_ENABLED_LED_VPIN V2
#define AUGER_STATUS_LED_VPIN V3

char auth[] = "YOUR_BLYNK_AUTH_KEY_HERE"; // REPLACE WITH YOUR AUTH KEY FROM BLYNK

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

WidgetLED unitOnlineWLed(UNIT_ONLINE_LED_VPIN);
WidgetLED systemEnabledWLed(SYSTEM_ENABLED_LED_VPIN);
WidgetLED augerStatusWLed(AUGER_STATUS_LED_VPIN);

unsigned long mid = 0;
bool augerEnabled = false;
bool augerButtonState = HIGH; // Initial state is off
bool systemEnabled = false;
bool unitOnline = false;
bool blinkUnitOnlineToggle = false;

unsigned long augerButtonPressStart = 0;
unsigned long lastMessage = 0;
unsigned long lastBlink = 0;

uint8_t rx_buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t rx_len = sizeof(rx_buf);
StaticJsonBuffer<RH_RF95_MAX_MESSAGE_LEN> jsonBuffer;

BLYNK_CONNECTED() {
  Blynk.syncVirtual(AUGER_ENABLE_BUTTON_VPIN);
}

void sendRadioPacket();

BLYNK_WRITE(AUGER_ENABLE_BUTTON_VPIN) {
  int buttonState = param.asInt();
  augerEnabled = buttonState == 1 ? true : false;
  sendRadioPacket();
}

void setup() 
{
  pinMode(SEND_STATUS_LED, OUTPUT);     
  pinMode(AUGER_ENABLE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(UNIT_ONLINE_LED_PIN, OUTPUT);
  pinMode(SYSTEM_ENABLED_LED_PIN, OUTPUT);
  pinMode(AUGER_STATUS_LED_PIN, OUTPUT); 
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Arduino Server Box Running!"));
  
  radio_init();
  blynk_init();
  sendRadioPacket();
}

void loop()
{
  //Check for button presses
  if (buttonPressed(AUGER_ENABLE_BUTTON_PIN, &augerButtonState)) {
    Serial.println(F("Button pushed... starting timer"));
    augerButtonPressStart = millis();
  } else if (augerButtonState == LOW) {
    if (augerButtonPressStart + BUTTON_PRESS_TIME < millis()) {
      augerButtonPressStart = millis();
      Serial.println(F("Button timer met... sending packet"));
      augerEnabled = !augerEnabled;
      Blynk.virtualWrite(AUGER_ENABLE_BUTTON_VPIN, augerEnabled);
      sendRadioPacket();
    }
  }
  
  //Get status message
  if (rf95.waitAvailableTimeout(10)) {
    if (rf95.recv(rx_buf, &rx_len)) {
      processMessage((char *)rx_buf, mid);
    }
  }

  if (lastMessage + ONLINE_TIMEOUT < millis()) {
    if (unitOnline) {
      unitOnline = false;
      unitOnlineWLed.off();
    }
  } else {    
    unitOnline = true;
    blinkUnitOnline();
  }
  
  Blynk.run();
}

void blinkUnitOnline() {
  if (lastBlink + BLINK_TIME < millis()) {    
    lastBlink = millis();
    toggleWLed(&unitOnlineWLed, blinkUnitOnlineToggle);
    digitalWrite(UNIT_ONLINE_LED_PIN, blinkUnitOnlineToggle);
    blinkUnitOnlineToggle = !blinkUnitOnlineToggle;
  }
}

bool processMessage(char * data, unsigned long mid) {
  
  bool rt = false;
  Serial.print(F("Received message: "));Serial.println(data);
  
  JsonObject& input = jsonBuffer.parseObject(data);
  JsonVariant _mid = input[F("mid")];
  JsonVariant _sysEn = input[F("sysEn")];
  JsonVariant _augEn = input[F("augEn")];

  if ( input.success() && _mid.success() && _sysEn.success() && _augEn.success() ) {

    lastMessage = millis();
  
    if (_mid.as<unsigned long>() == mid) {
      rt = true;
    } else {
      Serial.println(F("Mids mismatch"));
    }
    if (_sysEn.as<bool>() != systemEnabled) {
      systemEnabled = !systemEnabled;
      toggleWLed(&systemEnabledWLed, systemEnabled);
      digitalWrite(SYSTEM_ENABLED_LED_PIN, systemEnabled);
    }

    if (_augEn.as<bool>() != augerEnabled) {
      augerEnabled = !augerEnabled;
      Blynk.virtualWrite(AUGER_ENABLE_BUTTON_VPIN, augerEnabled);
    }

    if ((augerStatusWLed.getValue() > 0) != augerEnabled) {
      toggleWLed(&augerStatusWLed, augerEnabled);
      digitalWrite(AUGER_STATUS_LED_PIN, augerEnabled);
    }
  }
  jsonBuffer.clear();
  return rt;
}

void sendRadioPacket() {

  digitalWrite(SEND_STATUS_LED, HIGH);
  
  
  bool sent = false;
  uint8_t attempts = 0;
  while (!sent) {
    JsonObject& root = jsonBuffer.createObject();

    root[F("mid")] = ++mid;
    root[F("augEn")] = augerEnabled;
    
    root.printTo((char*)rx_buf, sizeof(rx_buf));
    
    jsonBuffer.clear();
    if (attempts++ > 10) {      
      break;
    }
    rf95.send((uint8_t *)rx_buf, sizeof(rx_buf));
    if (rf95.waitPacketSent(500)) {
      if (rf95.waitAvailableTimeout(500)) {
        
        if (rf95.recv(rx_buf, &rx_len)) {
          sent = processMessage((char *)rx_buf, mid);          
        } else { // Nothing received
          Serial.println(F("No response"));
          continue;
        }
      } else { // No message
        Serial.println(F("No message"));
        continue;
      }
    } else { // Packet failed to send
      Serial.println(F("Failed to send"));
      continue;
    }
  }

  if (!sent) {
    Serial.println(F("Failed to send radio packet"));
  }

  digitalWrite(SEND_STATUS_LED, LOW);
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

void blynk_init() {

  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH); // Deselect the SD card
  Blynk.begin(auth);  

  // Turn off virtual LEDs
  unitOnlineWLed.off();
  systemEnabledWLed.off();
  augerStatusWLed.off();
}

// Returns true if button was pressed. False otherwise.
bool buttonPressed (int pin, bool *state) {
  if( digitalRead(pin) == LOW ) {
    if ( *state == HIGH ) {
      *state = LOW;
      return true;
    }
  } else {
    *state = HIGH;
  }
  return false;
}

// Turns a Widget led on or off.
void toggleWLed(WidgetLED *wLed, bool state) {
  if (state) {
    wLed->on();
  } else {
    wLed->off();
  }
}
