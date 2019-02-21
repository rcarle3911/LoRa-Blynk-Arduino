/**
 * This sketch is for the Arduino Server Box in the blynk LoRa project for status and control of on auger.
 */
#include <SPI.h>
#include <RH_RF95.h>
#define TXPOWER 5 // TX power in dbm

#define SYSTEM_ENABLE_BUTTON_PIN 5
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#include <Ethernet.h>
#include <BlynkSimpleEthernet.h>
#define W5100_CS  10
#define SDCARD_CS 4

#define SEND_STATUS_LED A0
#define UNIT_ONLINE_LED_PIN 9
#define SYSTEM_STATUS_LED_PIN 6
#define AUGER_STATUS_LED_PIN 8

#define BUTTON_PRESS_TIME 2000
#define BLINK_TIME 1000
#define ONLINE_TIMEOUT 10000

// Blynk virtual pins
#define SYSTEM_ENABLED_BUTTON_VPIN V0
#define UNIT_ONLINE_LED_VPIN V1
#define SYSTEM_STATUS_LED_VPIN V2
#define AUGER_STATUS_LED_VPIN V3

char auth[] = "YOUR_BLYNK_AUTH_KEY_HERE"; // REPLACE WITH YOUR AUTH KEY FROM BLYNK

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

WidgetLED unitOnlineWLed(UNIT_ONLINE_LED_VPIN);
WidgetLED systemEnabledWLed(SYSTEM_STATUS_LED_VPIN);
WidgetLED augerStatusWLed(AUGER_STATUS_LED_VPIN);

uint8_t mid = 0;
bool augerStatus = false;
bool sysEnableButtonState = HIGH; // Initial state is off
bool systemEnabled = false;
bool unitOnline = false;
bool blinkUnitOnlineToggle = false;

unsigned long sysEnableButtonPressStart = 0;
unsigned long lastMessage = 0;
unsigned long lastBlink = 0;

uint8_t rx_buf[3];
uint8_t rx_len = sizeof(rx_buf);

BLYNK_CONNECTED() {
  Blynk.syncVirtual(SYSTEM_ENABLED_BUTTON_VPIN);
}

void sendRadioPacket();

BLYNK_WRITE(SYSTEM_ENABLED_BUTTON_VPIN) {
  int buttonState = param.asInt();
  systemEnabled = buttonState == 1 ? true : false;
  sendRadioPacket();
}

void setup() 
{
  pinMode(SEND_STATUS_LED, OUTPUT);     
  pinMode(SYSTEM_ENABLE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(UNIT_ONLINE_LED_PIN, OUTPUT);
  pinMode(SYSTEM_STATUS_LED_PIN, OUTPUT);
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
  if (buttonPressed(SYSTEM_ENABLE_BUTTON_PIN, &sysEnableButtonState)) {
    Serial.println(F("Button pushed... starting timer"));
    sysEnableButtonPressStart = millis();
  } else if (sysEnableButtonState == LOW) {
    if (sysEnableButtonPressStart + BUTTON_PRESS_TIME < millis()) {
      sysEnableButtonPressStart = millis();
      Serial.println(F("Button timer met... sending packet"));
      systemEnabled = !systemEnabled;
      Blynk.virtualWrite(SYSTEM_ENABLED_BUTTON_VPIN, systemEnabled);
      sendRadioPacket();
    }
  }
  
  //Get status message
  if (rf95.waitAvailableTimeout(10)) {
    if (rf95.recv(rx_buf, &rx_len)) {
      processMessage((char *)rx_buf, mid, true);
    }
  }

  if (millis() - lastMessage > ONLINE_TIMEOUT) {
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
  if (millis() - lastBlink > BLINK_TIME) {
    lastBlink = millis();
    toggleWLed(&unitOnlineWLed, blinkUnitOnlineToggle);
    digitalWrite(UNIT_ONLINE_LED_PIN, blinkUnitOnlineToggle);
    blinkUnitOnlineToggle = !blinkUnitOnlineToggle;
  }
}

bool processMessage(char * data, uint8_t mid, bool alter) {
  
  bool rt = false;
  Serial.print(F("Received message: "));Serial.println(data);

  uint8_t _mid = data[0];
  uint8_t _sysStatus = data[1];
  uint8_t _augStatus = data[2];
  
  lastMessage = millis();

  if (_mid == mid) {
    rt = true;
  } else {
    Serial.println(F("Mids mismatch"));
  }

  if (alter) {
    if ( (_sysStatus == 1) != systemEnabled) {
      systemEnabled = !systemEnabled;
      toggleWLed(&systemEnabledWLed, systemEnabled);
      digitalWrite(SYSTEM_STATUS_LED_PIN, systemEnabled);
      Blynk.virtualWrite(SYSTEM_ENABLED_BUTTON_VPIN, systemEnabled);
    }

    if ( (_augStatus == 1) != augerStatus) {
      augerStatus = !augerStatus;
      toggleWLed(&augerStatusWLed, augerStatus);
      digitalWrite(AUGER_STATUS_LED_PIN, augerStatus);
    }
  }

  return rt;
}

void sendRadioPacket() {

  digitalWrite(SEND_STATUS_LED, HIGH);  
  
  bool sent = false;

  // Capture the systemEnabled variable 
  // before it is altered by the response
  bool requestedSysEn = systemEnabled;
  
  uint8_t attempts = 0;
  while (!sent) {

    rx_buf[0] = ++mid;
    rx_buf[1] = requestedSysEn;
    
    if (attempts++ > 10) {      
      break;
    }
    rf95.send((uint8_t *)rx_buf, sizeof(rx_buf));
    if (rf95.waitPacketSent(500)) {
      if (rf95.waitAvailableTimeout(500)) {
        
        if (rf95.recv(rx_buf, &rx_len)) {
          sent = processMessage((char *)rx_buf, mid, false);
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
