#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#include <avr/sleep.h>
#include <avr/power.h>
// CHANGE THIS AES KEY!!!
#define AESKEY "1234567812345678"
#define CLIENT_ADDRESS 0x02
#define SERVER_ADDRESS 2
#define PIN_PULSE 3
#define PIN_BAT A0
#define PIN_INT_DIO0 2

#define SERDEBUG 1

RH_RF69 rf69(SS, PIN_INT_DIO0);
RHReliableDatagram manager(rf69, SERVER_ADDRESS);

void pin2Interrupt(void) {
  /* This will bring us back from sleep. */
  
  /* We detach the interrupt to stop it from 
   * continuously firing while the interrupt pin
   * is low.
   */
  detachInterrupt(0);
}
void enterSleep(void) {
  digitalWrite(13, LOW);
  detachInterrupt(0);
  attachInterrupt(0, pin2Interrupt, RISING);
  Serial.println("nini 😴, i goto sleep, wake me up when there is a packet waiting at the door");
  Serial.flush();
  delay(1);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  sleep_enable();
  
  sleep_mode();
  
  /* The program will continue from here. */
 
  /* First thing to do is disable sleep. */
  sleep_disable();
  digitalWrite(13, HIGH);
  attachInterrupt(0, rf69.isr0, RISING);
  rf69.handleInterrupt();
}

uint16_t batteryVoltage = 0xAABB; //analogRead(PIN_BAT);

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000); // making it easier to upload new code
  digitalWrite(13, LOW);
  Serial.begin(9600);
  Serial.println("0x00");
  pinMode(PIN_BAT, INPUT);
  pinMode(PIN_INT_DIO0, INPUT);
  pinMode(PIN_PULSE, OUTPUT);
  digitalWrite(PIN_PULSE, LOW);
  
  if (!manager.init()) Serial.println("Init of Modem failed!");
  if (!rf69.setFrequency(434.0)) Serial.println("setFrequency failed");
  rf69.setTxPower(13, true);
  rf69.setModemConfig(RH_RF69::FSK_Rb2Fd5);
  //Serial.println();
  rf69.setEncryptionKey((uint8_t*)AESKEY);
  uint8_t syncwords[] = { 0xDE, 0xAD };
  rf69.setSyncWords(syncwords, sizeof(syncwords));
  
  // Read Battery
  batteryVoltage = 0xAABB; //analogRead(PIN_BAT);
  Serial.println("sending initial packet ✉️");
  uint8_t ackData[] = { 0x00, 0x00, 0x00, 0x00 };
  ackData[0] = 0x88;
  ackData[1] = (batteryVoltage >> 8) & 0xff;
  ackData[2] = batteryVoltage & 0xff;
  ackData[3] = rf69.temperatureRead();
  rf69.send(ackData, sizeof(ackData));
  rf69.waitPacketSent();
}

long lastMessageID = 0x00000000;
void loop() {
  rf69.spiWrite(RH_RF69_REG_25_DIOMAPPING1, RH_RF69_DIOMAPPING1_DIO0MAPPING_01); // Set interrupt line 0 PayloadReady
  rf69.setOpMode(RH_RF69_OPMODE_MODE_RX); // Clears FIFO
  rf69.setModeRx();
  delay(20);
  enterSleep();
  #ifdef SERDEBUG
  Serial.println("I woke up ☕️");
  #endif
  // Should be a message for us now   
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf69.recv(buf, &len)) {
    #ifdef SERDEBUG
    Serial.print(F("headerFrom: 0x")); Serial.println(rf69.headerFrom(), HEX);
    Serial.print(F("headerTo: 0x")); Serial.println(rf69.headerTo(), HEX);
    Serial.print(F("headerId: 0x")); Serial.println(rf69.headerId(), HEX);
    Serial.print(F("headerFlags: 0x")); Serial.println(rf69.headerFlags(), HEX);
    RH_RF69::printBuffer("packet : ", buf, len);
    #endif
    if (rf69. headerTo() != CLIENT_ADDRESS) {
      #ifdef SERDEBUG
      Serial.println("Ignoring Packet, because its not for me!");
      #endif
      return;
    }
    #ifdef SERDEBUG
    Serial.print("RSSI: "); Serial.println(rf69.lastRssi(), DEC);
    Serial.print("len: "); Serial.println(len, DEC);
    #endif

    // send ACK
    digitalWrite(13, HIGH);
    bool packetSecure = true;
    if(len > 0) {
      uint8_t payloadLength = len;
      uint8_t byteCursor = 0;
      uint8_t packetType = buf[byteCursor++];
      payloadLength--;
      union {
          byte asBytes[4];
          long asLong;
      } CmessageID;
      long messageID = 0x00000000;
      for (byteCursor=1;byteCursor<5;byteCursor++)
        CmessageID.asBytes[byteCursor] = buf[byteCursor];
      messageID = CmessageID.asLong;
      payloadLength -= 4;
      #ifdef SERDEBUG
      Serial.print(F("messageID: x")); Serial.println(messageID);
      Serial.print(F("lastMessageID: x")); Serial.println(lastMessageID);
      #endif
      if (messageID <= lastMessageID) packetSecure = false; else lastMessageID = messageID;
      #ifdef SERDEBUG
      Serial.print(F("packetSecure: ")); Serial.println(packetSecure ? F("true") : F("false"));
      #endif

      // Packet Handling
      switch (packetType) {
        case 0x32: { // Battery Check CMD
            uint8_t ackData[] = { 0x00, 0x00, 0x00 };
            ackData[0] = packetSecure ? 0xFF : 0x00;
            ackData[1] = (batteryVoltage >> 8) & 0xff;
            ackData[2] = batteryVoltage & 0xff;
            delay(500);
            rf69.send(ackData, sizeof(ackData));
            rf69.waitPacketSent();
            // Payload Handling
            #ifdef SERDEBUG
            Serial.println("ACK+BAT Sent");
            Serial.println();
            #endif
        }; break;
        case 0x44: { // Shock CMD
          if (payloadLength == 3) {
            uint8_t pulseLengthFactor = buf[byteCursor++];
            uint8_t pulsePauseFactor = buf[byteCursor++];
            uint8_t pulseRepeat = buf[byteCursor++];
            // Acknowledge
            rf69.setHeaderTo(rf69.headerFrom());
            rf69.setHeaderFrom(CLIENT_ADDRESS);
            rf69.setHeaderFlags(0x80);
            uint8_t ackData[] = { 0x00, 0x00, 0x00 };
            ackData[0] = packetSecure ? 0xFF : 0x00;
            ackData[1] = (batteryVoltage >> 8) & 0xff;
            ackData[2] = batteryVoltage & 0xff;
            delay(500);
            rf69.send(ackData, sizeof(ackData));
            rf69.waitPacketSent();
            // Payload Handling
            #ifdef SERDEBUG
            Serial.println("ACK Sent");
            Serial.println();
            Serial.print(F("pulseLengthFactor: x")); Serial.println(pulseLengthFactor);
            Serial.print(F("pulsePauseFactor: x")); Serial.println(pulsePauseFactor);
            Serial.print(F("pulseRepeat: x")); Serial.println(pulseRepeat);
            #endif
            if (packetSecure == true) {
              #ifdef SERDEBUG
              Serial.println("SHOCK WILL BE APPLIED!!!11elf");
              #endif
              for (int i=0;i < pulseRepeat; i++) {
                digitalWrite(PIN_PULSE, HIGH);
                delayMicroseconds(10 * pulseLengthFactor);
                digitalWrite(PIN_PULSE, LOW);
                delayMicroseconds(10 * pulsePauseFactor);
              }
            }
          }
        }; break;
      }
    }
    digitalWrite(13, LOW);
  }
}
