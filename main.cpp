#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#include <avr/sleep.h>
#include <avr/power.h>

#define AESKEY "CHANGEMECHANGEME"
#define CLIENT_ADDRESS 0x01
#define SERVER_ADDRESS 0x02
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
uint16_t getBatVoltage() {
  return (analogRead(PIN_BAT) + analogRead(PIN_BAT) + analogRead(PIN_BAT) + analogRead(PIN_BAT) + analogRead(PIN_BAT) + analogRead(PIN_BAT) + analogRead(PIN_BAT) + analogRead(PIN_BAT) + analogRead(PIN_BAT) + analogRead(PIN_BAT)) / 10;
}
void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  Serial.begin(9600);
  Serial.println("0x00");
  pinMode(PIN_BAT, INPUT);
  pinMode(PIN_INT_DIO0, INPUT);
  pinMode(PIN_PULSE, OUTPUT);
  digitalWrite(PIN_PULSE, LOW);

  if (!manager.init()) Serial.println("Init of Modem failed!");
  if (!rf69.setFrequency(433.0)) Serial.println("setFrequency failed");
  rf69.setTxPower(2, true);
  //rf69.setModemConfig(RH_RF69::FSK_Rb2Fd5);
  rf69.setModemConfig(RH_RF69::GFSK_Rb2_4Fd4_8);
  //Serial.println();
  rf69.setEncryptionKey((uint8_t*)AESKEY);
  uint8_t syncwords[] = { 0xDE, 0xAD };
  rf69.setSyncWords(syncwords, sizeof(syncwords));

  // Read Battery
  batteryVoltage = getBatVoltage();
}

void array_to_string(byte array[], unsigned int len, char buffer[]) {
  for (unsigned int i = 0; i < len; i++) {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  buffer[len*2] = '\0';
}


uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];



#pragma region "0xBA General Status Request"
struct __attribute__ ((packed)) PacketGeneralStatusRequest {
  unsigned char head;
};
union PacketGeneralStatusRequestUnion {
  struct PacketGeneralStatusRequest Packet;
  uint8_t Byte[1];
};
PacketGeneralStatusRequestUnion packetGeneralStatusRequest;
#pragma endregion
#pragma region "0xBA General Status Response"
struct __attribute__ ((packed)) PacketGeneralStatusResponse {
  unsigned char check;
  unsigned short battery;
  unsigned char rfTemperature;
};
union PacketGeneralStatusResponseUnion {
  struct PacketGeneralStatusResponse Packet;
  uint8_t Byte[4];
};
PacketGeneralStatusResponseUnion packetGeneralStatusResponse;
#pragma endregion
#pragma region "0xAA Shock Command Request"
struct __attribute__ ((packed)) PacketShockCommandRequest {
  unsigned char head;
  unsigned long messageID;
  unsigned short pulseLength;
  unsigned short pauseLength;
  unsigned char repeatNum;
};
union PacketShockCommandRequestUnion {
  struct PacketShockCommandRequest Packet;
  uint8_t Byte[1+4+2+2+1];
};
PacketShockCommandRequestUnion packetShockCommandRequest;
#pragma endregion
#pragma region "0xAA Shock Command Response"
struct __attribute__ ((packed)) PacketShockCommandResponse {
  unsigned long messageID;
};
union PacketShockCommandResponseUnion {
  struct PacketShockCommandResponse Packet;
  uint8_t Byte[4];
};
PacketShockCommandResponseUnion packetShockCommandResponse;
#pragma endregion



void shockTheCat() {
  digitalWrite(13, HIGH);
  for (int i=0;i < packetShockCommandRequest.Packet.repeatNum; i++) {
    digitalWrite(PIN_PULSE, HIGH);
    delayMicroseconds(10 * packetShockCommandRequest.Packet.pulseLength);
    digitalWrite(PIN_PULSE, LOW);
    delayMicroseconds(10 * packetShockCommandRequest.Packet.pauseLength);
  }
  digitalWrite(13, LOW);
}

unsigned long lastMessageID = 0x00000000;
char debugMessage[50];
void loop() {

  packetShockCommandRequest.Packet.head = 0xAA;
  packetGeneralStatusRequest.Packet.head = 0xBA;
  
  rf69.spiWrite(RH_RF69_REG_25_DIOMAPPING1, RH_RF69_DIOMAPPING1_DIO0MAPPING_01); // Set interrupt line 0 PayloadReady
  rf69.setOpMode(RH_RF69_OPMODE_MODE_RX); // Clears FIFO
  rf69.setModeRx();
  delay(20);
  enterSleep();
  #ifdef SERDEBUG
  Serial.println("I woke up ☕️");
  #endif
  batteryVoltage = getBatVoltage();
  // Should be a message for us now
  if (manager.available()) {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from)) {
      Serial.print("got request from : 0x"); Serial.print(from, HEX); Serial.print(": "); array_to_string(buf, len, debugMessage); Serial.println(debugMessage);
      switch (buf[0]) {
        case 0xBA: { // General Status
          packetGeneralStatusResponse.Packet.check = 0x88;
          packetGeneralStatusResponse.Packet.battery = batteryVoltage;
          packetGeneralStatusResponse.Packet.rfTemperature = rf69.temperatureRead();
          if (!manager.sendtoWait(packetGeneralStatusResponse.Byte, sizeof(packetGeneralStatusResponse.Byte), from))
            Serial.println("sendtoWait failed");
        } break;
        case 0xAA: { // Shock Command
          for (int i = 0; i < len; i++) packetShockCommandRequest.Byte[i] = buf[i];

          packetShockCommandResponse.Packet.messageID = packetShockCommandRequest.Packet.messageID;
          Serial.print(F("messageID: x")); Serial.println(packetShockCommandRequest.Packet.messageID);
          Serial.print(F("lastMessageID: x")); Serial.println(lastMessageID);
          Serial.print(F("pulseLength: x")); Serial.println(packetShockCommandRequest.Packet.pulseLength);
          Serial.print(F("pauseLength: x")); Serial.println(packetShockCommandRequest.Packet.pauseLength);
          Serial.print(F("repeatNum: x")); Serial.println(packetShockCommandRequest.Packet.repeatNum);
          bool shockValid = false;
          if (packetShockCommandRequest.Packet.messageID > lastMessageID) {
            Serial.println("Message ID is higher, lets ⚡️ shock this 🐱 cat");
            lastMessageID = packetShockCommandRequest.Packet.messageID;
            shockValid = true;
          }
          if (!manager.sendtoWait(packetShockCommandResponse.Byte, sizeof(packetShockCommandResponse.Byte), from))
            Serial.println("sendtoWait failed");
          if (shockValid) shockTheCat();
        } break;
      }
      // Send a reply back to the originator client
    }
  }
}
#ifdef OLDCODE
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
#endif
