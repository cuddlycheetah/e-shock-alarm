#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#define CLIENT_ADDRESS 0x02
#define SERVER_ADDRESS 2
#define PULSE 3

RH_RF69 rf69(SS, 2);
RHReliableDatagram manager(rf69, SERVER_ADDRESS);

void setup() {
  Serial.begin(9600);
  Serial.println("start");
  pinMode(PULSE, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(PULSE, LOW);
  digitalWrite(13, HIGH);
  if (!manager.init()) Serial.println("Init of Modem failed!");
  if (!rf69.setFrequency(434.0)) Serial.println("setFrequency failed");
  rf69.setTxPower(13, true);
  rf69.setModemConfig(RH_RF69::FSK_Rb2Fd5);
  Serial.println(rf69.temperatureRead());
  rf69.setEncryptionKey((uint8_t*)"1234567891011121");
  uint8_t syncwords[] = { 0xDE, 0xAD };
  rf69.setSyncWords(syncwords, sizeof(syncwords));

  Serial.println("sending init packet");
  uint8_t data[] = "123Test";
  rf69.send(data, sizeof(data));
  rf69.waitPacketSent();
  Serial.println("done");
  Serial.flush();
  digitalWrite(13, LOW);
}


void loop() {

  if (manager.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len))
    {
      Serial.print(F("headerFrom: 0x")); Serial.println(rf69.headerFrom(), HEX);
      Serial.print(F("headerTo: 0x")); Serial.println(rf69.headerTo(), HEX);
      Serial.print(F("headerId: 0x")); Serial.println(rf69.headerId(), HEX);
      Serial.print(F("headerFlags: 0x")); Serial.println(rf69.headerFlags(), HEX);
      RH_RF69::printBuffer("request: ", buf, len);
      if (rf69. headerTo() != CLIENT_ADDRESS) {
        Serial.println("Ignoring Packet, because its not for me!");
        return;
      }
      Serial.print("got valid request for me: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);
      Serial.print("len: ");
      Serial.println(len, DEC);
      
      delay(100);
      // send ACK
      uint8_t data[] = { };
      digitalWrite(13, HIGH);
      if (len == 3) {
        delay(500);
        rf69.setHeaderTo(rf69.headerFrom());
        rf69.setHeaderFrom(CLIENT_ADDRESS);
        rf69.setHeaderFlags(0x80);
        rf69.send(data, sizeof(data));
        rf69.waitPacketSent();
        Serial.println("ACK Sent");
        Serial.println();
        
        uint8_t pulseLengthFactor = buf[0];
        uint8_t pulsePauseFactor = buf[1];
        uint8_t pulseRepeat = buf[2];

        Serial.print(F("pulseLengthFactor: x")); Serial.println(pulseLengthFactor);
        Serial.print(F("pulsePauseFactor: x")); Serial.println(pulsePauseFactor);
        Serial.print(F("pulseRepeat: x")); Serial.println(pulseRepeat);

        for (int i=0;i < pulseRepeat; i++) {
          digitalWrite(PULSE, HIGH);
          delayMicroseconds(10 * pulseLengthFactor);
          digitalWrite(PULSE, LOW);
          delayMicroseconds(10 * pulsePauseFactor);
        }
      }
      digitalWrite(13, LOW);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}
