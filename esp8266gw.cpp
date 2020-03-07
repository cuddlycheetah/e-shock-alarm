#include <Arduino.h>


#include <ESP8266WiFi.h>
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>
#include <ArduinoJson.h>
// Update these with values suitable for your network.

#define MQTT_IP IPAddress(255.255.255.255)
#define MQTT_PORT 1883

const char* mqtt_user = "MQTT USER";
const char* mqtt_password = "MQTT PW";
WiFiClient espClient;
PubSubClient client(espClient);

StaticJsonDocument<200> doc;
void setupWifi();

void setupRadio();
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#define AESKEY "CHANGEMECHANGEME"
#define CLIENT_ADDRESS 0x01
#define SERVER_ADDRESS 0x02
RH_RF69 rf69(D8, D1);
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(rf69, CLIENT_ADDRESS);




void callback(char* topic, byte* payload, unsigned int length);
///////////////////////////////////////////////
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  Serial.println("INIT");
  setupRadio();
  setupWifi();
  Serial.println("READY");
}
void setupWifi() {
  WiFiManager wifiManager;
  wifiManager.autoConnect("CC-GW-2A");
  client.setServer(MQTT_IP, MQTT_PORT);
  client.setCallback(callback);
}
void setupRadio() {
  if (!manager.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(433.0))
    Serial.println("setFrequency failed");

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);
  //rf69.setModemConfig(RH_RF69::FSK_Rb2Fd5);
  rf69.setModemConfig(RH_RF69::GFSK_Rb2_4Fd4_8);

  //rf69.printRegisters();
  rf69.setEncryptionKey((uint8_t*)AESKEY);
  uint8_t syncwords[] = { 0xDE, 0xAD };
  rf69.setSyncWords(syncwords, sizeof(syncwords));
}

void callback(char* topic, byte* payload, unsigned int length);

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "shock-client";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password, "link/0x01/state", 2, true, "0")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("link/0x01/state", (uint8_t*)"1", 1, true);
      // ... and resubscribe
      client.subscribe("link/0x01/0x02/shock");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

long lastMsg = 0;
long lastBatteryAsk = -50e3;
char mqttMessage[50];
int value = 0;
void array_to_string(byte array[], unsigned int len, char buffer[]) {
  for (unsigned int i = 0; i < len; i++) {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  buffer[len*2] = '\0';
}

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
  bool shocked;
};
union PacketShockCommandResponseUnion {
  struct PacketShockCommandResponse Packet;
  uint8_t Byte[5];
};
PacketShockCommandResponseUnion packetShockCommandResponse;
#pragma endregion

void askGeneralStatus() {
  Serial.println("Sending 0XBA to 0x02");
  packetGeneralStatusRequest.Packet.head = 0xBA;
  uint64_t start = micros64();
  if (manager.sendtoWait(packetGeneralStatusRequest.Byte, sizeof(packetGeneralStatusRequest.Byte), SERVER_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      //snprintf (msg, 50, "hello world #%ld", value);
      //
      if (sizeof(PacketGeneralStatusResponse) != len) {
        Serial.print("Response has wrong Length! Expected: "); Serial.print(sizeof(PacketGeneralStatusResponseUnion)); Serial.print("recieved: "); Serial.println(len);
        return;
      }
      String mqttMessageS = String(mqttMessage);
      for (uint i=0; i < sizeof(PacketGeneralStatusResponse); i++)
        packetGeneralStatusResponse.Byte[i] = buf[i];
      uint64_t roundTripTime = micros64() - start;
      Serial.print("got reply from : 0x"); Serial.print(from, HEX); Serial.print(": "); array_to_string(buf, len, mqttMessage); Serial.println(mqttMessage);
      snprintf (mqttMessage, 50, "%d", packetGeneralStatusResponse.Packet.rfTemperature);
      client.publish("link/0x01/0x02/rftemp", mqttMessage);
      snprintf (mqttMessage, 50, "%d", packetGeneralStatusResponse.Packet.battery);
      client.publish("link/0x01/0x02/battery", mqttMessage);
      // RoundTripTime
      snprintf (mqttMessage, 50, "%lld", roundTripTime);
      client.publish("link/0x01/0x02/rtt", mqttMessage);
      //RSSI
      snprintf (mqttMessage, 50, "%d", rf69.lastRssi());
      client.publish("link/0x01/0x02/rssi", mqttMessage);
      // Linkstate
      client.publish("link/0x01/0x02/linkstate", "UP");
    } else {
      Serial.println("No reply, linkstate is DOWN");
      client.publish("link/0x01/0x02/linkstate", "DOWN");
    }
  } else {
    Serial.println("sendtoWait failed");
    client.publish("link/0x01/0x02/linkstate", "DOWN");
  }
}
void sendShockCommand() {
  // packetShockCommandRequest
  Serial.println("Sending 0XBA to 0x02");
  // Send a message to manager_server
  packetShockCommandRequest.Packet.head = 0xAA;
  uint64_t start = micros64();
  if (manager.sendtoWait(packetShockCommandRequest.Byte, sizeof(packetShockCommandRequest.Byte), SERVER_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      //snprintf (msg, 50, "hello world #%ld", value);
      //
      if (sizeof(PacketShockCommandResponse) != len) {
        Serial.print("Response has wrong Length! Expected: "); Serial.print(sizeof(PacketShockCommandResponseUnion)); Serial.print("recieved: "); Serial.println(len);
        return;
      }
      String mqttMessageS = String(mqttMessage);
      for (uint i=0; i < sizeof(PacketShockCommandResponse); i++)
        packetShockCommandResponse.Byte[i] = buf[i];
      uint64_t roundTripTime = micros64() - start;
      Serial.print("got reply from : 0x"); Serial.print(from, HEX); Serial.print(": "); array_to_string(buf, len, mqttMessage); Serial.println(mqttMessage);
      // Response MessageID
      snprintf (mqttMessage, 50, "%ld %d", packetShockCommandResponse.Packet.messageID, packetShockCommandResponse.Packet.shocked ? 1 : 0);
      client.publish("link/0x01/0x02/shock/response", mqttMessage);
      // RoundTripTime
      snprintf (mqttMessage, 50, "%lld", roundTripTime);
      client.publish("link/0x01/0x02/rtt", mqttMessage);
      //RSSI
      snprintf (mqttMessage, 50, "%d", rf69.lastRssi());
      client.publish("link/0x01/0x02/rssi", mqttMessage);
      // Linkstate
      client.publish("link/0x01/0x02/linkstate", "UP");
    } else {
      Serial.println("No reply, linkstate is DOWN");
      client.publish("link/0x01/0x02/linkstate", "DOWN");
    }
  } else {
    Serial.println("sendtoWait failed");
    client.publish("link/0x01/0x02/linkstate", "DOWN");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String lol = String((char*)payload);
  for (uint i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(err.c_str());
  }
  Serial.print(F("deserializeJson() succ "));
  Serial.println(err.c_str());

  packetShockCommandRequest.Packet.messageID    = doc[0].as<unsigned long>();
  packetShockCommandRequest.Packet.pulseLength  = doc[1].as<unsigned short>();
  packetShockCommandRequest.Packet.pauseLength  = doc[2].as<unsigned short>();
  packetShockCommandRequest.Packet.repeatNum    = doc[3].as<unsigned char>();
  if (doc.size() == 4) sendShockCommand();
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastBatteryAsk > 60e3) {
    lastBatteryAsk = now;
    askGeneralStatus();
  }
}
