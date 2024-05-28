#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
#include "rooster_protocol.h"

////////////////////////
//    Pin Mappings    //
////////////////////////

const auto MOTOR01_PIN = GPIO_NUM_0;

/////////////////////
//    Variables    //
/////////////////////

int InputX_Raw = 0; // Analog values range from 0-4095
int InputY_Raw = 0;

// Enum for health of outgoing packets.
esp_now_send_status_t TransmitHealth;
esp_err_t             RMTStatus;

///////////////////
//    Defines    //
///////////////////

const int FAILSAFE_DRIVE_THROTTLE   = 999;
const int AnalogDeadband            = 150;

const uint8_t SelfAddress[] = {0xA0, 0x76, 0x4E, 0x40, 0x2E, 0x14}; // TODO Maybe don't hardcode this
const uint8_t RemoteAddress[] = {0x34, 0x85, 0x18, 0x03, 0x9b, 0x84}; // TODO Maybe don't hardcode this

////////////////////////
//    Constructors    //
////////////////////////

RoosterPacket       OutgoingPacket;
RoosterPacket       IncomingPacket;
esp_now_peer_info_t PeerInfo;

/////////////////////
//    Functions    //
/////////////////////

// Callback when data is sent.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  TransmitHealth = status;
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&IncomingPacket, incomingData, sizeof(IncomingPacket));
  InputX_Raw = IncomingPacket.Ch1;
  InputY_Raw = IncomingPacket.Ch2;
}

// Set device health variables.
void BuildTelemetryPacket()
{
  OutgoingPacket.Status = 1; // Situation normal, how are you?
}

void SendDriveESCThrottle(int rawAnalog)
{
  if (TransmitHealth == ESP_NOW_SEND_SUCCESS)
  {
    //esc.sendThrottleValue(abs(rawAnalog - 2047 ) > AnalogDeadband ? rawAnalog*0.488 : 999);
  }
  else
  {
    //esc.sendThrottleValue(FAILSAFE_DRIVE_THROTTLE);
  }
}

///////////////////
//    SETUP      //
///////////////////

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);

  ////// Initialize ESP-NOW. //////
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(PeerInfo.peer_addr, RemoteAddress, 6);
  PeerInfo.channel = 0;
  PeerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&PeerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  ////// Initialize ESCs //////
}

void loop() {
  BuildTelemetryPacket();
  esp_err_t result = esp_now_send(RemoteAddress, (uint8_t *) &OutgoingPacket, sizeof(OutgoingPacket)); 
  //SendDriveESCThrottle(Motor01, InputX_Raw);
  Serial.println(RMTStatus);
  delay(10);
}