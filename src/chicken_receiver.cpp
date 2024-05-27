#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "rooster_protocol.h"

/////////////////////
//    Variables    //
/////////////////////

int InputX_Raw = 0; // Analog values range from 0-4095
int InputY_Raw = 0;

// Display for health of outgoing packets.
String TransmitHealth;

///////////////////
//    Defines    //
///////////////////

uint8_t SelfAddress[] = {0xA0, 0x76, 0x4E, 0x40, 0x2E, 0x14}; // TODO Maybe don't hardcode this
uint8_t RemoteAddress[] = {0x34, 0x85, 0x18, 0x03, 0x9b, 0x84}; // TODO Maybe don't hardcode this

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
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    TransmitHealth = "Healthy";
  }
  else{
    TransmitHealth = "!!WARN!!";
  }
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

}

void loop() {
  BuildTelemetryPacket();
  esp_err_t result = esp_now_send(RemoteAddress, (uint8_t *) &OutgoingPacket, sizeof(OutgoingPacket)); 
  Serial.print("Input X = ");
  Serial.println(InputX_Raw);
  Serial.print("Input Y = ");
  Serial.println(InputY_Raw);
  delay(100);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}