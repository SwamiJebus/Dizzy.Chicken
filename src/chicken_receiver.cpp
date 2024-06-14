#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
#include <Servo.h>
#include "rooster_protocol.h"

////////////////////////
//    Pin Mappings    //
////////////////////////

const int MOTOR01_PIN = D0;
const int MOTOR02_PIN = D1;

/////////////////////
//    Variables    //
/////////////////////

int InputX_Raw = 0; // Analog values range from 0-4095
int InputY_Raw = 0;

// Enum for health of outgoing packets.
esp_now_send_status_t TransmitHealth;

///////////////////
//    Defines    //
///////////////////

const int FAILSAFE_DRIVE_THROTTLE   = 1500;
const int AnalogDeadband            = 70;

const uint8_t SelfAddress[] = {0xA0, 0x76, 0x4E, 0x40, 0x2E, 0x14}; // TODO Maybe don't hardcode this
const uint8_t RemoteAddress[] = {0x34, 0x85, 0x18, 0x03, 0x9b, 0x84}; // TODO Maybe don't hardcode this

////////////////////////
//    Constructors    //
////////////////////////

RoosterPacket       OutgoingPacket;
RoosterPacket       IncomingPacket;
esp_now_peer_info_t PeerInfo;

Servo ServoPWM = Servo();

/////////////////////
//    Functions    //
/////////////////////

// Callback when data is sent.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  TransmitHealth = status;
}

void SendDriveThrottle(int analogInput, Servo servo, int pin)
{
  Serial.print("Motor ");
  Serial.print(pin);
  Serial.print(": ");
  if (abs(analogInput - 2048) > AnalogDeadband)
  {
    int input_pwm_scaled = map(analogInput, 0, 4095, 1000, 2000);
    Serial.println(input_pwm_scaled);
    servo.writeMicroseconds(pin, input_pwm_scaled);
    
  }
  else
  {
    Serial.println("1500");
    servo.writeMicroseconds(pin, 1500);
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

  // Go to failsafe if we lose connection
  if (TransmitHealth == 1)
  {
    SendDriveThrottle(InputX_Raw, ServoPWM, MOTOR01_PIN);
    SendDriveThrottle(InputY_Raw, ServoPWM, MOTOR02_PIN);
  }
  else
  {
    Serial.println("No connection to TX!");
    ServoPWM.writeMicroseconds(MOTOR01_PIN, FAILSAFE_DRIVE_THROTTLE);
    ServoPWM.writeMicroseconds(MOTOR02_PIN, FAILSAFE_DRIVE_THROTTLE);
  }
  delay(10);
}