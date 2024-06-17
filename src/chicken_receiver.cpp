#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
#include <Servo.h>
#include <AS5600.h>
#include <tuple>
#include "rooster_protocol.h"

////////////////////////
//    Pin Mappings    //
////////////////////////

const int MOTOR_LL_PIN   = D0;
const int MOTOR_LU_PIN   = D1;

/////////////////////
//    Variables    //
/////////////////////

// Analog values range from 0-4095
int Ch1_Raw = 0;
int Ch2_Raw = 0;

// Enum for health of outgoing packets.
esp_now_send_status_t TransmitHealth;

///////////////////
//    Defines    //
///////////////////

const int FAILSAFE_DRIVE_THROTTLE   = 1500;
const int AnalogDeadband            = 70;

const uint8_t SelfAddress[] = {0xA0, 0x76, 0x4E, 0x40, 0x2E, 0x14}; // TODO Maybe don't hardcode this
const uint8_t RemoteAddress[] = {0x34, 0x85, 0x18, 0x03, 0x9b, 0x84}; // TODO Maybe don't hardcode this

struct pivotCommand {
  int PivotDirection, RollDirection, PivotScale;
};

////////////////////////
//    Constructors    //
////////////////////////

RoosterPacket       OutgoingPacket;
RoosterPacket       IncomingPacket;
esp_now_peer_info_t PeerInfo;

Servo ServoPWM = Servo();
AS5600 Encoder_L = AS5600();   //  use default Wire

/////////////////////
//    Functions    //
/////////////////////

// Callback when data is sent.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  TransmitHealth = status;
}

pivotCommand CalculatePivot(int currentHeading, int targetHeading)
{
  pivotCommand command;

  int delta = (((targetHeading - currentHeading + 1024 % 2048) + 2048) % 2048) - 1024; // Trust me, bro.

  // Determine pivot direction
  command.PivotDirection = delta < 0 ? -1: 1;
    // Determine roll direction
  command.RollDirection  = (abs(delta > 1024)) ? -1: 1;
  // Use delta as arbitrary scale for pivot speed.
  command.PivotScale = delta;

  return command;
}

void SendDriveThrottle()
{
  if (abs(Ch1_Raw - 2048) > AnalogDeadband || abs(Ch2_Raw - 2048) > AnalogDeadband)
  {
    //float polarR = sqrt(pow(IncomingPacket.Ch1,2) + pow(IncomingPacket.Ch2,2));
    float polarT = atan2(Ch1_Raw - 2048, -(Ch2_Raw - 2048));

    int commandHeading = (int)(((polarT + 3.14) / 6.28) * 4095) ; // Need to achieve this angle.
    int currentHeading = Encoder_L.readAngle();                 // We are here.

    pivotCommand pivot = CalculatePivot(currentHeading, commandHeading);

    Serial.print("(");
    Serial.print(currentHeading);
    Serial.print(",");
    Serial.print(commandHeading);
    Serial.print(")");


    Serial.print("(");
    Serial.print(pivot.PivotScale);
    Serial.print(",");
    Serial.print(pivot.PivotDirection);
    Serial.println(")");

    ServoPWM.writeMicroseconds(MOTOR_LL_PIN, pivot.PivotDirection*70 + 1500);
    ServoPWM.writeMicroseconds(MOTOR_LU_PIN, pivot.PivotDirection*70 + 1500);
  }
  else
  {
    ServoPWM.writeMicroseconds(MOTOR_LL_PIN, 1500);
    ServoPWM.writeMicroseconds(MOTOR_LU_PIN, 1500);
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&IncomingPacket, incomingData, sizeof(IncomingPacket));
  Ch1_Raw = IncomingPacket.Ch1;
  Ch2_Raw = IncomingPacket.Ch2;
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
    return;
  }

  // Initialize Encoders //
  Wire.begin(); // Initialize I2C on default pins.
  Encoder_L.begin();
  //Encoder_L.setAddress(0x40);   //  Simply use better hardware in the AS5600L
  Encoder_L.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
}

void loop() {
  BuildTelemetryPacket();
  esp_err_t result = esp_now_send(RemoteAddress, (uint8_t *) &OutgoingPacket, sizeof(OutgoingPacket)); 

  if (TransmitHealth == 1)
  {
    SendDriveThrottle();
  }
  else
  {
    // Go to failsafe if we lose connection
    ServoPWM.writeMicroseconds(MOTOR_LL_PIN, FAILSAFE_DRIVE_THROTTLE);
    ServoPWM.writeMicroseconds(MOTOR_LU_PIN, FAILSAFE_DRIVE_THROTTLE);
  }

  delay(50);
}