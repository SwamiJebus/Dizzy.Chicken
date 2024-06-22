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

const int MOTOR_LL_PIN      = D0;
const int MOTOR_LU_PIN      = D1;
const int ENCODER_L_DIR_PIN = D10;

/////////////////////
//    Variables    //
/////////////////////

// Analog values range from 0-4095
int Ch1_Raw = 0;
int Ch2_Raw = 0;

int cmdTime;

// Enum for health of outgoing packets.
esp_now_send_status_t TransmitHealth;

///////////////////
//    Defines    //
///////////////////

const int FAILSAFE_DRIVE_THROTTLE   = 1500;
const int AnalogDeadband            = 70;
const int PWMMidThrottle            = 1500;

const uint8_t SelfAddress[] = {0xA0, 0x76, 0x4E, 0x40, 0x2E, 0x14}; // TODO Maybe don't hardcode this
const uint8_t RemoteAddress[] = {0x34, 0x85, 0x18, 0x03, 0x9b, 0x84}; // TODO Maybe don't hardcode this

struct pivotCommand 
{
  int PivotDirection, RollDirection, AngleDelta;
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

float EaseInSine(float x)
{
  return 450 * (-cos(x/325.949)/2 + 0.5); // Max pivot speed = 500 us
}

// Callback when data is sent.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  TransmitHealth = status;
}

pivotCommand CalculatePivot(int currentHeading, int targetHeading)
{
  pivotCommand command;

  int delta = ((((targetHeading - currentHeading + 1024) % 2048) + 2048) % 2048) - 1024; // Trust me, bro.

  command.PivotDirection = delta > 0 ? -1: 1;
  command.RollDirection  = 1; // TODO Fix this
  command.AngleDelta = abs(delta);

  return command;
}

void SendDriveThrottle()
{
  int CartesianCh1 = Ch1_Raw - 2048;
  int CartesianCh2 = Ch2_Raw - 2048;

  if (abs(CartesianCh1) > AnalogDeadband || abs(CartesianCh2) > AnalogDeadband)
  {
    float polarR = sqrt(pow(CartesianCh1,2) + pow(CartesianCh2,2));
    float polarT = atan2(CartesianCh1, -CartesianCh2);

    int commandHeading = (int)(((polarT + 3.14) / 6.28) * 4095);
    int currentHeading = abs(Encoder_L.readAngle());

    int pivotSpeed  = 0;
    int rollSpeed   = 0;

    pivotCommand pivot = CalculatePivot(currentHeading, commandHeading);

    pivotSpeed = EaseInSine(pivot.AngleDelta)*pivot.PivotDirection;

    if (pivot.AngleDelta < 512)
    {
      rollSpeed = 0; //map(polarR, 120, 1800, 30, 499);
    }

    ServoPWM.writeMicroseconds(MOTOR_LL_PIN, PWMMidThrottle+pivotSpeed-(rollSpeed*pivot.RollDirection));
    ServoPWM.writeMicroseconds(MOTOR_LU_PIN, PWMMidThrottle+pivotSpeed+(rollSpeed*pivot.RollDirection));
  }
  else
  {
    ServoPWM.writeMicroseconds(MOTOR_LL_PIN, PWMMidThrottle);
    ServoPWM.writeMicroseconds(MOTOR_LU_PIN, PWMMidThrottle);
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
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

void setup() 
{
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
  Encoder_L.begin(ENCODER_L_DIR_PIN);
  //Encoder_L.setAddress(0x40);   //  Simply use better hardware in the AS5600L
  Encoder_L.setDirection(AS5600_COUNTERCLOCK_WISE);  //  default, just be explicit.
}

void loop() 
{
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
    delay(25);
  }
}