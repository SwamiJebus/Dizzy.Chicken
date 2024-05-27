#ifndef ROOSTER_PROTOCOL_H
#define ROOSTER_PROTOCOL_H

int PacketType = 0;
// 0 = Unknown
// 1 = Transmitter
// 2 = Robot Receiver

int CMD_Channel1;
int CMD_Channel2;

int Status;
// 0 = Unknown
// 1 = Booting
// 2 = Healthy

typedef struct RoosterPacket
{
    int Ch1;
    int Ch2;
    int Status;
} RoosterPacket;

#endif