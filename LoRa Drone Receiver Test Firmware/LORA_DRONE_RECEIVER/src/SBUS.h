#include <Arduino.h>

// Adapted from https://blog.quadmeup.com/2017/10/25/generate-s-bus-with-arduino-in-a-simple-way/

#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010
#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0F
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE  15 

// SBUS channels
#define ROLL 0 // A
#define PITCH 1 // E
#define THROTTLE 2 // T
#define YAW 3  // R
#define AUX1 4 // ARM
#define AUX2 5 // ANGLE/HORIZON
#define AUX3 6 // FAILSAFE

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe);

void printSBUSPacket(uint8_t packet[]);
void printSBUSChannel(int channels[]);