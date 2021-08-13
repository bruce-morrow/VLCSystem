#define FRAME_SIZE 16   // number of bytes in a frame
#define START_BYTE 0x02 // first byte on a frame (try 0x02 then)
#define SYNC_BYTE 0xAA  // used to synchronize the receiver
#define END_BYTE 0xFD   // last byte on a frame (logical inverse of START_BYTE)
#define LED_PIN 13      // Arduino digital pin to connect the transmitter LED
#define TX_RATE 1000    // Transmission rate in bps
#define DEBUG_ON true   // Used to print debug messages on the serial monitor

#include <TimerOne.h>

/**
   Frame structure is:
   START_BYTE - SYNC_BYTE - 12 bytes of data - 1 byte for parity - END_BYTE
*/

// ----------- global variables -----------------------------------------------------------------------
byte bitsBuffer[FRAME_SIZE][8];
bool isFrameReady = false;

// ----------- functions' headers ---------------------------------------------------------------------
void timer1Setup();
void byte2bits(byte b, int* bits);
void sendBit();
void buildFrame();
byte calculateParity(byte* data, int len);
void printDataBuffer(byte* dataBuffer);

// ----------- Arduino's functions --------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  timer1Setup();
}

void loop() {
  buildFrame();
}

// ----------- timer1 functions ---------------------------------------------------------------------
void timer1Setup() {
  Timer1.initialize(1 / TX_RATE);
  Timer1.attachInterrupt(sendBit);
}
// ----------- other functions ----------------------------------------------------------------------
void byte2bits(byte b, byte* bits) {
  for (int i = 7; i >= 0; i--) {
    bits[7 - i] = (b >> i) & 0x01;
  }
}

void buildFrame() {
  static byte dataBuffer[FRAME_SIZE];
  Serial.println("BEFORE");
  while(isFrameReady);
  Serial.println("HERE");
  if (Serial.available() == FRAME_SIZE - 3) {  // 3 comes from the number of non data bytes (4) minus the enter on the de end (1 byte)
    Serial.readBytes(dataBuffer, FRAME_SIZE);
    // read the final enter (not of our interest)
    Serial.read();
    if (DEBUG_ON) printDataBuffer(dataBuffer);
    byte2bits(START_BYTE, bitsBuffer[0]);
    byte2bits(SYNC_BYTE, bitsBuffer[1]);
    for (int i = 0; i < FRAME_SIZE - 4; i++) {
      byte2bits(dataBuffer[i], bitsBuffer[i + 2]);
    }
    byte2bits(calculateParity(dataBuffer, FRAME_SIZE - 4), bitsBuffer[FRAME_SIZE - 2]);
    byte2bits(END_BYTE, bitsBuffer[FRAME_SIZE - 1]);
    isFrameReady = true;
  }
}

void sendBit() {
  static bool idleBit = true;
  static int bitCounter = 0;
  if (isFrameReady) {
    digitalWrite(LED_PIN, bitsBuffer[bitCounter % 8][(int) bitCounter / 8]);
    bitCounter++;
    if (bitCounter == FRAME_SIZE * 8) {
      isFrameReady = false;
      bitCounter = 0;
      if (DEBUG_ON) Serial.println("End of tranmission");
    }
  } else {
    digitalWrite(LED_PIN, idleBit);
    idleBit = !idleBit;
  }
}

byte calculateParity(byte* data, int len) {
  // concatenate the data bytes
  int n = 0;
  for (int i = len - 1; i >= 0; i--) {
    n += data[i];
    n <<= 8;
  }
  // calculate the parity itself
  byte parity = 0;
  while (n != 0) {
    parity = !parity;
    n = n & (n - 1);
  }
  return parity;
}

void printDataBuffer(byte* dataBuffer) {
  Serial.print("Data buffer: ");
  Serial.write(dataBuffer, FRAME_SIZE - 4);
  Serial.println();
}
