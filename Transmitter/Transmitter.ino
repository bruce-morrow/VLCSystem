#define FRAME_SIZE 16   // number of bytes in a frame
#define START_BYTE 0x02 // first byte on a frame (try 0x02 then)
#define SYNC_BYTE 0xAA  // used to synchronize the receiver
#define END_BYTE 0xFD   // last byte on a frame (logical inverse of START_BYTE)
#define LED_PIN 13      // Arduino digital pin to connect the transmitter LED
#define TX_RATE 10      // Transmission rate in bps
#define DEBUG_ON true   // Used to print debug messages on the serial monitor

/**
   Frame structure is:
   START_BYTE - SYNC_BYTE - 12 bytes of data - 1 byte for parity - END_BYTE
*/

// ----------- global variables -----------------------------------------------------------------------
byte dataBuffer[FRAME_SIZE];
byte frameBuffer[FRAME_SIZE];
const float bitTimeMs = 1000.0 / TX_RATE;

// ----------- functions' headers ---------------------------------------------------------------------
void byte2bits(byte b, int* bits);
void buildFrame(byte* dataBytes, byte* frame);
void sendFrame(byte* frame);
byte calculateParity(byte* data, int len);
void printDataBuffer();
void printFrameBuffer();

// ----------- Arduino's functions --------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (Serial.available() == FRAME_SIZE - 3) {  // 3 comes from the number of non data bytes (4) minus the enter on the de end (1 byte)
    // read the data
    Serial.readBytes(dataBuffer, FRAME_SIZE);
    // read the final enter (not of our interest)
    Serial.read();
    buildFrame(dataBuffer, frameBuffer);
    if (DEBUG_ON) {
      printDataBuffer();
      printFrameBuffer();
    }
    sendFrame(frameBuffer);
  }
}

// ----------- other functions ----------------------------------------------------------------------
void byte2bits(byte b, int* bits) {
  for (int i = 7; i >= 0; i--) {
    bits[7 - i] = (b >> i) & 0x01;
  }
}

void buildFrame(byte* dataBytes, byte* frame) {
  frame[0] = START_BYTE;
  frame[1] = SYNC_BYTE;
  for (int i = 0; i < FRAME_SIZE - 4; i++) {
    frame[i + 2] = dataBytes[i];
  }
  frame[FRAME_SIZE - 2] = calculateParity(dataBytes, FRAME_SIZE - 4);
  frame[FRAME_SIZE - 1] = END_BYTE;
}

void sendFrame(byte* frame) {
  int bits[8];
  for (int i = 0; i < FRAME_SIZE; i++) {
    byte2bits(frame[i], bits);
    for (int j = 0; j < 8; j++) {
      int b = bits[j];
      if (b == 0) digitalWrite(LED_PIN, LOW);
      else digitalWrite(LED_PIN, HIGH);
      delay(bitTimeMs);  // TODO: this should later be changed to an interruption
    }
  }
  if (DEBUG_ON) {
    Serial.println("End of trasmission");
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

void printDataBuffer() {
  Serial.print("Data buffer: ");
  Serial.write(dataBuffer, FRAME_SIZE - 4);
  Serial.println();
}

void printFrameBuffer() {
  Serial.print("Frame buffer: ");
  Serial.print(frameBuffer[0], HEX);
  Serial.print(frameBuffer[1], HEX);
  for (int i = 2; i < FRAME_SIZE - 2; i++) {
    Serial.write(frameBuffer[i]);
  }
  Serial.print(frameBuffer[FRAME_SIZE - 2], HEX);
  Serial.println(frameBuffer[FRAME_SIZE - 1], HEX);
}
