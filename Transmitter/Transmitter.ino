#define FRAME_SIZE  16
#define LED_PIN 13
#define DEBUG_ON true
#define FREQUENCY 1

void byte2bits(byte b, int* bits);
void sendFrame(byte* frame);
void printBuffer(byte* b, int len);

byte frameBuffer[FRAME_SIZE]; // 1 frame start byte - 13 data bytes - 1 parity byte - 1 frame end byte
const float bitTime = 1.0 / (2.0 * FREQUENCY); 

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (Serial.available() == FRAME_SIZE + 1) {
    Serial.readBytes(frameBuffer, FRAME_SIZE);
    Serial.read();
    if (DEBUG_ON) {
      printBuffer(frameBuffer, FRAME_SIZE);
    }
    sendFrame(frameBuffer);
  }
}

void byte2bits(byte b, int* bits) {
  for (int i = 7; i >= 0; i--) {
    bits[7 - i] = (b >> i) & 0x01;
  }
}

void sendFrame(byte* frame) {
  int bits[8];
  for (int i = 0; i < FRAME_SIZE; i++) {
    byte2bits(frame[i], bits);
    for (int j = 0; j < 8; j++) {
      int b = bits[j];
      if (b == 0) digitalWrite(LED_PIN, LOW);
      else digitalWrite(LED_PIN, HIGH);
      delay(1000 * bitTime);
    }
  }
  if (DEBUG_ON) {
    Serial.println("End of trasmission");
  }
}

void printBuffer(byte* b, int len) {
  Serial.print("Buffer data: ");
  Serial.write(b, len);
  Serial.println();
}
