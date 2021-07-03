#define FRAME_SIZE  16
#define LED_PIN 13

void byte2bits(byte b, int* bits);
void sendFrame(byte* frame);

byte frameBuffer[FRAME_SIZE]; // 1 frame start byte - 13 data bytes - 1 parity byte - 1 frame end byte

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    Serial.readBytes(frameBuffer, 16);
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
    }
  }
}
