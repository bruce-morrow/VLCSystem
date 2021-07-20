#define FRAME_SIZE 16      // number of bytes in a frame
#define START_BYTE 0x02    // first byte on a frame
#define SYNC_BYTE 0xAA     // used to synchronize the receiver
#define END_BYTE 0xFD      // last byte on a frame (logical inverse of START_BYTE)
#define PD_PIN A0          // Arduino analog pin to connect the receiver photodiode
#define TX_RATE 1          // Transmission rate in bps
#define THRESHOLD 2.5      // Threshold value, in Volts, to decide wether the incoming bit is 0 or 1
#define NUM_SAMPLES_BIT 10 // Number of samples per bit
#define DEBUG_ON true      // Used to print debug messages on the serial monitor

/**
   Frame structure is:
   START_BYTE - SYNC_BYTE - 12 bytes of data - 1 byte for parity - END_BYTE
*/

// ----------- global variables -----------------------------------------------------------------------
byte frameBuffer[FRAME_SIZE];
const float bitTimeMs = 1000.0 / TX_RATE;

// ----------- functions' headers ---------------------------------------------------------------------
byte readBitFromPd(const unsigned int numSamples);
void waitStartByte();

// ----------- Arduino's functions --------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
}

void loop() {
  waitStartByte();
}

// ----------- other functions ----------------------------------------------------------------------
byte readBitFromPd(const unsigned int numSamples) {
  const float sampleTimeMs = bitTimeMs / numSamples;
  double value = 0.0;
  for (int i = 0; i < numSamples; i++) {
    value += analogRead(PD_PIN);
    delay(sampleTimeMs);
  }
  value *= 5.0 / (1023 * numSamples);
  byte decision = value >= THRESHOLD;
  if (DEBUG_ON) {
    Serial.print("Mean input value: ");
    Serial.print(value);
    Serial.print(",\tDecision: ");
    Serial.println(decision);
  }
  return decision;
}

void waitStartByte() {
  byte pdPinBit;
  byte oldPdPinBit = 0;
  bool gotZeroSequence = false;
  if (DEBUG_ON) Serial.println("Waiting for start byte");
  unsigned long startTime = millis();
  while (true) {
    pdPinBit = readBitFromPd(NUM_SAMPLES_BIT);
    if (pdPinBit ^ oldPdPinBit) {
      if (gotZeroSequence) {
        if (DEBUG_ON) Serial.println("Got start byte");
        return;
      }
      oldPdPinBit = pdPinBit;
      startTime = millis();
    } else if (oldPdPinBit == 0 && millis() - startTime >= 5.0 * bitTimeMs) {
      gotZeroSequence = true;
    }
  }
}
