#define FRAME_SIZE 16      // number of bytes in a frame
#define START_BYTE 0x02    // first byte on a frame
#define SYNC_BYTE 0xAA     // used to synchronize the receiver
#define END_BYTE 0xFD      // last byte on a frame (logical inverse of START_BYTE)
#define PRESCALER 0x07     // ADC prescaler value (values in the table - 0x07 -> 9615 SPS)
#define TX_RATE 1000       // Transmission rate in bps
#define NUM_SAMPLES_BIT 10 // number of samples per received bit
#define THRESHOLD 2.5      // Threshold value, in Volts, to decide wether the incoming bit is 0 or 1
#define PD_PIN 1           // Analog pin connected to the photdiode
#define DEBUG_ON true      // Used to print debug messages on the serial monitor

#include <TimerOne.h>

/**
   Frame structure is:
   START_BYTE - SYNC_BYTE - 12 bytes of data - 1 byte for parity - END_BYTE
*/

// ----------- global variables -----------------------------------------------------------------------
long sampleSum = 0;
bool fullSampleBuffer = false;
unsigned int sampleCounter = 0;

int bitSum = 0;
bool fullBitBuffer = false;
unsigned int bitCounter = 0;

int frameBuffer[FRAME_SIZE];
bool fullFrameBuffer = false;
unsigned int frameCounter = 0;

// ----------- functions' headers ---------------------------------------------------------------------
void adcSetup();
void adcStartConversion(int adcPin);
int adcReadConversion();
void timer1Setup();
void sampleBit();
void pushByteIntoFrame();
void verifyAndSendFrame();

// ----------- Arduino's functions --------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  adcSetup();
  timer1Setup();
}

void loop() {
  pushByteIntoFrame();
  verifyAndSendFrame();
}

// ----------- ADC functions ------------------------------------------------------------------------
void adcSetup() {
  ADCSRA =  bit (ADEN);                                // turn ADC on
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  // Prescaler of 128
  ADMUX  =  bit (REFS0);                               // external 5v reference
}

void adcStartConversion(int adcPin) {
  ADMUX &= ~(0x07);           //clearing enabled channels
  ADMUX  |= (adcPin & 0x07);  // AVcc and select input port
  bitSet(ADCSRA, ADSC);
}

int adcReadConversion() {
 while(bit_is_set(ADCSRA, ADSC));
 return ADC;
}

// ----------- Timer functions ----------------------------------------------------------------------
void timer1Setup() {
  Timer1.initialize(1 / (TX_RATE * NUM_SAMPLES_BIT));
  Timer1.attachInterrupt(sampleBit);
}

// ----------- Data functions -----------------------------------------------------------------------
void sampleBit() {
  int sample = adcReadConversion();
  adcStartConversion(PD_PIN);
  Serial.print("Sample = "); Serial.println(sample);
  if (!fullSampleBuffer) {
    if (sampleCounter < NUM_SAMPLES_BIT) {
      sampleSum += sample;
      sampleCounter++;
    }
    if (sampleCounter == NUM_SAMPLES_BIT) {
      fullSampleBuffer = true;
      sampleCounter = 0;
    }
  }
}

void pushByteIntoFrame() {
  if (fullSampleBuffer) {
    byte bitVal;
    long sampleSumCopy = sampleSum;
    sampleSum = 0;
    fullSampleBuffer = false;
    if (bitCounter < 8) {
      bitVal = ((float) sampleSumCopy * 5.0 / (1023.0 * NUM_SAMPLES_BIT)) >= THRESHOLD;
      bitSum |= (bitVal << bitCounter);
      bitCounter++;
    }
    if (bitCounter == 8) {
      frameBuffer[frameCounter] = bitSum;
      frameCounter++;
      bitSum = 0;
      bitCounter = 0;
    }
  }
}

void verifyAndSendFrame() {
  if (frameCounter == FRAME_SIZE) {
    if (DEBUG_ON) {
      for (int i = 0; i < FRAME_SIZE; i++) {
      Serial.print(frameBuffer[i], HEX);
      Serial.print(" ");
  }
  Serial.println();
    }
    if ((frameBuffer[0] & START_BYTE) &&
       (frameBuffer[1] & SYNC_BYTE) &&
       (frameBuffer[FRAME_SIZE - 2] & 0x00 || frameBuffer[FRAME_SIZE - 2] & 0x01) &&
       (frameBuffer[FRAME_SIZE - 1] & END_BYTE)) {
      for (int i = 2; i < FRAME_SIZE - 2; i++) Serial.write(frameBuffer[i]);
      frameCounter = 0;
    } else {
      for (int i = 1; i < FRAME_SIZE; i++) {
        frameBuffer[i - 1] = frameBuffer[i];
      }
      frameCounter = FRAME_SIZE - 1;
    }
 }
}
