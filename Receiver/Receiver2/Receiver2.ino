#define FRAME_SIZE 16      // number of bytes in a frame
#define START_BYTE 0x02    // first byte on a frame
#define SYNC_BYTE 0xAA     // used to synchronize the receiver
#define END_BYTE 0xFD      // last byte on a frame (logical inverse of START_BYTE)
#define PRESCALER 0x07     // ADC prescaler value (values in the table - 0x07 -> 9615 SPS)
#define TX_RATE 96.15      // Transmission rate in bps
#define NUM_SAMPLES_BIT 100// number of samples per received bit
#define THRESHOLD 2.5      // Threshold value, in Volts, to decide wether the incoming bit is 0 or 1
#define DEBUG_ON true      // Used to print debug messages on the serial monitor

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
void pushByteIntoFrame();
void verifyAndSendFrame();

// ----------- Arduino's functions --------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  pinMode(A1, INPUT);

  ADMUX = 0x00;
  ADMUX |= 0x40; // set the reference voltage as aVCC
  ADMUX |= 0x01; // pick analog input A0
 
  ADCSRA = 0x00;
  ADCSRA |= PRESCALER; // set the sampling frequency (prescaler) 128
  ADCSRA |= 0x08;      // turn interruptions on
  ADCSRA |= 0x20;      // turn auto-trigger on

  ADCSRB = 0x00;  // set the ADC to free running mode

  DIDR0 = 0xFF;   // disable digital input buffers

  SREG |= 0x80;   // turn global interruptions on

  ADCSRA |= 0x80; // turn the ADC on
  ADCSRA |= 0x40; // start the sampling
}

ISR(ADC_vect) {
  int sample = ADCL + (ADCH << 8);
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

void loop() {
  pushByteIntoFrame();
  verifyAndSendFrame();
}

// ----------- other functions ----------------------------------------------------------------------
void pushByteIntoFrame() {
  if (fullSampleBuffer) {
    int bitVal;
    long sampleSumCopy = sampleSum;
    sampleSum = 0;
    fullSampleBuffer = false;
    if (bitCounter < 8) {
      bitVal = sampleSumCopy * 5.0 / (1023.0 * NUM_SAMPLES_BIT) >= THRESHOLD;
      bitSum |= bitVal << bitCounter;
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
    for (int i = 0; i < FRAME_SIZE; i++) {
      Serial.print(frameBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    bool isFrame = (frameBuffer[0] == START_BYTE) &&
                   (frameBuffer[1] == SYNC_BYTE) &&
                   (frameBuffer[FRAME_SIZE - 2] == 0x00 || frameBuffer[FRAME_SIZE - 2] == 0x01) &&
                   (frameBuffer[FRAME_SIZE - 1] == END_BYTE);
   if (isFrame) {
    for (int i = 2; i < FRAME_SIZE - 2; i++) {
      Serial.print(frameBuffer[i]);
    }
    frameCounter = 0;
   } else {
    for (int i = 1; i < FRAME_SIZE; i++) {
      frameBuffer[i - 1] = frameBuffer[i];
    }
    frameCounter = FRAME_SIZE - 1;
   }
 }
}
