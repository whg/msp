
class Spectrum {
public:
  Spectrum(uint8_t strobePin=4, uint8_t resetPin=5, 
           uint8_t channel1Pin=A0, uint8_t channel2Pin=A1):
           mStrobePin(strobePin), mResetPin(resetPin) {

    mPins[0] = channel1Pin;
    mPins[1] = channel2Pin;

    // reset data
    for (uint8_t freq = 0; freq < 7; ++freq) {
      for (uint8_t chan = 0; chan < 2; ++chan) {
        mData[chan][freq] = 0;
      }
    }
  }

  void init() {
    pinMode(mStrobePin, OUTPUT);
    pinMode(mResetPin, OUTPUT);
    pinMode(mPins[0], INPUT);
    pinMode(mPins[1], INPUT);  
    
    digitalWrite(mStrobePin, HIGH);
    digitalWrite(mResetPin, HIGH);
  
    //Initialize Spectrum Analyzers
    digitalWrite(mStrobePin, LOW);
    delay(1);
    digitalWrite(mResetPin, HIGH);
    delay(1);
    digitalWrite(mStrobePin, HIGH);
    delay(1);
    digitalWrite(mStrobePin, LOW);
    delay(1);
    digitalWrite(mResetPin, LOW);
  }
  
  void update() {

    for (uint8_t freq = 0; freq < 7; ++freq) {
      for (uint8_t chan = 0; chan < 2; ++chan) {
        mData[chan][freq] = analogRead(mPins[chan]);
      }
      digitalWrite(STROBE, HIGH);
      digitalWrite(STROBE, LOW);
    }   
  }

  uint16_t* getData() { return (uint16_t*) mData; }

protected:
  uint16_t mData[2][7];
  uint8_t mPins[2];
  uint8_t mStrobePin, mResetPin;
};

uint32_t counter = 0;

const uint8_t DATA_LEN = 14 * 2 + 2;
uint8_t data[DATA_LEN];

Spectrum spectrum;

void setup() {
  Serial.begin(9600);

  spectrum.init();
}

void loop() {

  spectrum.update();

  data[0] = 12;
  memcpy((void*)&data[1], (void*) spectrum.getData(), sizeof(uint16_t) * 14);    
  
  data[DATA_LEN-1] = 11;
  Serial.write((byte*)data, DATA_LEN);
  
  delay(50);
}
