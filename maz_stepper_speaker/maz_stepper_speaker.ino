
struct TimedAction {
  TimedAction(uint32_t interval): 
  mActionInterval(interval), mLastUpdate(0) {}

  uint8_t tick(uint32_t currentMicros) {
    uint8_t output = 0;
    if (currentMicros - mLastUpdate > mActionInterval) {
      output = 1;
      mLastUpdate = currentMicros;
    }
    return output;
  }

  void setInterval(uint32_t interval) { mActionInterval = interval; }
  
  uint32_t mLastUpdate, mActionInterval;
};


class StepperDriver : public TimedAction {
public:
  StepperDriver(uint8_t stepPin, uint8_t dirPin, uint32_t stepSpeed=1000):
    TimedAction(stepSpeed),
    mStepPin(stepPin), mDirectionPin(dirPin), 
    mCurrentPosition(0), mTargetPosition(0),
    mStartPosition(0), mEndPosition(0),
    mStep(0) {

      pinMode(mStepPin, OUTPUT);
      pinMode(mDirectionPin, OUTPUT);   
  } 

  void setRange(uint16_t start, uint16_t end) {
    mStartPosition = start;
    mEndPosition = end;
  }

  void setTarget(uint16_t target) {
    mTargetPosition = target;

    mDirection = mCurrentPosition < mTargetPosition;
    digitalWrite(mDirectionPin, mDirection);
  }

  void update(uint32_t microsTime) {

    if (abs(mCurrentPosition - mTargetPosition) > 3) {
    
      if (tick(microsTime)) {
        digitalWrite(mStepPin, mStep);
        mStep ^= 1;
        mCurrentPosition+= mDirection ? 1 : -1;
//        char msg[50];
//        sprintf(msg, "%d, %d, %d", mCurrentPosition, mTargetPosition, mStep);
//        Serial.println(msg);
      }
    }
    
  }
  
protected:
  uint8_t mStepPin, mDirectionPin;
  int16_t mCurrentPosition, mTargetPosition;
  int16_t mStartPosition, mEndPosition;
  uint16_t mStepTime;

  uint8_t mStep, mDirection;
  
};


struct UltrasonicReader : protected TimedAction  {
  UltrasonicReader(uint8_t echoPin, uint8_t triggerPin, uint32_t updateTime=1000000):
    TimedAction(updateTime),
    mEchoPin(echoPin), mTriggerPin(triggerPin),
    mDistance(0) {
      
  }

  uint16_t update(uint32_t currentMicros) {
      
      if (tick(currentMicros)) {
        
        digitalWrite(mTriggerPin, LOW);
        delayMicroseconds(2);
        digitalWrite(mTriggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(mTriggerPin, LOW);
        mDistance = pulseIn(mEchoPin, HIGH);
        mDistance /= 58;

        Serial.println(mDistance);
      }

      return mDistance;
  }

  uint8_t mEchoPin, mTriggerPin;
  uint16_t mDistance;
};

//template<typename T>
uint16_t clamp(uint16_t v, uint16_t min, uint16_t max) {
  if (v > max) {
    v = max;
  }
  else if (v < min) {
    v = min;
  }
  return v;
}

StepperDriver stepper(7, 8);
UltrasonicReader distanceReader(4, 5);

uint32_t currentTime;

// as you go further out, it goes anti clockwise
// from bottom: yellow, red, gree, white

void setup() {
  Serial.begin(9600);
  stepper.setTarget(100);  

  pinMode(A5, INPUT);
}
uint16_t counter = 0;
void loop() {

  currentTime = micros();

 uint16_t distance = distanceReader.update(currentTime);
 distance = clamp(distance, 10, 300);
 uint16_t q = map(distance, 10, 300, 600, 0);
  //Serial.println(counter++);
//  int val = analogRead(A5);
 // Serial.println(distance);
//  delay(500);
//  stepper.setTarget(val);

  stepper.setTarget(q);

  stepper.update(currentTime);


}
