
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
  StepperDriver(uint8_t stepPin, uint8_t dirPin, uint16_t stepSpeed=400):
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

    if (abs(mCurrentPosition - mTargetPosition) > 10) {
    
      //uint16_t timeDiff = microsTime - mLastUpdate;
      //if (timeDiff > mStepTime) {
      if (tick(microsTime)) {
        digitalWrite(mStepPin, mStep);
        mStep ^= 1;
        mCurrentPosition+= mDirection ? 1 : -1;

        //mLastUpdate = microsTime;
      }
    }
    
  }
  
protected:
  uint8_t mStepPin, mDirectionPin;
  int16_t mCurrentPosition, mTargetPosition;
  int16_t mStartPosition, mEndPosition;
//  uint32_t mLastUpdate; // micros
  uint16_t mStepTime;

  uint8_t mStep, mDirection;
  
};


struct UltrasonicReader : protected TimedAction  {
  UltrasonicReader(uint8_t echoPin, uint8_t triggerPin, uint32_t updateTime=10000):
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

//        Serial.println(mDistance);
      }

      return mDistance;
  }

  uint8_t mEchoPin, mTriggerPin;
  uint16_t mDistance;
};


StepperDriver stepper(7, 8);
UltrasonicReader distanceReader(4, 5);

uint32_t currentTime;



void setup() {
  Serial.begin(9600);
  //stepper.setTarget(400);  

  pinMode(A5, INPUT);
}

void loop() {

  currentTime = micros();

  uint16_t distance = distanceReader.update(currentTime);

//  int val = analogRead(A5);
//  Serial.println(val);
//  delay(500);
//  stepper.setTarget(val);

  stepper.setTarget(distance*10);

  stepper.update(currentTime);


}
