
#define pedalPin 2
#define solPin 8

#define pMem 3

long t = 0; // Time value in milliseconds

bool pedalPressed = false;

float bpm = 100;
unsigned long bpmChangeT[2] = {0,0}; 
bool changedBPM = false;

int prevPot = 0;

int n = 1;

float pedalThresh = 4000;
unsigned int pressTime[pMem];

float lastHit = 0;
float hitTime = 40;

float calculateBPM(int T[pMem]) {
  // Sum of time differences between each 2 consecutive presses
  float diffsum = 0;
  // Shift values back to add new value
  for (int i = 0; i<pMem-1; i++) {
    diffsum += T[i+1] - T[i];
    T[i] = T[i+1];
  }
  T[pMem-1] = millis();
  int lastdiff = T[pMem-1]-T[pMem-2];
  diffsum += lastdiff;
  // Calculate BPM based on bpm = (ms in minute) / (avg time step)
  if (T[0] != 0 && diffsum < pedalThresh) {
    float bpm_ = 60000.0 / (diffsum/pMem);
    return bpm_;
  } 
  return -1;
}

// Handles pedal presses
float handlePedal() {
  if (digitalRead(pedalPin)) {
    if (!pedalPressed) {
      if (t - pressTime[pMem-1] < 500) {return -1;}
      pedalPressed = true;
      float newBPM = calculateBPM(pressTime);
      Serial.println(newBPM);
      return newBPM;
    }
  } else {
    pedalPressed = false;
  }
  return -1;
}

void setBPM(float newBPM) {
  bpmChangeT[1] += (t-bpmChangeT[0])*bpm;
  bpmChangeT[0] = t;
  bpm = newBPM;  
  changedBPM = true;
}

void handleBpmChange() {
  float newBPM = handlePedal();
  if (newBPM != -1) {
    setBPM(newBPM);
    return;
  }
  // newBPM = analogRead(A0)/4;
  // if (abs(newBPM-prevPot)>20) {
  //   setBPM(newBPM);
  //   prevPot = newBPM;
  // }
}

void setup() {
  Serial.begin(9600);
  pinMode(pedalPin, INPUT);
  pinMode(solPin, OUTPUT);
}

void loop() {
  handleBpmChange();
  t = millis();
  long event_time = n*60000.0/bpm - bpmChangeT[1]/bpm + bpmChangeT[0];
  if (t > event_time) {
    digitalWrite(solPin, LOW);
    lastHit = t;
    n++;
  } else if (t - lastHit >= hitTime) {
    digitalWrite(solPin, HIGH);
  }
}
