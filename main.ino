#include <SdFat.h>
#include <MD_MIDIFile.h>
#include <string.h>

#define pwmPin 37 // DC velocity
#define encPin 20 // Encoder 
#define inPin1 35 // DC dir1
#define inPin2 33 // DC dir2
#define pedalPin 2 // Pedal
#define soundPin 13 // Sound sensor

#define pMem 3 // Number of pedal press times saved

SDFAT SD;
MD_MIDIFile SMF;
File instrf;

// ------------------------------------
int n = 0;

long t = 0; // Time value in milliseconds
const int n_drums = 3; // Number of drums
bool isDrumming = true; // System is drumming

unsigned long lastDetectedHitTime = 0;
unsigned long lastMidiHitTime = 0;

/* Last BPM change time {RealTime, NormalizedEventTime}
   Real time is the time of the change in milliseconds.
   Normalized event time is the corresponding time for BPM=1 */
unsigned long bpmChangeT[2] = {0,0}; 

/* As the solenoids moves along the rail this array keeps track
   of which solenoids are in place for their next drumming.
*/
bool solArrived[n_drums];
int targetPos;
int encPos = 0; // Encoder position
// ------------------------------------
// Pedal
bool pedalPressed = false;
unsigned int pressTime[pMem]; // Pedal press times (ms)
unsigned int detectedTime[pMem];
bool changedBPM = false;

/* Pedal press difference threshhold. If this time (ms) has passed
   between pedal presses, no bpm change will occur.
   That way, when the user stops pressing the pedal the bpm will stay constant,
   and when they start pressing again the long gap between presses won't result in
   an unintended slow BPM.
*/
int pedalThresh = 4000; 

unsigned int prevPot = 0;
unsigned long startT = 0;

float bpm = 100;
float readBPM = 200;
// ------------------------------------

/* Contains the closest drumming midi event.
   Format is {drum id, event time, velocity, position on drum}*/
int event[4];

/* Note number values.
   In most midi files, different drums in a drum kit are represented by
   different numbers, which usually determine the pitch but with drums they determine which drum to use. 
   They are stored in this array.
   These notes can be translated to the corresponding drum in our system using note2id().
   (Midi protocol uses Middle C = 60. Each half step is an increment/decrement of 1)  */
short notes[3];

short nn = 0; // Number of existing notes (to keep track of how many note number values have been saved in notes so far)
const byte solPins[] = {8,9,1}; // Solenoid presses

bool arrived = false;
int dir = 1;
int e = 0;

// ------------------------------------
// Instruction calculations

// Calculate time (ms) needed to move a solenoid to its next position
int pos2msdelay(int pos, int id) {
  return 0; // Calculate time correction based on pos and id
}

// Convert velocity to position
int vel2pos(int vel, int id) {
  return (int) (vel/20.0); // Calculate position
}

/* Convert midi note value to drum id
  If a note does not exist and there is still an available drum, the note will be added to notes[] and assigned a drum*/
int note2id(int n) {
  for (int i=0; i<nn; i++) {
    if (n==notes[i]) {return i;}
  }
  if (nn == n_drums) {return -1;}
  notes[nn] = n;
  return nn++;
}

// ------------------------------------
// Midi reading

/* Handles single midi event - adds event to instruction file.
   Calculates pos, and corrects timing using pos2msdelay(), vel2pos(). 
   Linked to the midi file event listener. */
void midiCallback(midi_event *pev) {
  midi_event e = *pev;
  switch(e.data[0]) {
    case 144: // Note on
      int id = note2id(e.data[1]);
      int pos = vel2pos(e.data[2], id);
      int timecorrection = pos2msdelay(pos, id);
      if (id != -1) {
        instrf.println(millis() - timecorrection);
        instrf.println(id); // id
        instrf.println(e.data[2]); // vel
        instrf.println(pos); // pos
      }
      break;
  }
}

/* Convert midi to text instructions.
   Takes a midi file with name fname, and outputs instructions to a new file named fout. */
void wMidiToFile(char* fname, char* fout) {
  if (SD.exists(fout)) {return;SD.remove(fout);}
  instrf = SD.open(fout, FILE_WRITE);
  // File loaded succesfuly
  if (SMF.load(fname) == MD_MIDIFile::E_OK && instrf) {
    // Loop over file
    SMF.setTempo(readBPM);
    while (!SMF.isEOF()) { 
      SMF.getNextEvent();
    }
    SMF.close();
    instrf.close();
    Serial.println("Done creating instructions.");
  } else {
    Serial.println("Failed to load files.");
  }
}

// ------------------------------------
// Drumming

// Opens instructions file and reads first event
void initDrumFile(char* fname) {
  instrf = SD.open(fname, FILE_READ);
  startT = millis();
  for (int i=0; i<4; i++) {event[i] = next(instrf);}
}

/* Handles the entirity of the drumming logic.
   Calculates event time based on the normalized event time.
   When an event's (pre-adjusted) time has just passed:.
   Calls moveSolenoid() to move solenoids to place.
   Calls activateSolenoid() to activate the solenoid once the solenoid has reached its place.
   Read next event. */
void executeEvents() {
  // event: (time, id, vel pos)
  if (event[0]+event[1]+event[2]+event[3] == 0) {Serial.println("Disk missing."); isDrumming=false;}
  if (event[0] == -1) {Serial.println("Done playing file."); isDrumming=false;}  
  long event_time = (event[0]*readBPM-bpmChangeT[1])/bpm + bpmChangeT[0];
  if (t > event_time || t > 4000000000) {
    // activateSolenoid(event[1], event[2]);
    // lastMidiHitTime = event[0]*readBPM;
    // for (int i=0; i<4; i++) {event[i] = next(instrf);}
    // changedBPM = false;
    // n++;
    solArrived[event[1]] = moveSolenoid(event[1], event[3]);
    if (solArrived[event[1]]) {
      activateSolenoid(event[1], event[2]);
      lastMidiHitTime = event[0];
      for (int i=0; i<4; i++) {event[i] = next(instrf);}
      solArrived[event[1]] = false;
      changedBPM = false;
      arrived = false;
      n++;
    }
  }  
}

// Read next entry of the instructions file f
float next(File& f) {
  if (f.available()) {
    return f.readStringUntil('\n').toFloat();;
  } return -1;
}

// ------------------------------------
// Pedal

// Sets the bpm to newBPM. Updates bpmChangeT.
void setBPM(float newBPM) {
  bpmChangeT[1] += (t-bpmChangeT[0])*bpm;
  bpmChangeT[0] = t;
  bpm = newBPM;  
  changedBPM = true;
}

// Handles any bpm changes (pedal and potentiometer)
void handleBpmChange() {
  float newBPM = handlePedal();
  if (newBPM != -1) {
    setBPM(newBPM);
    return;
  }
  newBPM = analogRead(A0)/4;
  if (abs(newBPM-prevPot)>20) {
    setBPM(newBPM);
    prevPot = newBPM;
  }
}

/* Calculate pedal BPM based on pMem pedal presses.
   Called when the pedal is pressed.
   BPM is calculated using an average of the time differences between presses.
   T[pMem] is the time of pedal presses in milliseconds.
*/
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

void fixBPM() {
  if (!changedBPM) {
    float detectedBPM = (event[0]-lastMidiHitTime)/(millis()-lastDetectedHitTime);
    float delta = detectedBPM - bpm;
    setBPM(bpm - .2*delta);
  }
  lastDetectedHitTime = millis();
  changedBPM = false;
}

// Handles pedal presses
float handlePedal() {
  if (digitalRead(pedalPin)) {
    if (!pedalPressed) {
      if (t - pressTime[pMem-1] < 500) {return -1;}
      pedalPressed = true;
      float newBPM = calculateBPM(pressTime);
      return newBPM;
    }
  } else {
    pedalPressed = false;
  }
  return -1;
}


// ------------------------------------


// Activate solenoid
void activateSolenoid(int id, int vel) {
  digitalWrite(solPins[id], LOW);
  delay(50);
  digitalWrite(solPins[id], HIGH);
}

// Move solenoid and return if has arrived at its position 
bool moveSolenoid(int id, int targetPos_) {
  targetPos = targetPos_;
  e = targetPos - encPos;
  arrived = (abs(e) < 2);
  if (!arrived) {
    dir = -1;
    e = targetPos - encPos;
    if (e < 0) {dir = 1;}
    setMotor(dir, 130);
  }
  return arrived;
}

void readEncoder() {
  encPos -= dir;
  e = targetPos - encPos;
  arrived = (abs(e) < 2);
  if (arrived) { 
    encPos -= dir; // Slowdown error fix
    setMotor(0, 0); 
    dir=0;
  }
}

void setMotor(int dir, int pwmVal) {
  digitalWrite(inPin1, dir==1);
  digitalWrite(inPin2, dir==-1);
  analogWrite(pwmPin,pwmVal);
}

// ------------------------------------
  
// Setup pin modes
void setupPins() {
  for (int i=0; i<n_drums; i++) {
    pinMode(solPins[i], OUTPUT);
  }
  pinMode(pedalPin, INPUT);
  pinMode(encPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(inPin1, OUTPUT);
  pinMode(inPin2, OUTPUT);
}

// Create instruction file for midi file and start drumming accordingly
void start() {
  wMidiToFile("demo.mid", "instrf.txt");
  initDrumFile("instrf.txt");

  encPos = 0;
  attachInterrupt(digitalPinToInterrupt(encPin),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(soundPin),fixBPM,RISING);

  prevPot = analogRead(A0)/4;
  bpm = prevPot;
  isDrumming = true;
}

void setup() {
  Serial.begin(9600); 
  setupPins();
  if (!SD.begin(4,SPI_FULL_SPEED)) {
    Serial.println("Failed to open SD card."); 
    while (true);
  }
  SMF.begin(&SD); // Initialize
  SMF.setMidiHandler(midiCallback);
  for (int i=0; i<n_drums; i++) {
    digitalWrite(solPins[i],HIGH);
    solArrived[i] = true;
  }
  start();
}

void loop() {
  if (isDrumming) {
    t = millis()-startT;
    handleBpmChange();
    executeEvents();
  }
}

