/*
   SA - Share & Adapt.
   BY - Credit where credit is due.
   For any purpose including Commercial and Group Buys.
   No pressure to share design files.
   R&D funded by donation.
   https://gmsn.co.uk/products/r-d-funded-by-donations

   Creative Commons License
   Licensed under a Creative Commons Attribution 4.0 International License.
   http://creativecommons.org/licenses/by/4.0/

   Source from
   https://gmsn.co.uk/products/gmsn-pure-adsr

   original design and code from:
   https://gmsn.co.uk/

   This version by A.Cobley
   andy@r2-dvd.org

   Branch V1.0
*/


#include "SPI.h"
#include <EEPROM.h>


/*  Modes depend on the front multi way switch
     NORMAL behaves as an ADSR
     HOLD Sustain knob holds for a time regardless of Gate .  Hold button to change sustain level
     LOOP will loop !
*/

#define NORMAL 0
#define HOLD 1
#define LOOP 3
float enVal = 0;
int aPot, dPot, sPot, rPot;
boolean gate = 0, rising = false;
int buttonState, lastButtonState = HIGH, loopStage = 0, x = 0;
long lastDebounceTime = 0, debounceDelay = 500;
int TRIGGER = 2;
int BUTTONTRIGGER = 3;
int BUTTON = 4;
int GATEIN = 5;
int SW1 = 6;
int SW2 = 7;
int DACCS = 10;

boolean debug = false;
double alpha = 0.6; //attack coeff
double delta = 1.6; // decar coeff
double rho = 0.3; //release coeff

int Time = 0;
boolean SustainPhase = false;
int SustainLevel = 0;
boolean finished = false;
boolean decaying = false;
boolean ReleasePhase = false;
int SustainLength = 0; //Used for timed sustains.
bool TimedSustain = false;
int mode = NORMAL;


struct coeffStruct {
  double alpha;
  double delta;
  double rho;
  int SusLength;
};

coeffStruct defaultcoeff = {
  0.6d,
  1.6d,
  0.3,
  500
};

int eeAddress = 0;

void setup() {
  //DAC Comms
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  //Pins
  pinMode(DACCS, OUTPUT); //DAC CS
  pinMode(TRIGGER, INPUT); //TRIGGER IN  NOTE, this is inverted
  pinMode(BUTTONTRIGGER, INPUT); //BUTTON TRIGGER
  pinMode(BUTTON, INPUT); //Button
  pinMode(GATEIN, INPUT); //Gate In
  pinMode(SW1, INPUT); //MODE SW1
  pinMode(SW2, INPUT); //MODE SW2
  digitalWrite(DACCS, HIGH);
  //Interupts
  attachInterrupt(digitalPinToInterrupt(TRIGGER), gateOn, FALLING); //Actually on rising, the gate is inverted.
  attachInterrupt(digitalPinToInterrupt(BUTTONTRIGGER), SaveEEProm, RISING); //Actually on rising, the gate is inverted.
  coeffStruct coeff;
  EEPROM.get(eeAddress, coeff);
  if (isnan(coeff.alpha) )
    EEPROM.put(eeAddress, defaultcoeff);
  else {
    alpha = coeff.alpha;
    delta = coeff.delta;
    rho = coeff.rho;
    SustainLength=coeff.SusLength;
  }

}



void SaveEEProm() {
  //change this to use Update
  flash(2,100);
  coeffStruct coeff;
  coeff.alpha = alpha;
  coeff.delta = delta;
  coeff.rho = rho;
  coeff.SusLength=SustainLength;
  EEPROM.put(eeAddress, coeff);

}


int getMode() {
  if ((digitalRead(SW1) == false) and (digitalRead(SW2) == true) ) { //top Position
    mode = NORMAL;
    TimedSustain = false;
  }
  if ((digitalRead(SW1) == false) and (digitalRead(SW2) == false) ) { //MIddle Position
    mode = HOLD;
    TimedSustain = true;
  }
  if ((digitalRead(SW1) == true) and (digitalRead(SW2) == false) ) { //bottom Position
    mode = LOOP;
    TimedSustain = true;
  }
}

int ReadPort(int Port) {
  int value = 512;
  value = analogRead(Port);
  return value;
}

int Diff(int A, int B){
  int diff=A-B;
  return (abs(diff));
}

int OldAttackPot = -1;
int getAttack(int i) {
  int Attackpot;
  int readAttackpot = ReadPort(A3) + 1;
  if (Diff(readAttackpot,OldAttackPot)>5) {
    if (digitalRead(BUTTON) == true) {
      Attackpot = readAttackpot;
      OldAttackPot = readAttackpot;
    } else {
      int value = map(analogRead(A3), 0, 1024, 1024, 0);
      alpha = pow((double)value / (double)512, 2);
      Attackpot = OldAttackPot;
    }
  } else {
    Attackpot = OldAttackPot;
  }
  aPot = Attackpot;
  double max = pow(Attackpot, alpha);
  double y = pow(i, alpha);
  double env = y / max;
  return 4095 * env;

}

int OldDecayPot = -1;
int OldSustainLevel=-1;
int getDecay(int i) {
  int Decaypot;
  int readDecaypot = ReadPort(A2) + 1;
  if (TimedSustain == false) {
    SustainLevel = 4 * ReadPort(A1);
    sPot = SustainLevel;
  } else {
    
    if (digitalRead(BUTTON) == true) {
       SustainLength = 4 * ReadPort(A1);
       SustainLevel = sPot;
    }else{
       SustainLevel = 4 * ReadPort(A1);
       sPot = SustainLevel;
    }
  }
  if (Diff(readDecaypot,OldDecayPot)>5) {

    if (digitalRead(BUTTON) == true) {
      Decaypot = readDecaypot;
      OldDecayPot = readDecaypot;

    } else {
      int value = map(analogRead(A2), 0, 1024, 1024, 0);
      delta = pow((double)value / (double)512, 2);
      Decaypot = OldDecayPot;
      SustainLevel = sPot;
    }
  } else {
    Decaypot = OldDecayPot;

  }
  dPot = Decaypot;
  double max = pow(Decaypot, delta);
  double y = pow(i, delta);
  double env = y / max;
  int iEnv = 4095 - 4095 * env;

  if (iEnv <= SustainLevel) {
    iEnv = SustainLevel;
    SustainPhase = true;
  }
  return iEnv;

}
int OldReleasePot = -1;
int getRelease(int i) {

  int Releasepot;
  int readReleasepot = ReadPort(A0) + 1;
  if (Diff(readReleasepot,OldReleasePot)>5) {
    if (digitalRead(BUTTON) == true) {
      Releasepot = readReleasepot;
      OldReleasePot = readReleasepot;
    } else {
      int value = map(analogRead(A0), 0, 1024, 1024, 0);
      rho = pow((double)value / (double)512, 2);
      Releasepot = OldReleasePot;
    }
  } else {
    Releasepot = OldReleasePot;
  }
  dPot = Releasepot;
  double max = pow(Releasepot, rho);
  double y = pow(i, rho);
  double env = y / max;
  int iEnv = SustainLevel - 4095 * env;

  if (iEnv <= 0) {
    iEnv = 0;
    finished = true;

  }
  return iEnv;

}




void loop() {
  getMode();

  boolean GateIn = !digitalRead(GATEIN);
  if ((rising) and
      ((GateIn == HIGH) || (TimedSustain == true))
     ) {

    enVal = getAttack(Time);
    Time++;
    if (enVal >= 4095) {
      enVal = 4095;
      rising = false;
      Time = 0;
    }
    mcpWrite((int)enVal);
  }
  if ((rising) and
      ((GateIn == LOW) and  (TimedSustain == false))
     ) {
    //The gate was released before attack ended
    rising = false;
    Time = 0;
    SustainPhase = false;
    SustainLevel = enVal; //Make it the same as the last attack value;
  }
  //Check if Gate is On and not rising.  In decay/sustain phase;
  if (((GateIn == HIGH) || (TimedSustain == true))
      and (rising == false) and (ReleasePhase == false) and (finished == false)) {

    if (SustainPhase == false) {
      enVal = getDecay(Time);
      Time++;
      decaying = true;
      SustainLevel = enVal; // In case gate goes off before end of decay, release should start at current value
    }
    else {
      if (TimedSustain == false) {
        Time = 0;
      } else {
        Time ++;
      }
      enVal = SustainLevel;
      decaying = false;
    }
    mcpWrite((int)enVal);
  }

  if ((GateIn == LOW) && (TimedSustain == false)) {
    ReleasePhase = true;
    SustainPhase = false;

  }
  if ((TimedSustain == true) and (SustainPhase == true)) {
    if (Time > SustainLength) {
      Time = 0;
      ReleasePhase = true;
      SustainPhase = false;
    }
  }

  // If no Gate, write release values
  if (ReleasePhase == true) {
    if (decaying == true) {
      Time = 0;
      decaying = false;

    }
    SustainPhase = false;
    if (finished == false) {
      enVal = getRelease(Time);
      Time++;
    }
    else
    {
      Time = 0;
      ReleasePhase = false;
      if (mode==LOOP){
        gateOn();
      }
    }
    mcpWrite((int)enVal);
  }
  //delayMicroseconds(590);
}


//Interrupt routine for rising edge of Gate
void gateOn() {
  //flash (1, 200);
  enVal = 1;
  rising = true;
  Time = 0;
  SustainPhase = false;
  finished = false;
  ReleasePhase = false;

}

//Function for writing value to DAC. 0 = Off 4095 = Full on.

void mcpWrite(int value) {
  //set top 4 bits of value integer to data variable
  byte data = value >> 8;
  data = data & B00001111;
  data = data | B00110000;
  cli();
  digitalWrite(DACCS, LOW);
  SPI.transfer(data);
  data = value;
  SPI.transfer(data);
  digitalWrite(DACCS, HIGH);
  sei();
}

//Test function for flashing the led. Value = no of flashes, time = time between flashes in mS
void flash(int value, int time) {
  int x = 0;
  cli();
  while (x < value) {
    mcpWrite(4000);
    delay(time);
    mcpWrite(0);
    delay(200);
    x++;
  }
  sei();
}
