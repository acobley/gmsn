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
   * 
 * Changed Attack calculation
*/


#include "SPI.h"
#include <SoftwareSerial.h>

float aPot, aCoeff, enVal = 0, dPot, dCoeff, sPot, sCoeff, sVal, rPot, rCoeff;
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

boolean debug = true;
double alpha=0.6; //attack coeff
double delta=0.6; // decar coeff
double rho=0.6;   //release coeff

int Time=0;
boolean SustainPhase=false;
int SustainLevel=0;
boolean finished=false;

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
  Serial.begin(9600);  
    
 
}

int ReadPort(int Port){
  int value= 128;
  //if (debug==false)
     value=map(analogRead(Port), 0, 1024, 1024, 0);
  return value;
}


int getAttack(int i){
  int Attackpot=ReadPort(A3);
  double max=pow(Attackpot,alpha);
  double y=pow(i,alpha);
  double env=y/max;
  return 4095*env;
  
}

int getDecay(int i){
  int Decaypot=ReadPort(A2);
  double max=pow(Decaypot,delta);
  double y=pow(i,delta);
  double env=y/max;
  int iEnv=4095-4095*env;
  SustainLevel=4*ReadPort(A1);
  if (iEnv <= SustainLevel){
      iEnv=SustainLevel;
      SustainPhase=true;
  }
  return iEnv;
  
}

int getRelease(int i){
  int Releasepot=ReadPort(A0);
  double max=pow(Releasepot,rho);
  double y=pow(i,rho);
  double env=y/max;
  int iEnv=SustainLevel-4095*env;
  
  if (iEnv <= 0){
      iEnv=0;
      finished=true;

  }
  return iEnv;
  
}




void loop() {

  if ((rising) and (digitalRead(GATEIN) == LOW)){ // Inverted

    enVal=getAttack(Time);
    Time++;
    if (enVal >= 4095) {
      enVal = 4095;
      rising =false;
      Time=0;
    }
     mcpWrite((int)enVal);
  }

  //Check if Gate is On and not rising.  In decay/sustain phase;
  if ((digitalRead(GATEIN) == LOW) and (rising==false)) { // Inverted
     enVal=getDecay(Time);
     if (SustainPhase==false)
       Time++;
     mcpWrite((int)enVal);  
  }

  // If no Gate, write release values
 if (digitalRead(GATEIN) == HIGH) {
    SustainPhase=false;
    if (finished==false){
       enVal=getRelease(Time);
       Time++;
    }
    mcpWrite((int)enVal);
  }
  delayMicroseconds(590);
}


//Interrupt routine for rising edge of Gate
void gateOn() {
  //flash (1, 200);
  enVal = 1;
  rising = true;
  Time=0;
  SustainPhase=false;
  finished=false;
     Serial.print("GateOn ");
     Serial.println(rising);
}

//Function for writing value to DAC. 0 = Off 4095 = Full on.

void mcpWrite(int value) {
  if(debug==false){
  //CS
  digitalWrite(DACCS, LOW);

  //DAC1 write

  //set top 4 bits of value integer to data variable
  byte data = value >> 8;
  data = data & B00001111;
  data = data | B00110000;
  SPI.transfer(data);

  data = value;
  SPI.transfer(data);

  // Set digital pin DACCS HIGH
  digitalWrite(DACCS, HIGH);
  }else{
     Serial.print("Time ");
     Serial.print(Time);
     Serial.print(" enval ");
     Serial.println(value);
  }
}

//Test function for flashing the led. Value = no of flashes, time = time between flashes in mS
void flash(int value, int time) {
  int x = 0;
  while (x < value) {
    mcpWrite(4000);
    delay(time);
    mcpWrite(0);
    delay(200);
    x++;
  }
}
