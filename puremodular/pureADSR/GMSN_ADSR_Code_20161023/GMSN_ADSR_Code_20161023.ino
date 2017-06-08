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
#include <EEPROM.h>

float enVal = 0;
int aPot,dPot,sPot, rPot;
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
double delta=1.6; // decar coeff
double rho=0.3;   //release coeff

int Time=0;
boolean SustainPhase=false;
int SustainLevel=0;
boolean finished=false;

struct coeffStruct {
  double alpha;
  double delta;
  double rho;
};

coeffStruct defaultcoeff ={
  0.6d,
  1.6d,
  0.3
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
  Serial.begin(9600); 
  coeffStruct coeff;
  EEPROM.get(eeAddress,coeff);  
  if (isnan(coeff.alpha) )
      EEPROM.put(eeAddress,defaultcoeff);
  Serial.println(coeff.alpha);
  boolean b=digitalRead(SW1);
   Serial.println(b);
   b=digitalRead(SW2);
   Serial.println(b);
}

int ReadPort(int Port){
 
  int value= 512;
   
  //if (debug==false)
   value=map(analogRead(Port), 0, 1024, 1024, 0);
   
   
  return value;
}
void getCoeff(){
if ((digitalRead(SW1) ==false) and (digitalRead(SW2) ==false) ){
      int value=map(analogRead(A3), 0, 1024, 1024, 0);
      alpha=pow((double)value/(double)512,2);
      value=map(analogRead(A2), 0, 1024, 1024, 0);
      delta=pow((double)value/(double)512,2);
      value=map(analogRead(A0), 0, 1024, 1024, 0);
      rho=pow((double)value/(double)512,2);
      if (debug==true){
         Serial.print("attack coeff");
         Serial.println(alpha);
      }
   }
}
int getAttack(int i){
  int Attackpot;
  if ((digitalRead(SW1) ==false) and (digitalRead(SW2) ==true) ){
      Attackpot=ReadPort(A3);
      aPot=Attackpot;
  }else{
    Attackpot=aPot;
  }
  double max=pow(Attackpot,alpha);
  double y=pow(i,alpha);
  double env=y/max;
  return 4095*env;
  
}

int getDecay(int i){
  int Decaypot;

  if ((digitalRead(SW1) ==false) and (digitalRead(SW2) ==true) ){
      Decaypot=ReadPort(A2);
      SustainLevel=4*ReadPort(A1);
      dPot=Decaypot;
      sPot=SustainLevel;
  }else{
    Decaypot=dPot;
    SustainLevel=sPot;
  }
  double max=pow(Decaypot,delta);
  double y=pow(i,delta);
  double env=y/max;
  int iEnv=4095-4095*env;
  
  if (iEnv <= SustainLevel){
      iEnv=SustainLevel;
      SustainPhase=true;
  }
  return iEnv;
  
}

int getRelease(int i){
  int Releasepot;

  if ((digitalRead(SW1) ==false) and (digitalRead(SW2) ==true) ){
      Releasepot=ReadPort(A0);
      rPot=Releasepot;
  }else{
    Releasepot=rPot;
  }
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
getCoeff();
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
  if((rising) and (digitalRead(GATEIN) == HIGH)){ // Inverted
    //The button was released before attack ended
    rising =false;
    Time=0;
    SustainPhase=false;
    SustainLevel=enVal; //Make it the same as the last attack value;
  }
  //Check if Gate is On and not rising.  In decay/sustain phase;
  if ((digitalRead(GATEIN) == LOW) and (rising==false)) { // Inverted
     
     if (SustainPhase==false){
       enVal=getDecay(Time);
       Time++;
     }
     else{
       Time=0;
       enVal=SustainLevel;
     }  
     mcpWrite((int)enVal);  
  }


  // If no Gate, write release values
 if (digitalRead(GATEIN) == HIGH) {
    SustainPhase=false;
    if (finished==false){
       enVal=getRelease(Time);
       Time++;
    }
    else 
    { 
      Time=0;
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
    if (value >0){
     Serial.print("Sustain Level ");
     Serial.print(SustainLevel); 
     Serial.print(" Time ");
     Serial.print(Time);
     Serial.print(" enval ");
     Serial.println(value);
     
    }
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
