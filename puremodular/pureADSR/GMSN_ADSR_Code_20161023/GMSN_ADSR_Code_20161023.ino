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

boolean debug = false;
float alpha=0.25;
float attackPower=(alpha-1)/alpha;
float delta=0.25;
float decayPower=(delta-1)/delta;
float release=0.25;
float releasePower=(delta-1)/release;
float dx=1/1024;

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
  //  Say we are alive by flashing the LED
  flash (10, 500);
}

int ReadPort(int Port){
  int value= 512;
  if (debug=false)
     map(analogRead(Port), 0, 1024, 1024, 0);
  return value;
}
void GetPots(){
  aPot = ReadPort(A3);
  dx=aPot/1024;
  aCoeff = alpha*pow(enVal/4100,attackPower)*dx;
  dPot = ReadPort(A2);
  dx=dPot/1024;
  dCoeff = alpha*pow(enVal/4100,decayPower)*dx;
  sPot = map(analogRead(A1), 0, 1024, 0, 4096);
  rPot = ReadPort(A0);
  dx=rPot/1024;
  dCoeff = alpha*pow(enVal/4100,releasePower)*dx;
}

void loop() {
  GetPots();
  
  if (rising) {

    enVal+=aCoeff * 4100;
    if (enVal > 4095) {
      enVal = 4095;
    }
  }

  //Check if Gate is On
  if (digitalRead(GATEIN) == LOW) { // Inverted

    //Attack
    if (rising) {
      if (enVal >= 4094) {
        rising = false;
      }
    } if (enVal <=sPot){  //  Sustain
        mcpWrite((int)enVal);
    }
    else {
      //else continue with decay to sustain
      enVal += dCoeff * (-4100);
      mcpWrite((int)enVal);
    }
  }

  // If no Gate, write release values
  else {

    enVal += rCoeff * (-4100);
    if (enVal < 0) {
      enVal = 0;
    }
    mcpWrite((int)enVal);
  }
  delayMicroseconds(590);
}


//Interrupt routine for rising edge of Gate
void gateOn() {
  //flash (1, 200);
  enVal = 0;
  rising = true;

}

//Function for writing value to DAC. 0 = Off 4095 = Full on.

void mcpWrite(int value) {

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
