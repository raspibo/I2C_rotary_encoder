#include <Wire.h>
#include "Adafruit_MCP23017.h"  // https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library

// setup the port expander
Adafruit_MCP23017 mcp0;

int butPress = 101;          // stores which button has been pressed
int encSelect[2] = {101, 0}; // stores the last encoder used and direction {encNo, 1=CW or 2=CCW}
unsigned long currentTime;   //used as debounce timer
unsigned long loopTime;

volatile boolean awakenByInterrupt = false;

// Interrupts from the MCP will be handled by this PIN on Arduino
byte arduinoIntPin = 3;
// ... and this interrupt vector
byte arduinoInterrupt = 1;

int led = 13;

int int_num = 0;

/*const int encCount0 = 2;  // number of rotary encoders
  // encoder pin connections to MCP23017
  //    EncNo { Encoder pinA  GPAx, Encoder pinB  GPAy },
  const int encPins0[encCount0][2] = {
  {0,1},   // enc:0 AA GPA0,GPA1 - pins 21/22 on MCP23017
  {3,4}    // enc:1 BB GPA3,GPA4 - pins 24/25 on MCP23017
  };
*/

const int encCount0 = 1;  // number of rotary encoders
const int encPins0[encCount0][2] = {
  {0, 1}  // enc:0 AA GPA0,GPA1 - pins 21/22 on MCP23017
};

/*const int butCount0 = 2;  // number of buttons
  //             button on encoder: A  B
  const int butPins0[butCount0] = { 2, 5 };
  // arrays to store the previous value of the encoders and buttons
*/
const int butCount0 = 1;  // number of buttons
//             button on encoder: A  = GPIO A2
const int butPins0[butCount0] = { 2 };

unsigned char encoders0[encCount0];
unsigned char buttons0[butCount0];

// read the rotary encoder on pins X and Y, output saved in encSelect[encNo, direct]
unsigned char readEnc(Adafruit_MCP23017 mcpX, const int *pin, unsigned char prev, int encNo) {

  unsigned char encA = mcpX.digitalRead(pin[0]);    // Read encoder pins
  unsigned char encB = mcpX.digitalRead(pin[1]);

  if ((!encA) && (prev)) {
    encSelect[0] = encNo;
    if (encB) {
      encSelect[1] = 1;  // clockwise
    }
    else {
      encSelect[1] = 2;  // counter-clockwise
    }
  }
  return encA;
}

// read the button on pin N. Change saved in butPress
unsigned char readBut(Adafruit_MCP23017 mcpX, const int pin, unsigned char prev, int encNo) {

  unsigned char butA = mcpX.digitalRead(pin);    // Read encoder pins
  if (butA != prev) {
    if (butA == HIGH) {
      butPress = encNo;
    }
  }
  return butA;
}

// setup the encoders as inputs.
unsigned char encPinsSetup(Adafruit_MCP23017 mcpX, const int *pin) {

  mcpX.pinMode(pin[0], INPUT);  // A
  mcpX.pullUp(pin[0], HIGH);    // turn on a 100K pullup internally
  mcpX.setupInterruptPin(pin[0], CHANGE); //Enable interrupt handling for this pin suggested mode CHANGE - then FALLING mode cause continuos interrputs if encoder stop on intermediate position, RISING emit continuos interrupt (not usable)
  mcpX.pinMode(pin[1], INPUT);  // B
  mcpX.pullUp(pin[1], HIGH);
  mcpX.setupInterruptPin(pin[1], CHANGE);
}

// setup the push buttons
void butPinsSetup(Adafruit_MCP23017 mcpX, const int pin) {
  mcpX.pinMode(pin, INPUT);
  mcpX.pullUp(pin, HIGH);
  mcpX.setupInterruptPin(pin, CHANGE); //With CHANGE an interrupt is generate when botton status change, FALLING cause continuos interrupt if button is pressed
}

void setup() {
  pinMode(arduinoIntPin, INPUT);

  mcp0.begin(0);    // 0 = i2c address 0x20
  for (int count = 0; count < 16; count++) {
    mcp0.pinMode(count, OUTPUT);
  }

  // setup the pins using loops, saves coding when you have a lot of encoders and buttons
  for (int n = 0; n < encCount0; n++) {
    encPinsSetup(mcp0, encPins0[n]);
    encoders0[n] = 1;  // default state
  }

  // buttons and encoders are in separate arrays to allow for additional buttons
  for (int n = 0; n < butCount0; n++) {
    butPinsSetup(mcp0, butPins0[n]);
    buttons0[n]  = 0;
  }

  Serial.begin(115200);
  Serial.println("---- Startup ----------");

  currentTime = millis();
  loopTime = currentTime;
  mcp0.setupInterrupts(true, false, LOW);


  // enable interrupts before going to sleep/wait
  // And we setup a callback for the arduino INT handler.
  attachInterrupt(arduinoInterrupt, intCallBack, FALLING);
}

void intCallBack() {
  awakenByInterrupt = true;
}

void handleInterrupt() {
  // disable interrupts while handling them.
  detachInterrupt(arduinoInterrupt);

  mcp0.getLastInterruptPin();
  mcp0.getLastInterruptPinValue();

  digitalWrite(led, !digitalRead(led));
  // check the encoders and buttons every 1 millis for debounce
  currentTime = millis();
  if (currentTime >= (loopTime + 1)) {

    for (int n = 0; n < encCount0; n++) {
      encoders0[n] = readEnc(mcp0, encPins0[n], encoders0[n], n);
    }

    for (int n = 0; n < butCount0 ; n++) {
      buttons0[n] = readBut(mcp0, butPins0[n], buttons0[n], n);
    }

    loopTime = currentTime;  // Updates loopTime
  }

  // when an encoder has been rotated

  if (encSelect[0] < 100) {
    Serial.print("Enc: ");
    Serial.print(encSelect[0]);
    Serial.print(" " );

    switch (encSelect[1]) {
      case (1): // clockwise
        Serial.println("CW  ");
        break;
      case (2): // counter-clockwise
        Serial.println("CCW  ");
        break;
    }

    // do something when a particular encoder has been rotated.
    if (encSelect[0] == 1) {
      Serial.println("Encoder One has been used");
    }

    // set the selection to 101 now we have finished doing things. Not 0 as there is an encoder 0.
    encSelect[0] = 101;
  }
  // ready for the next change

  // do things when a when a button has been pressed
  if (butPress < 100) {
    Serial.print("But: ");
    Serial.println(butPress);

    butPress = 101;
  }
  cleanInterrupts();
  //we set callback for the arduino INT handler.
  attachInterrupt(arduinoInterrupt, intCallBack, FALLING);
}

void cleanInterrupts() {
  EIFR = 0x01;
  awakenByInterrupt = false;
}


void loop() {
  if (awakenByInterrupt) handleInterrupt();

  delay(50);                                //Maximum delay acceptable for normal and reactive encoder response time without loose events
}
