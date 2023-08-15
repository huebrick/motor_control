/////////////////////////////////   (LIBRARY)  ///////////////////////////////////

#include <movingAvg.h>
#include <neotimer.h>

/////////////////////////////////   (INPUT)  ///////////////////////////////////

#define stop 2
#define limit 3
#define up 5
#define down 4
#define in1 6
#define in2 7
#define cur_mes A3
#define HV A2

/////////////////////////////////   (OUTPUT)  //////////////////////////////////

/* -->IT IS VERY IMPORTANT THAT MOTOR MOVES IN THE EXPECTED DIERECTION, WHEN YOUR PUSH THE UP BUTTON IT MUST MOVE UP AND VICE VERSA!<--
If motor movement is inverted switch out these pins, example:
#define up_rel 9
#define down_rel 8

Or swtich out the motor wireing :)  */

#define up_rel 8
#define down_rel 9

/////////////////////////////////   (DEBUG)  ///////////////////////////////////

#define cur_fault A1
#define cap_fault A0
#define up_debug 11
#define down_debug 12

/////////////////////////////////   (VAR)  ///////////////////////////////////

int cur_raw;
int cur_avg;
bool fault = false;
bool up_dbg = false;
bool down_dbg = false;
bool fault_dbg = false;
bool cap_dbg = false;
bool cur_dbg = false;
bool surge = false;
bool surge_state = false;
bool shutdown = false;


bool info = false;    //Send out debug information over serial (if true, set cdebug to false)
bool cdebug = false;  //Send out current reading every 100ms, useful for calibration and can be used with a serial plotter (if true, set info to false)
bool runaway = true;  //When the cap sensor is triggered the motor moves in the oppsoite direction
bool OCPR = true;     //Over Current Position Reset - Motor will move to the highest position when over current event is triggered
bool OVP = true;      //Over Voltage Protection shuts down the controller if the input voltage reaches over the maximum allowed voltage


#define surgetimer 20  //This value sets how many miliseconds of current measurements should be rejected at the moment of motor starting to avoid false OC triggers. This value should be AS SMALL AS POSSIBLE 
/*
Given that the installed sensor can measure currents flowing both ways its resting point(no current flowing) is centerd around the half point of its power supply. In this case that is 1023 / 2. When you define 
current diviation you are defing a current value that can flow in both directions. For example lets say our motor draws 100 above the baseline (515) in its normal operation,we could 
specify our UCD and DCD at around 120 so the protection will only trigger at 620 or 380. MCD is the value at which the controller will shut down and only start again after a power reset, this value is a current that should
never be crossed and is best set at the limits of the measuring IC so around 400-500.

State is the input voltage to the motor, this board can measure up to 50V but the controller maximum is set at 24V. The program will move into a error state if it encounters overvoltage(input voltage higher than state 1) so
so it is best to set state1 at aroudn 25V so it dosent trigger at lower voltages.
*/
int UCD; //UP current diviation (500 +- UCD) {max value - 500}
int DCD; //DOWN current diviation (500 +- DCD) {max value - 500}
int MCD; //MAXIMUM current diviation (500 +- MCD) {max value - 500}
int MID = 515; //resting point of the current sensor(no current flowing).
int VR;  //Input voltage expressed in volts
int MV = 25; //Maximum allowed voltage
//CURRENT DIVIATION AT STATE 1 (MAX VOLTAGE)
#define state1 25
#define UCD1 300 
#define DCD1 100 
#define MCD1 500 

//CURRENT DIVIATION AT STATE 2
#define state2 20 
#define UCD2 300 
#define DCD2 100 
#define MCD2 500 

//CURRENT DIVIATION AT STATE 3
#define state3 14 
#define UCD3 300 
#define DCD3 500 
#define MCD3 500 

//CURRENT DIVIATION AT STATE 4 (MIN VOLTAGE)
#define state4 8 
#define UCD4 100 
#define DCD4 100 
#define MCD4 500 

/////////////////////////////////   (OTHER)  ///////////////////////////////////

movingAvg average(50); //This value sets how many samples does the moving average take into the calculation

Neotimer surgefilter = Neotimer(surgetimer);
Neotimer t1000;
Neotimer t1;
Neotimer t100;
/////////////////////////////////   (END)  ///////////////////////////////////

void setup() {

  /////////////////////////////////   (INPUT)  ///////////////////////////////////

  pinMode(stop, INPUT_PULLUP);
  pinMode(up, INPUT_PULLUP);
  pinMode(down, INPUT_PULLUP);
  pinMode(in1, INPUT_PULLUP);
  pinMode(in2, INPUT_PULLUP);
  pinMode(cur_mes, INPUT);
  pinMode(HV, INPUT);
  pinMode(limit, INPUT_PULLUP);

  /////////////////////////////////   (OUTPUT)  ///////////////////////////////////

  pinMode(cur_fault, OUTPUT);
  pinMode(cap_fault, OUTPUT);
  pinMode(up_rel, OUTPUT);
  pinMode(down_rel, OUTPUT);
  pinMode(up_debug, OUTPUT);
  pinMode(down_debug, OUTPUT);

  /////////////////////////////////   (OTHER)  ///////////////////////////////////

if (cdebug == true || info == true ){  //starts serial if its needed
  Serial.begin(9600);
}

  attachInterrupt(digitalPinToInterrupt(stop), cap_trig, LOW); //Sets up two most important imputs as interrupts.
  attachInterrupt(digitalPinToInterrupt(limit), lmt, LOW);

  average.begin(); //starts the moving average function

  digitalWrite(up_rel, LOW); //Sets both relays to off as a start condition
  digitalWrite(down_rel, LOW);

  t1000.set(1000); //Settings for different timers. Most fucnctions dont need to run at loop speed so they are seperated in different timings.
  t1.set(1);
  t100.set(100);

}

void loop() {

if (shutdown == true){ //This sets the code in a loop preventing it from running anything else
  cur_dbg = true; down_dbg = true; up_dbg = true; cap_dbg = true;
  debug();
  delay(500);
  cur_dbg = false; down_dbg = false; up_dbg = false; cap_dbg = false;
  debug();
  delay(500);
  return;
}

if (info == true) {
  if (t1000.repeat()) { SerailDebug();} //This function is called every 1000ms
} 
if (cdebug == true){
  if (t100.repeat()) { Serial.println(cur_avg);} //This function is called every 100ms
}

if (t1.repeat()) { OC(); } //This function is called every 1ms
if (t100.repeat()) {debug();}//This function is called every 100ms


    /////////////////////////////////   (UP)  ///////////////////////////////////

    if (digitalRead(up) == 0) {
      up_dbg = true;
      down_dbg = false;
      digitalWrite(up_rel, HIGH);
      digitalWrite(down_rel, LOW);
      if (surge_state == false) {
        surgefilter.start();//Activates the surgefilter timer
        surge_state = true;
      }
    }

    else {
     
      /////////////////////////////////   (DOWN)  ///////////////////////////////////

      if (digitalRead(down) == 0) {
        down_dbg = true;
        up_dbg = false;
        digitalWrite(up_rel, LOW);
        digitalWrite(down_rel, HIGH);
        if (surge_state == false) {
          surgefilter.start(); //Activates the surgefilter timer
          surge_state = true;
        }
      }
 
      /////////////////////////////////   (NO INPUT)  ///////////////////////////////////

      else {
        down_dbg = false;
        up_dbg = false;
        digitalWrite(up_rel, LOW);
        digitalWrite(down_rel, LOW);
        surge_state = false;
      }
    }

 
  /*------------------------------  (SURGE FILTER)  ---------------------------------
This is a simple filter to remove the surge current of the starting motor that will cause false tirgers at every
motor start. It stops the reading of the motor current when motor is started for a specified amount of time.
  */
if (surgefilter.done()) {
  surgefilter.reset();
  surge = false;
}
if (surgefilter.waiting()) {
  surge = true;
}


}//end of loop()


void debug(){
  //Checks the state of debug variables and activates the LEDs accordingly
    if (up_dbg == true) {
    digitalWrite(up_debug, HIGH);
  } else {
    digitalWrite(up_debug, LOW);
  }

  if (down_dbg == true) {
    digitalWrite(down_debug, HIGH);
  } else {
    digitalWrite(down_debug, LOW);
  }

  if (cap_dbg == true) {
    digitalWrite(cap_fault, HIGH);
  } else {
    digitalWrite(cap_fault, LOW);
  }

  if (cur_dbg == true) {
    digitalWrite(cur_fault, HIGH);
  } else {
    digitalWrite(cur_fault, LOW);
  }
}

void OC() {
  //Checks if there is an Over Current event and reacts accordingly

  //DATA COLECTION//
  voltage();
  if (surge == false) {
    cur_raw = analogRead(cur_mes);
  } else {
    cur_raw = 500;
  }
  cur_avg = average.reading(cur_raw);


  //RESPONSE//
  if (digitalRead(down) == 0) { //Checking for an OC event while the motor is moving down (This part is critical)
    if (MID - DCD > cur_avg || MID + DCD < cur_avg) {
      if (OCPR == true) {
        cur_dbg = true;
        up_dbg = true;
        down_dbg = false;
        digitalWrite(down_rel, LOW);
        digitalWrite(up_rel, HIGH);
        debug();
        delay(5000);
        digitalWrite(down_rel, LOW);
        digitalWrite(up_rel, LOW);
        cur_dbg = false;
      } else {
        digitalWrite(down_rel, LOW);
        digitalWrite(up_rel, LOW);
        cur_dbg = true;
        down_dbg = false;
        up_dbg = false;
        debug();
        delay(5000);
      }
    } else {
      cur_dbg = false;
    }
  }


  if (digitalRead(up) == 0) { //Checking for an OC event while the motor is moving up (This part is not critical)
    if (MID - UCD > cur_avg || MID + UCD < cur_avg) {
      cur_dbg = true;
      up_dbg = false;
      down_dbg = false;
      digitalWrite(down_rel, LOW);
      digitalWrite(up_rel, LOW);
      debug();
      delay(5000);
      digitalWrite(down_rel, LOW);
      digitalWrite(up_rel, LOW);
      cur_dbg = false;
    } else {
      cur_dbg = false;
    }


    if (MID - MCD > cur_avg || MID + MCD < cur_avg) { //Checking for a critical OC event
      stopprog();
    }
  }
  return cur_avg;
}

void voltage() { //Checks the current input voltage and adjust the current limits accordingly
  VR = map(analogRead(HV), 0, 1023, 0, 55);

if (OVP == true){
    if (VR > MV) { //Overvoltage protection
  stopprog();
  }
}
  if (VR <= state1 && VR >= state2) {
    UCD = UCD1;
    DCD = DCD1;
    MCD = MCD1;
  }

  if (VR <= state2 && VR >= state3) {
    UCD = UCD2;
    DCD = DCD2;
    MCD = MCD2;
  }

  if (VR <= state3 && VR >= state4) {
    UCD = UCD3;
    DCD = DCD3;
    MCD = MCD3;
  }

  if (VR <= state4) {
    UCD = UCD4;
    DCD = DCD4;
    MCD = MCD4;
  }

  return MCD, DCD, UCD;
}



void stopprog() { //This function shuts down the microcontroller untill the next reset
  shutdown = true;
  return shutdown;
}

void cap_trig(){ //This function reacts to the triggering of the capacitive sensor
  if (runaway == false){ //Stops the motor while the capacitive sensor is triggered
    while(digitalRead(stop) == 0){
      cap_dbg = true;
      if (digitalRead(up) == 1) {
    digitalWrite(up_rel, LOW);
    digitalWrite(down_rel, LOW);
    digitalWrite(up_debug, LOW);
    digitalWrite(down_debug, LOW);
    }
    else{
      digitalWrite(up_rel, HIGH);
    }
}
  }
  else{//Motor moves up while the capacitive sensor is triggered
    if(digitalRead(stop) == 0){
    digitalWrite(down_rel, LOW);
    digitalWrite(up_debug, LOW);
    digitalWrite(down_debug, LOW);
    digitalWrite(up_rel, HIGH);
    up_dbg = true;
    debug();
    }
    else{
      digitalWrite(up_rel, LOW);
      up_dbg = false;
    debug();
    }
  }
}

void lmt() { //This function stops the motor when it reaches a limit switch but allows it to move up
  while (digitalRead(limit) == 0) {
    if (digitalRead(up) == 1) {
      digitalWrite(up_rel, LOW);
      digitalWrite(down_rel, LOW);
      digitalWrite(up_debug, LOW);
      digitalWrite(down_debug, LOW);
    } else {
      digitalWrite(up_rel, HIGH);
    }
  }
}

void SerailDebug() { //This function prints out all the useful parameters to the serial monitor when its called upon
  Serial.println();
  Serial.println("--------------DEBUG INFO--------------");
  Serial.println();
  Serial.print("INPUT VOLTAGE:");
  Serial.print(map(analogRead(HV), 0, 1023, 0, 50));
  Serial.print("V (");
  Serial.print(analogRead(HV));
  Serial.print(")");
  Serial.println();
  Serial.print("MOTOR CURRENT:");
  Serial.println(cur_avg);
  Serial.print("UCD:");
  Serial.println(UCD);
  Serial.print("MCD:");
  Serial.println(MCD);
  Serial.print("DCD:");
  Serial.println(DCD);
}