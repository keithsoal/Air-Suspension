#include <Controllino.h>
#include <stdio.h>

// Cylinder Middle Position ----------------------------------------
const long middlePosition = 180;  //  115 200
const long middlePositionRange = 25; // 25 start test 50 / 40 was too big
const long relativeABCdist = 20; // 15

// Lights & Buttons ------------------------------------------------
const int GreenLight   = CONTROLLINO_R13;
const int RedLight   = CONTROLLINO_R12;
const int OrangeLight   = CONTROLLINO_R14;
const int GreenButton  = CONTROLLINO_A14;
const int RedButton  = CONTROLLINO_A15;
bool GreenFlag = false;
bool RedFlag = false;
bool OrangeFlag = false;

unsigned long previousMillis = 0;
unsigned long previousMillisThreshold = 0;
const long interval = 1000; // interval at which to blink
unsigned long serialPreviousMillis = 0;
const long serialInterval = 1500; // interval at which to write serial
unsigned long previousMillisOrange = 0;

// Station A -------------------------------------------------------
const int VentilA_SchnellEin = CONTROLLINO_R0;
const int VentilA_SchnellAus = CONTROLLINO_R1;
const int VentilA_Heben = CONTROLLINO_R2;
const int VentilA_Senken = CONTROLLINO_R3;

int TravelSensorA = CONTROLLINO_A5; // usefull range (24-261)
int lnDSPA = 0;

// Station B -------------------------------------------------------
const int VentilB_SchnellEin = CONTROLLINO_R4;
const int VentilB_SchnellAus = CONTROLLINO_R5;
const int VentilB_Heben = CONTROLLINO_R6;
const int VentilB_Senken = CONTROLLINO_R7;

int TravelSensorB = CONTROLLINO_A3; // usefull range (80-287) max range (11-350 mV)
int lnDSPB = 0;

// Station C -------------------------------------------------------
const int VentilC_SchnellEin = CONTROLLINO_R8;
const int VentilC_SchnellAus = CONTROLLINO_R9;
const int VentilC_Heben = CONTROLLINO_R10;
const int VentilC_Senken = CONTROLLINO_R11;

int TravelSensorC = CONTROLLINO_A1; // usefull range (28-264)
int lnDSPC = 0;

// Other ----------------------------------------------------------
int state = 0;
bool senkenFlagA = false;
bool senkenFlagB = false;
bool senkenFlagC = false;

// Pressure vessels state
bool TankA = false;
bool TankB = false;
bool TankC = false;

// poti TARE
int TareA = 0;
int TareB = 0;
int TareC = 0;
int LiftOff = 10; // mV for cyclinder off bottom position

int staticRange = 5; // tolerance mV for value close

// Overshoot
unsigned long previousMillisOverShootA = 0;
unsigned long previousMillisOverShootB = 0;
unsigned long previousMillisOverShootC = 0;
int overShootCountA = 0;
int overShootCountB = 0;
int overShootCountC = 0;
int overShootBuffer = 30;

// DELAY
unsigned long delayMillisA = 0;
int delayFlagA = 0;
unsigned long delayMillisB = 0;
int delayFlagB = 0;
unsigned long delayMillisC = 0;
int delayFlagC = 0;

// Cylinder Rise Delay
int cylinderRiseFlag = 0;
unsigned long cylinderRiseMillis = 0;
const long delayInterval = 500; //

const long DELAY = 5000; // delay interval

// ----------------------------------------------------------------

void setup() {
  // setup code:
  Serial.begin(9600); // starts the serial monitor
  pinMode(GreenButton, INPUT);
  pinMode(RedButton, INPUT);
  pinMode(GreenLight, OUTPUT);
  pinMode(RedLight, OUTPUT);
  pinMode(OrangeLight, OUTPUT);
  // Station A -----------------------
  pinMode(VentilA_SchnellEin, OUTPUT);
  pinMode(VentilA_SchnellAus, OUTPUT);
  pinMode(VentilA_Heben, OUTPUT);
  pinMode(VentilA_Senken, OUTPUT);
  pinMode(TravelSensorA, INPUT);
  // Station B -----------------------
  pinMode(VentilB_SchnellEin, OUTPUT);
  pinMode(VentilB_SchnellAus, OUTPUT);
  pinMode(VentilB_Heben, OUTPUT);
  pinMode(VentilB_Senken, OUTPUT);
  pinMode(TravelSensorB, INPUT);
  // Station C -----------------------
  pinMode(VentilC_SchnellEin, OUTPUT);
  pinMode(VentilC_SchnellAus, OUTPUT);
  pinMode(VentilC_Heben, OUTPUT);
  pinMode(VentilC_Senken, OUTPUT);
  pinMode(TravelSensorC, INPUT);
}

void loop() {

  // TARE -------------------------------------------------------------

  if (state == 0 && TareA == 0 && TareB == 0 && TareC == 0) {
    TareA = tare_potiA();
    TareB = tare_potiB();
    TareC = tare_potiC();
  }

  // STATE 0 ----------------------------------------------------------

  if (state == 0) {
    digitalWrite(RedLight, HIGH);
    RedFlag = true;
  }

  if (state == 0) {
    TankA = check_tankA_condition(TareA, LiftOff);
    TankB = check_tankB_condition(TareB, LiftOff);
    TankC = check_tankC_condition(TareC, LiftOff);
  }

  // STATE 1 ----------------------------------------------------------
  // start filling cylinders
  if (state == 0 && digitalRead(GreenButton) == HIGH) {
    start_filling_cyclinders();
    // Lights
    digitalWrite(GreenLight, HIGH);
    digitalWrite(RedLight, HIGH);
    GreenFlag = true;
    RedFlag = true;
    state  = 1;
  }

  // Blinking Green Light
  if (state == 1) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      GreenFlag = blinking_green_light(GreenFlag);
    }
  }

  // stop filling cylinders
  if (state == 1 && digitalRead(RedButton) == HIGH) {
    close_valves();
    digitalWrite(GreenLight, LOW);
    digitalWrite(RedLight, HIGH);
    GreenFlag = false;
    RedFlag = true;
    state = 0;
  }

  // STATE 2 ----------------------------------------------------------
  // if initial displacement is achieved, stop filling and wait
  if (state == 1) {
    TankA =  check_initial_displacementA(TareA, LiftOff);
    TankB =  check_initial_displacementB(TareB, LiftOff);
    TankC =  check_initial_displacementC(TareC, LiftOff);
  }

  // stop filling and wait
  if (state == 1 && TankA == true && TankB == true && TankC == true) {
    close_valves();
    // Lights
    digitalWrite(GreenLight, HIGH);
    GreenFlag = true;
    state = 2;
  }


  // STATE 3 ----------------------------------------------------------
  // wait for user push and pump up to pre-set level
  if (state == 2 && digitalRead(GreenButton) == HIGH) {
    ventil_heben();
    state  = 3;
    digitalWrite(GreenLight, HIGH);
    digitalWrite(RedLight, LOW);
    GreenFlag = true;
    RedFlag = false;
    delay(500);
  }

  // push up step wise to level
  if (state == 3) {

// THIS DOESNT WORK YET
//    if (cylinderRiseFlag == 0){
//      cylinderRiseMillis = millis();
//      cylinderRiseFlag = 1;
//      delay(100);
//    }
//    unsigned long currentMillis = millis();
//    if (currentMillis - cylinderRiseMillis >= delayInterval) {
//      // check that level relative to other displacement sensors does not exceed threshold (ONLY CHECKS EVERY 2 SECONDS)
//      cylinder_relative_rise(TareA, TareB, TareC, middlePosition, relativeABCdist);
//      cylinderRiseFlag = 0;
//    }
//    else {
//      cylinderRiseFlag = 0;
//    }

    // REMOVE THIS LINE
    cylinder_relative_rise(TareA, TareB, TareC, middlePosition, relativeABCdist);

    lnDSPA = analogReadA(TareA);
    lnDSPB = analogReadB(TareB);
    lnDSPC = analogReadC(TareC);
    // if system reached middle position close valves
    if (lnDSPA >= middlePosition && lnDSPB >= middlePosition && lnDSPC >= middlePosition) {
      state = 4;
      close_valves();
      digitalWrite(GreenLight, HIGH);
      digitalWrite(RedLight, LOW);
      GreenFlag = true;
      RedFlag = false;
    }
  }

  // Blinking green and Light
  if (state == 3) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      GreenFlag = blinking_green_light(GreenFlag);
    }
  }

  // Red Button options
  if (state == 3 && digitalRead(RedButton) == HIGH) {
    close_valves();
    // Lights
    digitalWrite(GreenLight, LOW);
    digitalWrite(RedLight, HIGH);
    GreenFlag = false;
    RedFlag = true;
    state = 6;
    delay(500);
  }

  // STATE 4 ----------------------------------------------------------
  // maintain middle position

  if (state == 4) {
    // check that level is not dropping +- threshold
    lnDSPA = analogReadA(TareA);
    lnDSPB = analogReadB(TareB);
    lnDSPC = analogReadC(TareC);
    if (lnDSPA < middlePosition - middlePositionRange) {

      // initialize delayMillis on first entry
      if (delayFlagA == 0){
        delayMillisA = millis();
        // set delay flag 1 start delay timer
        delayFlagA = 1;
        delay(100);
      }

      // if delay timer exceeds 5 seconds change state
      unsigned long currentMillis = millis();
      if (currentMillis - delayMillisA > DELAY) {
        state = 3;
        delayFlagA = 0;
      }
      // else keep counting   
    }
    else {
      delayFlagA = 0;
    }
    
    if (lnDSPB < middlePosition - middlePositionRange) {

      // initialize delayMillis on first entry
      if (delayFlagB == 0){
        delayMillisB = millis();
        // set delay flag 1 start delay timer
        delayFlagB = 1;
        delay(100);
      }

      // if delay timer exceeds 5 seconds change state
      unsigned long currentMillis = millis();
      if (currentMillis - delayMillisB > DELAY) {
        state = 3;
        delayFlagB = 0;
      }
      // else keep counting 
    }
    else {
      delayFlagB = 0;
    }
    if (lnDSPC < middlePosition - middlePositionRange) {

      // initialize delayMillis on first entry
      if (delayFlagC == 0){
        delayMillisC = millis();
        // set delay flag 1 start delay timer
        delayFlagC = 1;
        delay(100);
      }

      // if delay timer exceeds 5 seconds change state
      unsigned long currentMillis = millis();
      if (currentMillis - delayMillisC > DELAY) {
        state = 3;
        delayFlagC = 0;
      }
      // else keep counting 
    }
    else {
      delayFlagC = 0;
    }

    // Overshoot checks every 2 seconds

    if (lnDSPA > middlePosition + middlePositionRange + overShootBuffer) {
      digitalWrite(VentilA_Senken, HIGH);
      senkenFlagA = true;
    }
    if (lnDSPB > middlePosition + middlePositionRange + overShootBuffer) {
      digitalWrite(VentilB_Senken, HIGH);
      senkenFlagB = true;
    }
    if (lnDSPC > middlePosition + middlePositionRange + overShootBuffer) {
      digitalWrite(VentilC_Senken, HIGH);
      senkenFlagC = true;
    }
  }

  // cylinder A
  if (state == 4 && senkenFlagA == true) {
    lnDSPA = analogReadA(TareA);
    if (lnDSPA < middlePosition + middlePositionRange) {
      digitalWrite(VentilA_Senken, LOW);
      senkenFlagA = false;
    }
  }
  // cylinder B
  if (state == 4 && senkenFlagB == true) {
    lnDSPB = analogReadB(TareB);
    if (lnDSPB < middlePosition + middlePositionRange) {
      digitalWrite(VentilB_Senken, LOW);
      senkenFlagB = false;
    }
  }
  // cylinder C
  if (state == 4 && senkenFlagC == true) {
    lnDSPC = analogReadC(TareC);
    if (lnDSPC < middlePosition + middlePositionRange) {
      digitalWrite(VentilC_Senken, LOW);
      senkenFlagC = false;
    }
  }

  // STATE 5 ----------------------------------------------------------
  // future place holder


  // STATE 6 ----------------------------------------------------------
  // Senken

  if (state == 2 && digitalRead(RedButton) == HIGH) {
    ventil_senken();
    // Lights
    digitalWrite(GreenLight, LOW);
    GreenFlag = false;
    state = 6;
    delay(500);
  }

  // Red Button options
  if (state == 4 && digitalRead(RedButton) == HIGH) {
    ventil_senken();
    // Lights
    digitalWrite(GreenLight, LOW);
    GreenFlag = false;
    state = 6;
    delay(500);
  }

  if (state == 6) {
    // check that level relative to other displacement sensors does not exceed threshold
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisThreshold >= 2 * interval) {
      // save the last time you blinked the LED
      previousMillisThreshold = currentMillis;
      // check that level relative to other displacement sensors does not exceed threshold (ONLY CHECKS EVERY 2 SECONDS)
      cylinder_relative_descend(TareA, TareB, TareC, LiftOff, relativeABCdist);
    }
  }

  // Blinking Red Light
  if (state == 6) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      RedFlag = blinking_red_light(RedFlag);
    }
  }

  if (state == 6 && digitalRead(GreenButton) == HIGH ) {
    close_valves();
    state = 2;
    delay(500);
  }

  //  if (state == 6 && analogRead(TravelSensorA) < stationA) {
  //    digitalWrite(VentilA_Senken, LOW);
  //  }
  //  if (state == 6 && analogRead(TravelSensorB) < stationB) {
  //    digitalWrite(VentilB_Senken, LOW);;
  //  }
  //  if (state == 6 && analogRead(TravelSensorC) < stationC) {
  //    digitalWrite(VentilC_Senken, LOW);
  //  }
  // check all three are down
  if (state == 6 && analogReadA(TareA) <= LiftOff + staticRange && analogReadB(TareB) <= LiftOff + staticRange && analogReadC(TareC) <= LiftOff + staticRange) {
    close_valves();
    state = 0;
  }

  // STATE 7 ----------------------------------------------------------
  // Schnellaus

  if (state == 0 && digitalRead(RedButton) == HIGH) {
    ventil_schnell_aus();
    // Lights
    digitalWrite(GreenLight, LOW);
    GreenFlag = false;
    state = 7;
    delay(500);
  }

  // Blinking Light
  if (state == 7) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      RedFlag = blinking_red_light(RedFlag);
    }
  }

  if (state == 7 && digitalRead(GreenButton) == HIGH) {
    close_valves();
    state = 0;
    delay(500);
  }

  if (digitalRead(GreenButton) == HIGH && digitalRead(RedButton) == HIGH) {
    close_valves();
    digitalWrite(RedLight, HIGH);
    RedFlag = true;
    state = 0;
    delay(1000);
  }

  // Poti failure ----------------------------------------------
  //
  //   if (state == 3 && analogRead(TravelSensorA) < stationA || state == 3 && analogRead(TravelSensorB) < stationB ||
  //   state == 3 && analogRead(TravelSensorC) < stationC || state == 4 && analogRead(TravelSensorA) < stationA ||
  //   state == 4 && analogRead(TravelSensorB) < stationB || state == 4 && analogRead(TravelSensorC) < stationC) {
  //    ventil_senken();
  //    digitalWrite(RedLight, HIGH);
  //    RedFlag = true;
  //    state = 6;
  //  }

  //if (state == 4 && analogRead(TravelSensorA) < stationA ||
  //    state == 4 && analogRead(TravelSensorB) < stationB || state == 4 && analogRead(TravelSensorC) < stationC) {
  //  ventil_senken();
  //  digitalWrite(RedLight, HIGH);
  //  RedFlag = true;
  //  state = 6;
  //}

  // OTHER ----------------------------------------------------------
  // Serial print

  lnDSPA = analogReadA(TareA);
  lnDSPB = analogReadB(TareB);
  lnDSPC = analogReadC(TareC);

  // reduced speed to serial write
  unsigned long currentMillis = millis();
  if (currentMillis - serialPreviousMillis >= serialInterval) {
    // save the last time you blinked the LED
    serialPreviousMillis = currentMillis;

    // Serial Monitor
    Serial.print("Poti A: "); Serial.println(lnDSPA);
    Serial.print("Poti B: "); Serial.println(lnDSPB);
    Serial.print("Poti C: "); Serial.println(lnDSPC);
    Serial.print("State: "); Serial.println(state);
    Serial.print("TankA: "); Serial.println(TankA);
    Serial.print("TankB: "); Serial.println(TankB);
    Serial.print("TankC: "); Serial.println(TankC);
    Serial.println("-----------");
    
    // Python -> InfluxDB -> Grafana
//    Serial.print(lnDSPA);
//    Serial.print(",");
//    Serial.print(lnDSPB);
//    Serial.print(",");
//    Serial.print(lnDSPC);
//    Serial.print(",");
//    Serial.println(state);
  }

  // Blinking Orange Light
  if ( state == 3 ||  state == 4 ||  state == 6 ) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisOrange >= interval) {
      // save the last time you blinked the LED
      previousMillisOrange = currentMillis;
      OrangeFlag = blinking_orange_light(OrangeFlag);
    }
  }

  // Orange light off
  if (state == 0 || state == 1 || state == 2 ||  state == 7) {
    digitalWrite(OrangeLight, LOW);
    OrangeFlag = false;
  }


}

// TARE -----------------------------------------------------------------

int tare_potiA() {
  int TareA;
  TareA = analogRead(TravelSensorA);
  return TareA;
}

int tare_potiB() {
  int TareB;
  TareB = analogRead(TravelSensorB);
  return TareB;
}

int tare_potiC() {
  int TareC;
  TareC = analogRead(TravelSensorC);
  return TareC;
}


// Analog Read ----------------------------------------------------------

int analogReadA(int TareA) {
  int lnDSPA;
  lnDSPA = analogRead(TravelSensorA);
  lnDSPA = lnDSPA - TareA;
  return lnDSPA;
}

int analogReadB(int TareB) {
  int lnDSPB;
  lnDSPB = analogRead(TravelSensorB);
  lnDSPB = lnDSPB - TareB;
  return lnDSPB;
}

int analogReadC(int TareC) {
  int lnDSPC;
  lnDSPC = analogRead(TravelSensorC);
  lnDSPC = lnDSPC - TareC;
  return lnDSPC;
}

// Valve Control ------------------------------------------------------

void start_filling_cyclinders() {
  // start filling cyclinders
  digitalWrite(VentilA_SchnellEin, HIGH);
  digitalWrite(VentilB_SchnellEin, HIGH);
  digitalWrite(VentilC_SchnellEin, HIGH);
  digitalWrite(VentilA_SchnellAus, LOW);
  digitalWrite(VentilB_SchnellAus, LOW);
  digitalWrite(VentilC_SchnellAus, LOW);
  digitalWrite(VentilA_Heben, LOW);
  digitalWrite(VentilB_Heben, LOW);
  digitalWrite(VentilC_Heben, LOW);
  digitalWrite(VentilA_Senken, LOW);
  digitalWrite(VentilB_Senken, LOW);
  digitalWrite(VentilC_Senken, LOW);
  delay(500);
}


void close_valves() {
  digitalWrite(VentilA_SchnellEin, LOW);
  digitalWrite(VentilB_SchnellEin, LOW);
  digitalWrite(VentilC_SchnellEin, LOW);
  digitalWrite(VentilA_Heben, LOW);
  digitalWrite(VentilB_Heben, LOW);
  digitalWrite(VentilC_Heben, LOW);
  digitalWrite(VentilA_SchnellAus, LOW);
  digitalWrite(VentilB_SchnellAus, LOW);
  digitalWrite(VentilC_SchnellAus, LOW);
  digitalWrite(VentilA_Senken, LOW);
  digitalWrite(VentilB_Senken, LOW);
  digitalWrite(VentilC_Senken, LOW);
  delay(500);
}

void ventil_heben() {
  digitalWrite(VentilA_Heben, HIGH);
  digitalWrite(VentilB_Heben, HIGH);
  digitalWrite(VentilC_Heben, HIGH);
  digitalWrite(VentilA_Senken, LOW);
  digitalWrite(VentilB_Senken, LOW);
  digitalWrite(VentilC_Senken, LOW);
  digitalWrite(VentilA_SchnellEin, LOW);
  digitalWrite(VentilB_SchnellEin, LOW);
  digitalWrite(VentilC_SchnellEin, LOW);
  digitalWrite(VentilA_SchnellAus, LOW);
  digitalWrite(VentilB_SchnellAus, LOW);
  digitalWrite(VentilC_SchnellAus, LOW);
  delay(500);
}

void ventil_senken() {
  digitalWrite(VentilA_Senken, HIGH);
  digitalWrite(VentilB_Senken, HIGH);
  digitalWrite(VentilC_Senken, HIGH);
  digitalWrite(VentilA_Heben, LOW);
  digitalWrite(VentilB_Heben, LOW);
  digitalWrite(VentilC_Heben, LOW);
  digitalWrite(VentilA_SchnellEin, LOW);
  digitalWrite(VentilB_SchnellEin, LOW);
  digitalWrite(VentilC_SchnellEin, LOW);
  digitalWrite(VentilA_SchnellAus, LOW);
  digitalWrite(VentilB_SchnellAus, LOW);
  digitalWrite(VentilC_SchnellAus, LOW);
  delay(500);
}

void ventil_schnell_aus() {
  digitalWrite(VentilA_SchnellAus, HIGH);
  digitalWrite(VentilB_SchnellAus, HIGH);
  digitalWrite(VentilC_SchnellAus, HIGH);
  digitalWrite(VentilA_Senken, LOW);
  digitalWrite(VentilB_Senken, LOW);
  digitalWrite(VentilC_Senken, LOW);
  digitalWrite(VentilA_Heben, LOW);
  digitalWrite(VentilB_Heben, LOW);
  digitalWrite(VentilC_Heben, LOW);
  digitalWrite(VentilA_SchnellEin, LOW);
  digitalWrite(VentilB_SchnellEin, LOW);
  digitalWrite(VentilC_SchnellEin, LOW);
  delay(500);
}

// Poti checks ------------------------------------------------------

bool check_initial_displacementA(int TareA, int LiftOff) {
  if (analogReadA(TareA) > LiftOff) {
    digitalWrite(VentilA_SchnellEin, LOW);
    TankA = true;
  }
  return TankA;
}
bool check_initial_displacementB(int TareB, int LiftOff) {
  if (analogReadB(TareB) > LiftOff) {
    digitalWrite(VentilB_SchnellEin, LOW);
    TankB = true;
  }
  return TankB;
}
bool check_initial_displacementC(int TareC, int LiftOff) {
  if (analogReadC(TareC) > LiftOff) {
    digitalWrite(VentilC_SchnellEin, LOW);
    TankC = true;
  }
  return TankC;
}

bool check_tankA_condition(int TareA, int LiftOff) {
  if (analogReadA(TareA) < LiftOff) {
    TankA = false;
  }
  return TankA;
}
bool check_tankB_condition(int TareB, int LiftOff) {
  if (analogReadB(TareB) < LiftOff) {
    TankB = false;
  }
  return TankB;
}
bool check_tankC_condition(int TareC, int LiftOff) {
  if (analogReadC(TareC) < LiftOff) {
    TankC = false;
  }
  return TankC;
}

void cylinder_relative_rise(int TareA, int TareB, int TareC, int middlePosition, int relativeABCdist) {
  // check that level relative to other displacement sensors does not exceed threshold
  lnDSPA = analogReadA(TareA);
  lnDSPB = analogReadB(TareB);
  lnDSPC = analogReadC(TareC);
  // cylinder A
  if (lnDSPA < middlePosition && lnDSPA <= (lnDSPB + relativeABCdist) && lnDSPA <= (lnDSPC + relativeABCdist)) {
    digitalWrite(VentilA_Heben, HIGH);
  }
  else {
    digitalWrite(VentilA_Heben, LOW);
  }
  // cylinder B
  if (lnDSPB < middlePosition && lnDSPB <= (lnDSPC + relativeABCdist) && lnDSPB <= (lnDSPA + relativeABCdist)) {
    digitalWrite(VentilB_Heben, HIGH);
  }
  else {
    digitalWrite(VentilB_Heben, LOW);
  }
  // cylinder C
  if (lnDSPC < middlePosition && lnDSPC <= (lnDSPA + relativeABCdist) && lnDSPC <= (lnDSPB + relativeABCdist)) {
    digitalWrite(VentilC_Heben, HIGH);
  }
  else {
    digitalWrite(VentilC_Heben, LOW);
  }
}

void cylinder_relative_descend(int TareA, int TareB, int TareC, int LiftOff, int relativeABCdist) {
  // check that level relative to other displacement sensors does not exceed threshold
  lnDSPA = analogReadA(TareA);
  lnDSPB = analogReadB(TareB);
  lnDSPC = analogReadC(TareC);
  // cylinder A
  if (lnDSPA < LiftOff || lnDSPA <= (lnDSPB - relativeABCdist) || lnDSPA <= (lnDSPC - relativeABCdist)) {
    digitalWrite(VentilA_Senken, LOW);
  }
  else {
    digitalWrite(VentilA_Senken, HIGH);
  }
  // cylinder B
  if (lnDSPB < LiftOff || lnDSPB <= (lnDSPC - relativeABCdist) || lnDSPB <= (lnDSPA - relativeABCdist)) {
    digitalWrite(VentilB_Senken, LOW);
  }
  else {
    digitalWrite(VentilB_Senken, HIGH);
  }
  // cylinder C
  if (lnDSPC < LiftOff || lnDSPC <= (lnDSPA - relativeABCdist) || lnDSPC <= (lnDSPB - relativeABCdist)) {
    digitalWrite(VentilC_Senken, LOW);
  }
  else {
    digitalWrite(VentilC_Senken, HIGH);
  }
}

// Blinker ------------------------------------------------------

bool blinking_green_light(bool GreenFlag) {
  // if the LED is off turn it on and vice-versa:
  if (GreenFlag == false) {
    digitalWrite(GreenLight, HIGH);
    GreenFlag = true;
  }
  else {
    digitalWrite(GreenLight, LOW);
    GreenFlag = false;
  }
  return GreenFlag;
}

bool blinking_red_light(bool RedFlag) {
  // if the LED is off turn it on and vice-versa:
  if (RedFlag == false) {
    digitalWrite(RedLight, HIGH);
    RedFlag = true;
  }
  else {
    digitalWrite(RedLight, LOW);
    RedFlag = false;
  }
  return RedFlag;
}

bool blinking_orange_light(bool OrangeFlag) {
  // if the LED is off turn it on and vice-versa:
  if (OrangeFlag == false) {
    digitalWrite(OrangeLight, HIGH);
    OrangeFlag = true;
  }
  else {
    digitalWrite(OrangeLight, LOW);
    OrangeFlag = false;
  }
  return OrangeFlag;
}