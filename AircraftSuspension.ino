#include <Controllino.h>
#include <stdio.h>

// Cylinder Middle Position ----------------------------------------
const long offSet = 115;  //  200
const long offSetRange = 25; //50
const long relativeABCdist = 15;
const long staticRange = 4;

// Lights & Buttons ------------------------------------------------
const int GreenLight   = CONTROLLINO_R13;
const int RedLight   = CONTROLLINO_R12;
const int GreenButton  = CONTROLLINO_A14;
const int RedButton  = CONTROLLINO_A15;
bool GreenFlag = false;
bool RedFlag = false;

unsigned long previousMillis = 0;
const long interval = 1000; // interval at which to blink
unsigned long serialPreviousMillis = 0;
const long serialInterval = 300; // interval at which to write serial

// Station A -------------------------------------------------------
const int VentilA_SchnellEin = CONTROLLINO_R0;
const int VentilA_SchnellAus = CONTROLLINO_R1;
const int VentilA_Heben = CONTROLLINO_R2;
const int VentilA_Senken = CONTROLLINO_R3;

int TravelSensorA           = CONTROLLINO_A1;
int lnDSPA = 0;

// Station B -------------------------------------------------------
const int VentilB_SchnellEin = CONTROLLINO_R4;
const int VentilB_SchnellAus = CONTROLLINO_R5;
const int VentilB_Heben = CONTROLLINO_R6;
const int VentilB_Senken = CONTROLLINO_R7;

int TravelSensorB           = CONTROLLINO_A3;
int lnDSPB = 0;

// Station C -------------------------------------------------------
const int VentilC_SchnellEin = CONTROLLINO_R8;
const int VentilC_SchnellAus = CONTROLLINO_R9;
const int VentilC_Heben = CONTROLLINO_R10;
const int VentilC_Senken = CONTROLLINO_R11;

int TravelSensorC           = CONTROLLINO_A5;
int lnDSPC = 0;

// Other ----------------------------------------------------------
int state = 0;
bool senkenFlagA = false;
bool senkenFlagB = false;
bool senkenFlagC = false;

// initial poti voltage
const long stationA = 25;
const long stationB = 5;
const long stationC = 5;
// ----------------------------------------------------------------

void setup() {
  // setup code:
  Serial.begin(9600); // starts the serial monitor
  pinMode(GreenButton, INPUT);
  pinMode(RedButton, INPUT);
  pinMode(GreenLight, OUTPUT);
  pinMode(RedLight, OUTPUT);
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


  // STATE 0 ----------------------------------------------------------

  if (state == 0) {
    digitalWrite(RedLight, HIGH);
    RedFlag = true;
  }

  // STATE 1 ----------------------------------------------------------
  // start filling cylinders
  if (state == 0 && digitalRead(GreenButton) == HIGH) {
    start_filling_cyclinders();
    state  = 1;
    GreenFlag = true;
    RedFlag = true;
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
    stop_filling_cylinders();
    GreenFlag = false;
    RedFlag = true;
    state = 0;
  }

  // STATE 2 ----------------------------------------------------------
  // if initial displacement is achieved, stop filling and wait
  if (state == 1) {
    check_initial_displacement();
  }

  // stop filling and wait
  if (state == 1 && analogRead(TravelSensorA) > stationA && analogRead(TravelSensorB) > stationB && analogRead(TravelSensorC) > stationC) {
    close_valves();
    // Lights
    digitalWrite(GreenLight, HIGH);
    GreenFlag = true;
    state = 2;
  }

  // check and return to state 1 if cylinder becomes empty
  if (state == 2 && analogRead(TravelSensorA) < stationA - staticRange || state == 2 && analogRead(TravelSensorB) < stationB - staticRange || state == 2 && analogRead(TravelSensorC) < stationC - staticRange) {
    digitalWrite(GreenLight, LOW);
    GreenFlag = false;
    state = 1;
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

    // check that level relative to other displacement sensors does not exceed threshold
    cylinder_relative_rise(offSet, relativeABCdist);


    // if system reached middle position close valves
    if (lnDSPB >= offSet && lnDSPB >= offSet && lnDSPC >= offSet) {
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
    state = 2;
    delay(500);
  }

  // STATE 4 ----------------------------------------------------------
  // maintain middle position

  if (state == 4) {
    // check that level is not dropping +- threshold
    lnDSPA = analogRead(TravelSensorA);
    lnDSPB = analogRead(TravelSensorB);
    lnDSPC = analogRead(TravelSensorC);
    if (lnDSPA < offSet - offSetRange) {
      state = 3;
    }
    if (lnDSPB < offSet - offSetRange) {
      state = 3;
    }
    if (lnDSPC < offSet - offSetRange) {
      state = 3;
    }

    if (lnDSPA > offSet + offSetRange) {
      digitalWrite(VentilA_Senken, HIGH);
      senkenFlagA = true;
    }
    if (lnDSPB > offSet + offSetRange) {
      digitalWrite(VentilB_Senken, HIGH);
      senkenFlagB = true;
    }
    if (lnDSPC > offSet + offSetRange) {
      digitalWrite(VentilC_Senken, HIGH);
      senkenFlagC = true;
    }
  }

  // cylinder A
  if (state == 4 && senkenFlagA == true) {
    lnDSPA = analogRead(TravelSensorA);
    if (lnDSPA < offSet) {
      digitalWrite(VentilA_Senken, LOW);
      senkenFlagA = false;
    }
  }
  // cylinder B
  if (state == 4 && senkenFlagB == true) {
    lnDSPB = analogRead(TravelSensorB);
    if (lnDSPB < offSet) {
      digitalWrite(VentilB_Senken, LOW);
      senkenFlagB = false;
    }
  }
  // cylinder C
  if (state == 4 && senkenFlagC == true) {
    lnDSPC = analogRead(TravelSensorC);
    if (lnDSPC < offSet) {
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

  // cylinder A
  if (state == 6 && analogRead(TravelSensorA) == stationA) {
    digitalWrite(VentilA_Senken, LOW);
  }
  // cylinder B
  if (state == 6 && analogRead(TravelSensorB) == stationB) {
    digitalWrite(VentilB_Senken, LOW);;
  }
  // cylinder C
  if (state == 6 && analogRead(TravelSensorC) == stationC) {
    digitalWrite(VentilC_Senken, LOW);
  }
  // check all three are down
  if (state == 6 && analogRead(TravelSensorA) <= stationA && analogRead(TravelSensorB) <= stationB && analogRead(TravelSensorC) <= stationC) {
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

  // OTHER ----------------------------------------------------------
  // Serial print

  lnDSPA = analogRead(TravelSensorA);
  lnDSPB = analogRead(TravelSensorB);
  lnDSPC = analogRead(TravelSensorC);
  //  Serial.println(lnDSPA);
  //  Serial.println(state);

  // reduced speed to serial write
  unsigned long currentMillis = millis();
  if (currentMillis - serialPreviousMillis >= serialInterval) {
    // save the last time you blinked the LED
    serialPreviousMillis = currentMillis;

    Serial.println(lnDSPA);
    Serial.println(lnDSPB);
    Serial.println(lnDSPC);
    Serial.println(state);
  }
}

//FUNCTIONS-------------------------------------------------------

void start_filling_cyclinders() {
  // start filling cyclinders
  digitalWrite(VentilA_SchnellAus, LOW);
  digitalWrite(VentilA_SchnellEin, HIGH);
  digitalWrite(VentilB_SchnellAus, LOW);
  digitalWrite(VentilB_SchnellEin, HIGH);
  digitalWrite(VentilC_SchnellAus, LOW);
  digitalWrite(VentilC_SchnellEin, HIGH);
  // Lights
  digitalWrite(GreenLight, HIGH);
  digitalWrite(RedLight, HIGH);
  delay(500);
}

void stop_filling_cylinders() {
  digitalWrite(VentilA_SchnellEin, LOW);
  digitalWrite(VentilB_SchnellEin, LOW);
  digitalWrite(VentilC_SchnellEin, LOW);
  // Lights
  digitalWrite(GreenLight, LOW);
  digitalWrite(RedLight, HIGH);
  delay(500);
}

void check_initial_displacement() {
  if (analogRead(TravelSensorA) > stationA) {
    digitalWrite(VentilA_SchnellEin, LOW);
  }
  if (analogRead(TravelSensorA) < stationA) {
    digitalWrite(VentilA_SchnellEin, HIGH);
  }
  if (analogRead(TravelSensorB) > stationB) {
    digitalWrite(VentilB_SchnellEin, LOW);
  }
  if (analogRead(TravelSensorB) < stationB) {
    digitalWrite(VentilB_SchnellEin, HIGH);
  }
  if (analogRead(TravelSensorC) > stationC) {
    digitalWrite(VentilC_SchnellEin, LOW);
  }
  if (analogRead(TravelSensorC) < stationC) {
    digitalWrite(VentilC_SchnellEin, HIGH);
  }
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
  digitalWrite(GreenLight, HIGH);
  digitalWrite(RedLight, HIGH);
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
}

void cylinder_relative_rise(int offSet, int relativeABCdist) {
  // check that level relative to other displacement sensors does not exceed threshold
  lnDSPA = analogRead(TravelSensorA);
  lnDSPB = analogRead(TravelSensorB);
  lnDSPC = analogRead(TravelSensorC);
  // cylinder A
  if (lnDSPA < offSet && lnDSPA <= (lnDSPB + relativeABCdist) && lnDSPA <= (lnDSPC + relativeABCdist)) {
    digitalWrite(VentilA_Heben, HIGH);
  }
  else {
    digitalWrite(VentilA_Heben, LOW);
  }
  // cylinder B
  if (lnDSPB < offSet && lnDSPB <= (lnDSPC + relativeABCdist) && lnDSPB <= (lnDSPA + relativeABCdist)) {
    digitalWrite(VentilB_Heben, HIGH);
  }
  else {
    digitalWrite(VentilB_Heben, LOW);
  }
  // cylinder C
  if (lnDSPC < offSet && lnDSPC <= (lnDSPA + relativeABCdist) && lnDSPC <= (lnDSPB + relativeABCdist)) {
    digitalWrite(VentilC_Heben, HIGH);
  }
  else {
    digitalWrite(VentilC_Heben, LOW);
  }
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
}

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
