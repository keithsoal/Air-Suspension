#include <Controllino.h>
#include <stdio.h>

int TravelSensorA = CONTROLLINO_A5; // usefull range (24-261)
int TravelSensorB = CONTROLLINO_A3; // usefull range (80-287) max range (11-350 mV)
int TravelSensorC = CONTROLLINO_A1; // usefull range (28-264)
int lnDSPA = 0;
int lnDSPB = 0;
int lnDSPC = 0;

void setup() {
  Serial.begin(9600);
  pinMode(TravelSensorA, INPUT);
  pinMode(TravelSensorB, INPUT);
  pinMode(TravelSensorC, INPUT);
}

void loop() {

  lnDSPA = analogRead(TravelSensorA);
  lnDSPB = analogRead(TravelSensorB);
  lnDSPC = analogRead(TravelSensorC);

  Serial.print("Poti A: "); Serial.println(lnDSPA);
  Serial.print("Poti B: "); Serial.println(lnDSPB);
  Serial.print("Poti C: "); Serial.println(lnDSPC);
  
}
