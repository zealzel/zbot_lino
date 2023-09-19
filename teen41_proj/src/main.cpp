#pragma once
#include <Arduino.h>

int trigPin1 = 22;    // Trigger
int echoPin1 = 23;    // Echo
long duration, cm, inches;
float distance_m;

int trigPin[2] = {22, 35};
int echoPin[2] = {23, 36};
float ranges[2];
 
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  for (int i=0;i<2;i++) {
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }
}

void loop() {
  for (int i=0;i<2;i++) {
    digitalWrite(trigPin[i], LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[i], LOW);
    pinMode(echoPin[i], INPUT);
    duration = pulseIn(echoPin[i], HIGH);
    ranges[i] = (duration/2) * 0.01 / 29.1;     // Divide by 29.1 or multiply by 0.0343
  }
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  // digitalWrite(trigPin1, LOW);
  // delayMicroseconds(5);
  // digitalWrite(trigPin1, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(trigPin1, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  // pinMode(echoPin, INPUT);
  // duration = pulseIn(echoPin2, HIGH);
 
  // Convert the time into a distance
  // cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  // distance_m = (duration/2) * 0.01 / 29.1;     // Divide by 29.1 or multiply by 0.0343
  // inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
  
  // Serial.print(inches);
  // Serial.print("in, ");
  // Serial.print(cm);
  // Serial.print("cm, ");
  // Serial.print(distance_m);
  // Serial.print("m");
  // Serial.println();

  Serial.print("range1: ");
  Serial.print(ranges[0]);
  Serial.print("range2: ");
  Serial.print(ranges[1]);
  Serial.println();
  
  delay(250);
}
