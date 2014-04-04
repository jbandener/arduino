/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:

#ifndef Arduino_h
#define Arduino_h

#include <WProgram.h>
#include <avr/power.h>

#endif

int led = 22;


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
  clock_prescale_set(clock_div_1);  // set frequency back to 8MHz (8MHz / div1)
}

// the loop routine runs over and over again forever:
void loop() {
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(1);               // wait for a second
  analogWrite(led,100);
}
