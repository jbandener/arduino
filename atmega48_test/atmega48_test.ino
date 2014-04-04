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

#include <avr/io.h>
#include <stdlib.h>
#include <avr/sfr_defs.h>
//#include <avr/deprecated.h>

#define sbi(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define cbi(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define outp(VAL,ADRESS) ADRESS=VAL
#define inp(VAL) VAL

#endif

int led = 21;
int inValue = 0;

int ADC0read(void) {
    int inValue = 0;
    ADMUX = ((0<<REFS1)|(1<<REFS0)|0x00);  // select internal AVCC ref and CHANNEL 0 for ADC
    ADCSRA = ((1<<ADEN)|(1<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0));  // select pre scale
    ADCSRB &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0)); // select trigger source as continous
    ADCSRA |= (1<<ADSC);  // start conversion
    while((ADCSRA&ADSC)!=ADSC) {   
    }
    inValue = ADCL;
    inValue = inValue + (ADCH<<8);
    return inValue;
}


void pwm21(int val) {
  if (val == 0)
  {
     cbi(TCCR0A, COM0B1);
     pinMode(21,OUTPUT);
     digitalWrite(21, LOW);
  }
  else if (val == 255)
  {
    cbi(TCCR0A, COM0B1);
    pinMode(21,OUTPUT);
    digitalWrite(21, HIGH);
  }
  else
  {
    sbi(TCCR0A, COM0B1);
    OCR0B = val;
  }	
}


int tmp=0;
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT); 
  pinMode(8,INPUT);  
  clock_prescale_set(clock_div_1);  // set frequency back to 8MHz (8MHz / div1)
  delay(50);
}

// the loop routine runs over and over again forever:
void loop() {
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(1);               // wait for a second
  //analogWrite(led,100);
  tmp = ADC0read();
  if(tmp<=650)  pwm21(0);
  else pwm21(255);
}
