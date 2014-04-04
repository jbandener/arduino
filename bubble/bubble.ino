/*
Date: 2014-03-28

Description:
This application is desined to control the bubble gum dispenser 

ToDo: add NFC support

CHANGELOG:
+ switched from light barrier to reed-sensor
*/


#include <avr/io.h>
#include <util/delay.h>

#include <avr/power.h>


#define sbi(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define cbi(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define outp(VAL,ADRESS) ADRESS=VAL
#define inp(VAL) VAL



/*
PIN DEFINITIONS
*/
// FOR ARDUINO MEGA 2560
//int lightSensor = A0;
//int reedSensor = 30;
//int indicatorLED = 32;
//int pwm_out = 8;
//int in_pin1 = 36;
//int in_pin2 = 37;

// FOR ATMEGA48
int lightSensor = 8;
int indicatorLED = 5;
int pwm_out = 21
;
int in_pin1 = 17;
int in_pin2 = 18;
int reedSensor = 7;



/*
CONSTANT DEFINITIONS
*/
int pwm_speed = 150;
int lightThreshold = 950;
int reedDelay = 10;
int initialDelay = 600;

/*
VARIABLE DEFINITIONS
*/
int sensorValue = 0;
int valid = 0;
int reedCounter = 0;

/*
Initialize PWM motor driver
*/
//void init_pwm() {
//}

/*
Initialize photo barrier for detecting start and stop
*/
//void init_light_barrier() {
//}

/*
set compare register for OC0B (pin 21 on atmega48)
*/
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

/*
reading out ADC0 
*/
int ADC0read(void) {
    int inValue = 0;
    ADMUX = ((0<<REFS1)|(1<<REFS0)|0x00);  // select internal AVCC ref and CHANNEL 0 for ADC
    ADCSRA = ((1<<ADEN)|(1<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0));  // select pre scale
    ADCSRB &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0)); // select trigger source as continous
    ADCSRA |= (1<<ADSC);  // start conversion
    delay(10);
    while((ADCSRA&ADSC)!=ADSC) {   
    }
    inValue = ADCL;
    inValue = inValue + (ADCH<<8);
    return inValue;
}

void read_sensor() {
  //sensorValue = analogRead(lightSensor);
  sensorValue = ADC0read();
//  sensorValue = digitalRead(reedSensor);
//  Serial.println(sensorValue);
//  Serial.println(analogRead(lightSensor));
}

/*
Man setup and pin mapping
*/
void setup() {
//    Serial.begin(9600);  
//    init_light_barrier();
    clock_prescale_set(clock_div_1);  // set frequency back to 8MHz (8MHz / div1) // only used for atmega48 standalone
    pinMode(indicatorLED, OUTPUT);
    pinMode(lightSensor, INPUT);
    digitalWrite(indicatorLED, LOW);
    pinMode(pwm_out, OUTPUT);
    pwm21(0);
    pinMode(in_pin1, INPUT);
    pinMode(in_pin2, INPUT);    
    //pinMode(reedSensor, INPUT);  
}

void rotatePart() {
    read_sensor();
    if(valid == 1) {
      digitalWrite(indicatorLED, HIGH);
      //analogWrite(pwm_out, pwm_speed);
      pwm21(pwm_speed);
      
      //if (sensorValue == 1) {
        if (sensorValue <= 650) {
        //delay(reedDelay);
        digitalWrite(indicatorLED, LOW);
        //analogWrite(pwm_out, 0);
        pwm21(0);
        valid = 0;
      }
    }
    else {
    }
}

void initial_rotation(){
  digitalWrite(indicatorLED, HIGH);
  //analogWrite(pwm_out, pwm_speed);
  pwm21(pwm_speed);
  delay(initialDelay);
}

void loop() {
    rotatePart();
    if (digitalRead(in_pin1) == 1) {
      valid = 1;
      initial_rotation();
      }
}

