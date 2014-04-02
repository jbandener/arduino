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

/*
PIN DEFINITIONS
*/
// FOR ARDUINO MEGA 2560
int lightSensor = A0;
int reedSensor = 30;
int indicatorLED = 32;
int pwm_out = 8;
int in_pin1 = 36;
int in_pin2 = 37;

// FOR ATMEGA48
//int lightSensor = PINC0;
//int indicatorLED = PINB5;
//int pwm_out = PINB1;
//int in_pin1 = PIND6;
//int in_pin2 = PIND7;
//int reedSensor = PINB7;



/*
CONSTANT DEFINITIONS
*/
int pwm_speed = 120;
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

void read_sensor() {
  sensorValue = analogRead(lightSensor);
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
    pinMode(indicatorLED, OUTPUT);
    digitalWrite(indicatorLED, LOW);
    pinMode(pwm_out, OUTPUT);
    analogWrite(pwm_out,0);
    pinMode(in_pin1, INPUT);
    pinMode(in_pin2, INPUT);    
    pinMode(reedSensor, INPUT);  
}

void rotatePart() {
    read_sensor();
    if(valid == 1) {
      digitalWrite(indicatorLED, HIGH);
      analogWrite(pwm_out, pwm_speed);
      
      //if (sensorValue == 1) {
        if (sensorValue <= 650) {
        //delay(reedDelay);
        digitalWrite(indicatorLED, LOW);
        analogWrite(pwm_out, 0);
        valid = 0;
      }
    }
    else {
    }
}

void initial_rotation(){
  digitalWrite(indicatorLED, HIGH);
  analogWrite(pwm_out, pwm_speed);
  delay(initialDelay);
}

void loop() {
    rotatePart();
    if (digitalRead(in_pin1) == 1) {
      valid = 1;
      initial_rotation();
      }
}

