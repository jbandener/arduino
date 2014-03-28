/*
Date: 2014-03-28

Description:
This application is desined to control the bubble gum dispenser 

ToDo: add NFC support
*/


#include <avr/io.h>
#include <util/delay.h>

/*
PIN DEFINITIONS
*/
int lightSensor = A0;
int indicatorLED = 32;
int pwm_out = 8;
int in_pin1 = 36;
int in_pin2 = 37;

/*
CONSTANT DEFINITIONS
*/
int pwm_speed = 150;
int lightThreshold = 950;

/*
VARIABLE DEFINITIONS
*/
int sensorValue = 0;
int valid = 0;

/*
Initialize PWM motor driver
*/
void init_pwm() {
}

/*
Initialize photo barrier for detecting start and stop
*/
void init_light_barrier() {
}

void read_sensor() {
  sensorValue = analogRead(lightSensor);
  Serial.println(sensorValue);
}

/*
Man setup and pin mapping
*/
void setup() {
    Serial.begin(9600);  
    init_light_barrier();
    pinMode(indicatorLED, OUTPUT);
    digitalWrite(indicatorLED, LOW);
    pinMode(pwm_out, OUTPUT);
    analogWrite(pwm_out,0);
    pinMode(in_pin1, INPUT);
    pinMode(in_pin2, INPUT);    
}

void rotatePart() {
  read_sensor();
    if (sensorValue > lightThreshold && valid == 1) {
      digitalWrite(indicatorLED, HIGH);
      analogWrite(pwm_out,pwm_speed);
    }
    else {
      digitalWrite(indicatorLED, LOW);
      analogWrite(pwm_out,0);
      valid = 0;
    }
}

void loop() {
    rotatePart();
    if (digitalRead(in_pin1) == 1) valid = 1;
}

