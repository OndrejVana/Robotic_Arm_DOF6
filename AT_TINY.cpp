#include <Arduino.h>

#define DIR 1
#define PUL 2

int tol = 0, wait_time;
double Target_angle, Real_angle, gain = 1, corretion = 0;

void setup() {

}

void loop() {
  while(true){
    if(Target_angle > Real_angle + corretion or Target_angle > Real_angle - corretion){
      digitalWrite(DIR, HIGH);
      digitalWrite(PUL, HIGH);
      delay(wait_time);
      digitalWrite(PUL, LOW);
    }
    else if (Target_angle < Real_angle + corretion or Target_angle < Real_angle - corretion){
      digitalWrite(DIR, LOW);
      digitalWrite(PUL, HIGH);
      delay(wait_time);
      digitalWrite(PUL, LOW);
    }
    {
      /* code */
    }
    
  }

}