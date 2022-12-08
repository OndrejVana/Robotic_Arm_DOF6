#include <Arduino.h>

#define DIR 1
#define PUL 2

#define S_speed 500 
#define F_speed 5000
#define kp 1

int tol = 0, wait_time, Rot_dir = 1, d_speed = 0;
double Target_angle, Real_angle, gain = 1, corretion = 0;

void setup() {

}

void Step(){
  if(Target_angle > Real_angle){
    Rot_dir = -1;
  }
  else{
    Rot_dir = 1; 
  }
  int x = (abs(abs(Target_angle) - abs(Real_angle)) * kp);
  wait_time = constrain(x, S_speed, F_speed);
  
  if(Rot_dir == 1){
      digitalWrite(DIR, HIGH);
      digitalWrite(PUL, HIGH);
      delay(wait_time);
      digitalWrite(PUL, LOW);
    }
  else if (Rot_dir == -1){
      digitalWrite(DIR, LOW);
      digitalWrite(PUL, HIGH);
      delay(wait_time);
      digitalWrite(PUL, LOW);
    }

}

void loop() {
  while(true){
    
    {
      Step();
      /* code */
    }
    
  }

}