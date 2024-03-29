#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <Servo.h>

// Defines

//#define MOSI 11
//#define MISO 12
//#define SCK 13
#define SL1 5
#define SL2 6
#define SL3 7
#define SL4 8
#define SL5 9
#define SL6 10
#define SLnum 6

#define LED1 A2
#define LED2 A1
#define LED3 A0 

#define SERVOPWM 3
#define servo_open 1000
#define servo_close 1500

#define P1 2
#define P2 4

Servo servo1;

// Global variables
double DH[6][4] = {{0.01, -90, 220, 3.5},
                  {-90, 0, 0, 500}, 
                  {0.01, 90, 0, 0}, 
                  {0.01 , -90, 360, 0}, 
                  {0.01, 90, 0, 0}, 
                  {0.01, 0, 44, 0}};
int Q;
double time_d = 0;
char Angle_type;
int servo_res;
double ResVal[6] = {};
double limits[6][2] = {{360, -360},{360, -360},{360, -360},{360, -360},{360, -360},{360, -360}}; // MAX, MIN

//SPI Communication
int SPI_com(int message, int slave)
{
  uint8_t mess_L =  message;
  uint8_t mess_H = message >> 8;
  uint8_t resMess_L = 0;
  uint8_t resMess_H = 0; 
  int resMess = 0;
  if(Angle_type == 'i'|| Angle_type == 'd')
  {
  	Serial.print("Slave: ");
  	Serial.println(slave);
  }
  switch (slave)
  {
  case 1:
    digitalWrite(SL1, LOW);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, HIGH);
    resMess_L = SPI.transfer(mess_L);
    resMess_H = SPI.transfer(mess_H);
    break;
  case 2:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, LOW);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, HIGH);
    resMess_L = SPI.transfer(mess_L);
    resMess_H = SPI.transfer(mess_H);
    break;
  case 3:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, LOW);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, HIGH);
    resMess_L = SPI.transfer(mess_L);
    resMess_H = SPI.transfer(mess_H);
    break;
  case 4:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, LOW);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, HIGH);
    resMess_L = SPI.transfer(mess_L);
    resMess_H = SPI.transfer(mess_H);   
    break;
  case 5:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, LOW);
    digitalWrite(SL6, HIGH);
    resMess_L = SPI.transfer(mess_L);
    resMess_H = SPI.transfer(mess_H);
    break;
  case 6:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, LOW);
    resMess_L = SPI.transfer(mess_L);
    resMess_H = SPI.transfer(mess_H);
    break;
  default:
    SPI.endTransaction();
    return 0;
    break;
  }
  Serial.print("Message: ");
  Serial.println(message);
  if(Angle_type == 'i'|| Angle_type == 'd')
  {
  	Serial.print("Message L: ");
  	Serial.println(mess_L);
  	Serial.print("Message H: ");
  	Serial.println(mess_H);
  	Serial.print("Receive message L: ");
  	Serial.println(resMess_L);
  	Serial.print("Receive message H: ");
  	Serial.println(resMess_H);
  	
  }
  resMess = resMess_L << 8;
  resMess = resMess | resMess_H;
  Serial.print("Receive message: ");
  Serial.println(resMess);

  if(message == resMess){
    Serial.println("OK");
    delay(500);
    digitalWrite(SL1, HIGH);
  	digitalWrite(SL2, HIGH);
  	digitalWrite(SL3, HIGH);
  	digitalWrite(SL4, HIGH);
  	digitalWrite(SL5, HIGH);
  	digitalWrite(SL6, HIGH);
    return 1;
  }
  else{
    delay(500);
    digitalWrite(SL1, HIGH);
  	digitalWrite(SL2, HIGH);
  	digitalWrite(SL3, HIGH);
  	digitalWrite(SL4, HIGH);
  	digitalWrite(SL5, HIGH);
  	digitalWrite(SL6, HIGH);
    return 0;
  }
}
void TransSPI()
{
  int count = 1;
  int err = 0;
  int sent_err = 0; 
  while(count < SLnum + 1)
  {
  	if(Angle_type == 'i'|| Angle_type == 'd')
  	{
  		Serial.print("TRY: ");
    	Serial.println(err);
  	}
    
    if(SPI_com(DH[count-1][0], count) == 1)
    {
      count++;
    }
    else
    {
    	err++;	
    }
    
    if(err > 10)
    {
    	if(Angle_type == 'i'|| Angle_type == 'd')
    	{
    		Serial.println("ERR NEXT");
    	}
      count++;
      sent_err++;
      err = 0;
    }
    else
    {
    	digitalWrite(LED1, HIGH);
    	delay(120);
    	digitalWrite(LED1, LOW);
    	if(Angle_type == 'i'|| Angle_type == 'd')
    	{
    		Serial.println("SEND");
    	}
    }
  }
  if(sent_err == 0)
  {
  	Serial.println("ALL OK");
  	digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
  }
  else
  {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW); 
    Serial.println("ERR");  
  }
  
}

//Value limitation MAX, MIN
double ValLim(double val, double maxV, double minV)
{
  if(val >= maxV)
  {
    digitalWrite(LED2, HIGH);
    return maxV;
  }
  else if(val <= minV)
  {
    digitalWrite(LED2, HIGH);
    return minV;
  }
  else
  {
    digitalWrite(LED2, LOW);
    return val;
  }
}

//Rotational matrix 
void RotMat(double angle, double alpha, double d, double a, double(*M)[4])
{
  double angle_r = radians(angle);
  double alpha_r = radians(alpha);
  //-----
  M[0][0] = cos(angle_r);
  M[0][1] = -sin(angle_r) * cos(alpha_r);
  M[0][2] = sin(angle_r) * sin(alpha_r);
  M[0][3] = a * cos(angle_r);
  //-----
  M[1][0] = sin(angle_r);
  M[1][1] = cos(angle_r) * cos(alpha_r);
  M[1][2] = -cos(angle_r) * sin(alpha_r);
  M[1][3] = a * sin(angle_r);
  //-----
  M[2][0] = 0;
  M[2][1] = sin(alpha_r);
  M[2][2] = cos(alpha_r);
  M[2][3] = d; 
  //-----
  M[3][0] = 0;
  M[3][1] = 0;
  M[3][2] = 0;
  M[3][3] = 1;

}

//matrix mutlipication
void MatMul(double (*A)[4], double (*B)[4], int size, double (*C)[4])
{
  for(int i = 0; i < size; i++)
  {
    for(int j = 0; j < size; j++)
    {
      for(int k = 0; k < size; k++)
      {
        C[i][j] += A[i][k] * B[k][j]; 
      }
    }
  }
}

//matrix inversion
void MatInv(double (*X)[4], double (*L)[4])
{
  for(int i = 0; i < 4; i++){
    for(int y = 0; y < 4; y++){
        L[i][y] = X[y][i];
    }
  }
}

//Inverse Kinematics
void InverseKinematics(double Xcor, double Ycor, double Zcor, double y_cor, double p_cor, double r_cor)
{
  double time_s = micros();
  //QVADRANT
  if(Xcor > 0 && Ycor > 0){
    Q = 1;
  }
  else if(Xcor < 0 && Ycor > 0){
    Q = 2;
  }
  else if(Xcor < 0 && Ycor < 0){
    Q = 3;
  }
  else if(Xcor > 0 && Ycor < 0){
    Q = 4;
  }
  
  //degrees to readians
  double y_cor_r = radians(y_cor);
  double p_cor_r = radians(p_cor);
  double r_cor_r = radians(r_cor);

  //ROT Matrix
  double ROT[4][4] = {{0, 0, 0, 0}, 
                    {0, 0, 0, 0}, 
                    {0, 0, 0, 0}, 
                    {0, 0, 0, 0}};
  ROT[0][0] = -((cos(y_cor_r) * cos(r_cor_r)) - (cos(p_cor_r) * sin(y_cor_r) * sin(r_cor_r)));
  ROT[0][1] = (cos(r_cor_r) * sin(y_cor_r)) + (cos(y_cor_r) * cos(p_cor_r) * sin(r_cor_r));
  ROT[0][2] = sin(p_cor_r) * sin(r_cor_r);
  ROT[0][3] = Xcor;
  //-------
  ROT[1][0] = (cos(p_cor_r) * cos(r_cor_r) * sin(y_cor_r)) + (cos(y_cor_r) * sin(r_cor_r));
  ROT[1][1] = (cos(y_cor_r) * cos(p_cor_r) * cos(r_cor_r)) - (sin(y_cor_r) * sin(r_cor_r));
  ROT[1][2] = cos(r_cor_r) * sin(p_cor_r);
  ROT[1][3] = Ycor;
  //-------
  ROT[2][0] = sin(y_cor_r) * sin(p_cor_r);
  ROT[2][1] = cos(y_cor_r) * sin(p_cor_r);
  ROT[2][2] = -cos(p_cor_r);
  ROT[2][3] = Zcor;
  //-------
  ROT[3][0] = 0;
  ROT[3][1] = 0;
  ROT[3][2] = 0;
  ROT[3][3] = 1; 

  //R06_r Matrix
  double R06_r[4][4] = {{-1, 0.00001, 0, 0}, 
                      {-0.00001, -1, 0, 0}, 
                      {0, 0, 1, -44}, 
                      {0, 0, 0, 1}};

  
  //R05 Matrix
  double R05[4][4] = {{0}};
  MatMul(ROT, R06_r, 4, R05);

  //Angle J1
  double angle_J1 = atan(R05[1][3] / R05[0][3]);
  if(Q == 2){
    angle_J1 = degrees(angle_J1) * (-1) + 90;
  }
  else if(Q == 3){
    angle_J1 = degrees(angle_J1) * (-1) - 90;
  }
  else{
    angle_J1 = degrees(angle_J1);
  }

  //ARM POS CAL
  double pX = sqrt(R05[0][3] * R05[0][3] + R05[1][3] * R05[1][3]);
  double pY = R05[2][3] - DH[0][2];
  double pXa = pX - DH[0][3];
  double pa2H = sqrt(pY * pY + pXa * pXa);
  double pa3H = sqrt(DH[3][2] * DH[3][2] + DH[2][3] * DH[2][3]);
  double angle_J2, angle_J3, angle_A, angle_B;
  //ARM FORWARD
  if(pXa > 0)
  {
    angle_A = atan(pY / pXa);
    angle_B = acos((DH[1][3]*DH[1][3] + pa2H*pa2H - pa3H*pa3H)/(2*DH[1][3]*pa2H));
    angle_J2 = -(angle_A + angle_B);
    angle_J3 =PI - acos((pa3H*pa3H + DH[1][3]*DH[1][3] - pa2H*pa2H) / (2*pa3H*DH[1][3]));
  }
  //ARM MID
  else
  {
    angle_A = acos((DH[1][3]*DH[1][3] + pa2H*pa2H - DH[3][2]*DH[3][2])/(2*DH[1][3]*pa2H));
    angle_B = atan(pXa / pY);
    angle_J2 = -PI/2 - (angle_A + angle_B);
    angle_J3 = PI - acos((pa3H*pa3H + DH[1][3]*DH[1][3] - pa2H*pa2H)/(2*pa3H*DH[1][3]));
  }

  //DH table update
  DH[0][0] = ValLim(angle_J1, limits[0][0], limits[0][1]);
  DH[1][0] = ValLim(degrees(angle_J2), limits[1][0], limits[1][1]);
  DH[2][0] = ValLim(degrees(angle_J3), limits[2][0], limits[2][1]);

  //J1 Matrix
  double J1[4][4] = {{0}};
  RotMat(DH[0][0], DH[0][1], DH[0][2], DH[0][3], J1);
  //J2 Matrix
  double J2[4][4] = {{0}};
  RotMat(DH[1][0], DH[1][1], DH[1][2], DH[1][3], J2);
  //J3 Matrix
  double J3[4][4] = {{0}};
  RotMat(DH[2][0] - 90, DH[2][1], DH[2][2], DH[2][3], J3);

  //R03 Matrix
  double R02[4][4] = {{0}};
  MatMul(J1, J2, 4, R02);
  double R03[4][4] = {{0}};
  MatMul(R02, J3, 4, R03);
  //Inverse R03 Matrix
  double R03_inv[4][4] = {{0}};
  MatInv(R03, R03_inv);

  //R36 Matrix
  double R36[4][4] = {{0}};
  MatMul(R03_inv, R05, 3, R36);

  //J4, J5 and J6 calculation
  double angle_J4, angle_J5, angle_J6;
  if(DH[3][0] >= 0){
    angle_J5 = atan2(sqrt(1 - R36[2][2]*R36[2][2]), R36[2][2]);
    angle_J4 = atan2(R36[1][2], R36[0][2]);
    angle_J6 = atan2(R36[2][1], -R36[2][0]);
  }
  else if(DH[3][0] < 0){
    angle_J5 = atan2(- sqrt(1 - R36[2][2]*R36[2][2]), R36[2][2]);
    angle_J4 = atan2(-R36[1][2], -R36[0][2]);
    angle_J6 = atan2(-R36[2][1], R36[2][0]);
  }

  //DH update
  DH[3][0] = ValLim(degrees(angle_J4), limits[3][0], limits[3][1]);
  DH[4][0] = ValLim(degrees(angle_J5), limits[4][0], limits[4][1]);
  DH[5][0] = ValLim(degrees(angle_J6), limits[5][0], limits[5][1]);

  time_d = micros() - time_s;
  Serial.print("Cas ");
  Serial.println(time_d);
}

//Read position value 
void ReadPos()
{
  //Serial.println("RES");
  String mess = Serial.readString();
  //Serial.println(mess);
  Angle_type = mess[0];
  //Serial.println(Angle_type);
  if(Angle_type == 'S' || Angle_type == 's')
  {
  	String ser_h;
  	int i = 1;
  	while(mess[i] != ';')
  	{
  		ser_h = ser_h + mess[i];
  		i++;
  		if(i > 3)
  		{
  			break;
  		}
  	}
  	servo_res = ser_h.toInt();
  }
  else
  {
  	int i = 1;
  int last = 1;
  int end = 1;
  int count = 0;
  String help;
  while (true)
  {
    if(mess[i] == '/')
    {
      help = "";
      for(int y = end; y < last; y++)
      {
        help = help + mess[y];
        //Serial.println(mess[y]);
      }
      //Serial.println(" ");
      //Serial.println(help);
      ResVal[count] = help.toInt();
      count++;
      end = last + 1;
      
    }
    else if(mess[i] == ';' or count == 6)
    {
      
      for(int i = 0; i < 6; i++)
      {
        Serial.println(ResVal[i]);
      }
      
      break;
    }
    last++;
    i++;
  }
  	
  }
}

void test()
{
  for(int i = 1; i < 7; i++){
    digitalWrite(LED1, HIGH);
    switch (i)
  {
  case 1:
    digitalWrite(SL1, LOW);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, HIGH);
    delay(500);
    break;
  case 2:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, LOW);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, HIGH);
    delay(500);
    break;
  case 3:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, LOW);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, HIGH);
    delay(500);
    break;
  case 4:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, LOW);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, HIGH);
    delay(500);
    break;
  case 5:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, LOW);
    digitalWrite(SL6, HIGH);
    delay(500);
    break;
  case 6:
    digitalWrite(SL1, HIGH);
    digitalWrite(SL2, HIGH);
    digitalWrite(SL3, HIGH);
    digitalWrite(SL4, HIGH);
    digitalWrite(SL5, HIGH);
    digitalWrite(SL6, LOW);
    delay(500);
    break;
  default:
    break;
  }
  digitalWrite(LED1, LOW);
  digitalWrite(SL1, HIGH);
  digitalWrite(SL2, HIGH);
  digitalWrite(SL3, HIGH);
  digitalWrite(SL4, HIGH);
  digitalWrite(SL5, HIGH);
  digitalWrite(SL6, HIGH);
  }

  
  

}

void servo(int val)
{
	int ser_val = map(val, 0, 100, servo_open, servo_close);
	servo1.writeMicroseconds(ser_val);
	
}
void setup() {

  pinMode(SL1, OUTPUT);
  pinMode(SL2, OUTPUT);
  pinMode(SL3, OUTPUT);
  pinMode(SL4, OUTPUT);
  pinMode(SL5, OUTPUT);
  pinMode(SL6, OUTPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT); 

  pinMode(SERVOPWM, OUTPUT);
  pinMode(P1, OUTPUT);
  pinMode(P2, OUTPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV128, LSBFIRST, SPI_MODE2));
  
  digitalWrite(SL1, HIGH);
  digitalWrite(SL2, HIGH);
  digitalWrite(SL3, HIGH);
  digitalWrite(SL4, HIGH);
  digitalWrite(SL5, HIGH);
  digitalWrite(SL6, HIGH);

  Serial.begin(9600);
}

void loop() 
{
  digitalWrite(LED3, HIGH);
  delay(500);
  digitalWrite(LED3, LOW);
  delay(500);
  digitalWrite(LED3, HIGH);
  delay(500);
  digitalWrite(LED3, LOW);

  while (true)
  {
    if(Serial.available() > 0)
    {
      	ReadPos();
      	digitalWrite(LED3, HIGH);
      
      	if(Angle_type == 'D' || Angle_type == 'd')
    	{
      		Serial.println("Direct control");
      		for(int i = 0; i < 6; i++)
      		{
        		DH[i][0] = ValLim(ResVal[i], limits[i][0], limits[i][1]);
      		}
      		TransSPI();
    	}
    	else if(Angle_type == 'I' || Angle_type == 'i')
    	{
      		Serial.println("Inverse kinematics");
      		InverseKinematics(ResVal[0], ResVal[1], ResVal[2], ResVal[3], ResVal[4], ResVal[5]);
      		for(int i = 0; i < 6; i++)
      		{
        		Serial.println(DH[i][0]);
      		}
      		TransSPI();
    	}
    	else if(Angle_type == 'S' || Angle_type == 's')
    	{
    		Serial.println("Gripper move");
    		servo(servo_res);
    		if(Angle_type == 's')
    		{
    			Serial.print("Servo position: ");
    			Serial.println(servo_res);
    		}
    	}
    	else
    	{
      		Serial.println("Unknown command!");
    	}
    	//delay(1000);
    	digitalWrite(LED3, LOW);
  	}

  }
    
}
