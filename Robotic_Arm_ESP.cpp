#include <Arduino.h>
#include <math.h>

// Global variables
double DH[6][4] = {{0.01, -90, 220, 3.5},
                  {-90, 0, 0, 500}, 
                  {0.01, 90, 0, 0}, 
                  {0.01 , -90, 360, 0}, 
                  {0.01, 90, 0, 0}, 
                  {0.01, 0, 44, 0}};
int Q;
double time_d = 0;

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
  DH[0][0] = angle_J1;
  DH[1][0] = degrees(angle_J2);
  DH[2][0] = degrees(angle_J3);

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
  DH[3][0] = degrees(angle_J4);
  DH[4][0] = degrees(angle_J5);
  DH[5][0] = degrees(angle_J6);

  time_d = micros() - time_s;
}

void setup() {
  Serial.begin(9600);

}

void loop() {
  InverseKinematics(200, 200, 300, 10, 20, 30);

  for(int i = 0; i < 6; i++)
  {
    Serial.print("osa ");
    Serial.print(i +1);
    Serial.print(" ");
    Serial.println(DH[i][0]);
  }
  Serial.print("cas: ");
  Serial.println(time_d);
  Serial.println("----------------");

delay(5000);
}