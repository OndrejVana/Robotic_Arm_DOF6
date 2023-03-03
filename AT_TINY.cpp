#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>

#define DIR 1
#define PUL 2
#define LED 3

#define S_speed 500 
#define F_speed 5000
#define kp 5

#define MOSI_bp 1
#define MOSI_bm (1 << MOSI_bp)
#define MISO_bp 2
#define MISO_bm (1 << MISO_bp)
#define SCK_bp 3
#define SCK_bm (1 << SCK_bp)
#define SS_bp 4
#define SS_bm (1 << SS_bp)

#define DUMMY 0xFF

int adrr = 0x36;
int raw_ang_hi = 0x0c;
int raw_ang_lo = 0x0d;
int ang_hi = 0x0e;
int ang_lo = 0x0f;

int Ldata = 0;

volatile uint8_t receiveData = 0;

volatile uint8_t writeData = 0;


void SPI0_init(void)
{
	/*
 PORTA.DIR &= ~MOSI_bm; 
 PORTA.DIR |= MISO_bm; 
 PORTA.DIR &= ~SCK_bm; 
 PORTA.DIR &= ~PIN7_bm; 
 */
 PORTA.DIRSET = MISO_bm;
 
  SPI0.CTRLB =
        SPI_MODE_2_gc; // Mode2
 
 SPI0.CTRLA = SPI_DORD_bm /* LSB is transmitted first */
 | SPI_ENABLE_bm /* Enable module */
 & (~SPI_MASTER_bm); /* SPI module in Slave mode */
 SPI0.INTCTRL = SPI_IE_bm; /* SPI Interrupt enable */
}

ISR(SPI0_INT_vect)
{
 receiveData = SPI0.DATA;
 writeData = receiveData; 
 Ldata = receiveData;
 SPI0.DATA = writeData;
 SPI0.INTFLAGS = SPI_IF_bm; /* Clear the Interrupt flag by writing 1 */
}

void setup() 
{
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(LED, OUTPUT);
}

void cos_delay(int time)
{
  int time_s = millis();
  while (true)
  {
    if(millis() - time_s > time)
    {
      return;
    }
  }

}

bool find_dir(double val_r, double val_s)
{
  if(val_r > val_s)
  {
    return 0;
  }
  else
  {
    return 1; 
  }
}

void step(double set_angle, double real_angle)
{
  int wait_time = abs(set_angle - real_angle) * kp;
  wait_time = F_speed - wait_time;
  wait_time = constrain(wait_time, S_speed, F_speed);
  digitalWrite(DIR, find_dir(real_angle, set_angle));
  digitalWrite(PUL, HIGH);
  cos_delay(wait_time);
  digitalWrite(PUL, LOW);
}

double re_cal_angle(double angle_new, double angle_last)
{
  double diff = angle_new - angle_last;
  if(diff > 180)
  {
    double dif1 = 360 - angle_last;
    diff = dif1 + angle_new; 
  }
  else if(diff < -180)
  {
    double dif1 = 0 - angle_last;
    diff =  dif1 - angle_new;
  }
  return angle_last + diff;
}

double ReadAngle()
{
  word retVal = -1;

  //Read Low Byte
  Wire.beginTransmission(adrr);
  Wire.write(ang_lo);
  Wire.endTransmission();
  Wire.requestFrom(adrr, 1);
  while(Wire.available() == 0);
  word low = Wire.read();
 
  // Read High Byte  
  Wire.beginTransmission(adrr);
  Wire.write(ang_hi);
  Wire.endTransmission();
  Wire.requestFrom(adrr, 1);
  while(Wire.available() == 0);
  word high = Wire.read();
  
  high = high << 8;
  retVal = high | low;
  
  return retVal;
}

void startup()
{
  for(int i = 0; i < 3; i++)
  {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}

void loop() 
{
  SPI0_init();
 	sei(); //global interrups
  startup();
  while(true)
  {
    step(Ldata, ReadAngle());
  }
}
