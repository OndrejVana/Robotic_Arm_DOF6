#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>

// pin defines
#define DIRE 2
#define PUL 1
#define LED 3

// user defines
#define S_speed 600 
#define F_speed 5000
#define SenTimeFrq 100
#define I2Ctime 100

//SPI pin defines
#define MOSI_bp 1
#define MOSI_bm (1 << MOSI_bp)
#define MISO_bp 2
#define MISO_bm (1 << MISO_bp)
#define SCK_bp 3
#define SCK_bm (1 << SCK_bp)
#define SS_bp 4
#define SS_bm (1 << SS_bp)

// Mainloop variables
double SenVal;
double LastAng;
long long SenTime;

// I2C addresses 
int adrr = 0x36;
int raw_ang_hi = 0x0c;
int raw_ang_lo = 0x0d;
int ang_hi = 0x0e;
int ang_lo = 0x0f;

// Sensor reading variables
word high;
int low;		
word retVal = -1;
int val;


// SPI reading variables
uint8_t receiveData = 0;
uint8_t receiveData_L = 0;
uint8_t receiveData_H = 0;
uint8_t writeData = 0;
uint8_t writeData_L = 0;
uint8_t writeData_H = 0;
int b_count = 1;
int Ldata = 0;

// P regulator variables
int wait_time;
int kp = 1;

// SPI setup
void SPI0_init(void)
{
 PORTA.DIRSET = MISO_bm;
 
  SPI0.CTRLB =
        SPI_MODE_2_gc; // Mode2
 
 SPI0.CTRLA = SPI_DORD_bm /* LSB is transmitted first */
 | SPI_ENABLE_bm /* Enable module */
 & (~SPI_MASTER_bm); /* SPI module in Slave mode */
 SPI0.INTCTRL = SPI_IE_bm; /* SPI Interrupt enable */
}

// SPI interrupt
ISR(SPI0_INT_vect)
{
  receiveData = SPI0.DATA;
  writeData = receiveData; 
  SPI0.DATA = writeData;
 if(b_count == 1)
 {
 	receiveData_L = receiveData;
 	b_count++;
 }
 else if(b_count == 2)
 {
 	receiveData_H = receiveData;
 	Ldata = 0;
 	Ldata = receiveData_H << 8;
 	Ldata = Ldata | receiveData_L;
 	b_count--;
 }
 
 
 SPI0.INTFLAGS = SPI_IF_bm; /* Clear the Interrupt flag by writing 1 */
}

void setup() 
{
  pinMode(DIRE, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(LED, OUTPUT);

  Wire.begin();
}

// solving the direction of turnig 
bool find_dir(double val_r, double val_s)
{
  if(val_r > val_s)
  {
    return 1;
  }
  else
  {
    return 0; 
  }
}

// stepping a motor with P regulator 
void stepp(double set_angle, double real_angle)
{
  wait_time = abs(set_angle - real_angle) * kp;
  wait_time = F_speed - wait_time;
  wait_time = constrain(wait_time, S_speed, F_speed);
  digitalWrite(DIRE, find_dir(real_angle, set_angle));
  digitalWrite(PUL, HIGH);
  delayMicroseconds(wait_time);
  digitalWrite(PUL, LOW);
  delayMicroseconds(wait_time);
  digitalWrite(PUL, HIGH);
  delayMicroseconds(wait_time);
  digitalWrite(PUL, LOW);
  delayMicroseconds(wait_time);
}

// solving an angle of multi rotation system
double re_cal_angle(double scaled_angle ,double angle_new, double angle_last)
{
  double diff = angle_new - angle_last;
  if(diff > 180)
  {
    diff = angle_last + (360 - angle_new); 
  }
  else if(diff < -180)
  {
    diff =  angle_last - (360 - angle_new);
  }
  return scaled_angle + diff;
}

// reading a value of angle 
double ReadAngle()
{
  int timeW;
  digitalWrite(LED, HIGH);
 
  //Read Low Byte
  Wire.beginTransmission(adrr);
  Wire.write(raw_ang_lo);
  Wire.endTransmission();
  Wire.requestFrom(adrr, 1);
  timeW = millis();
  while(Wire.available() == 0)
  {
  	if((millis() - timeW) > I2Ctime)
  	{
  		break;
  	}
  }
  low = Wire.read();
 
  // Read High Byte  
  Wire.beginTransmission(adrr);
  Wire.write(ang_hi);
  Wire.endTransmission();
  Wire.requestFrom(adrr, 1);
  timeW = millis();
  while(Wire.available() == 0)
  {
  	if((millis() - timeW) > I2Ctime)
  	{
  		break;
  	}
  }
  high = Wire.read();
  //combine two bytes
  high = high << 8;
  retVal = high | low;
  int newAng = 0.087 * retVal;

  digitalWrite(LED, LOW);
  
  return newAng;
}

// startup LED sequence 
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
  startup(); //start diode indicator
  SenTime = millis();
  //SenVal = ReadAngle(); //first angle readings
  LastAng = ReadAngle();
  while(true)
  {
  	if((millis() - SenTime) > SenTimeFrq)
  	{
  		SenTime = millis();
  		//SenVal = ReadAngle();
  		SenVal = re_cal_angle(SenVal, ReadAngle(), LastAng); //angle reading
  		LastAng = ReadAngle();
  	}
  	stepp(Ldata, SenVal); 
  }
}
