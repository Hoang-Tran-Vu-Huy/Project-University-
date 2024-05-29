
//Balance Robot

/////////////////////////////////////////////////////////////////////////////////////////////////
//Thư viện
#include "stmpu6050.h"
SMPU6050 mpu6050;
//             chân ARDUINO   ký hiệu trên          Chân PORT AVR
//                            board Arduino nano    Atmega 328P
#define Enable 4              //D4                  PORTD 4
#define Step_1 5              //D5                  PORTD 5
#define Step_2 6              //D6                  PORTD 6
#define Dir_1  7              //D7                  PORTD 7
#define Dir_2  8              //D8                  PORTB 0

/////////////////////////////////////////////////////////////////////////////////////////////////
//khai báo biến
int8_t Dir_LMotor, Dir_RMotor;                          
volatile int16_t Count_time_LMotor, Count_timer_RMotor;  
volatile int32_t Step_LMotor, Step_RMotor;
int16_t Count_TOP_LMotor, Count_BOT_LMotor, Count_TOP_RMotor, Count_BOT_RMotor;  
float input_MPU;
float input_MPU_last;
float input_LMotor, M_LMotor, MotorL;
float input_RMotor, M_RMotor, MotorR;
float output_MPU;
float I;

float Offset = 0.75;//Offset là độ lệch ban đầu của Robot
float Kp = 15;
float Ki = 0.5;
float Kd = 0.01;

/////////////////////////////////////////////////////////////////////////////////////////////////
//Hàm khai báo các chân Arduino Nano
void Pins_Arduino() 
{
  pinMode(Enable, OUTPUT);
  pinMode(Step_1, OUTPUT);
  pinMode(Step_2, OUTPUT);
  pinMode(Dir_1, OUTPUT);
  pinMode(Dir_2, OUTPUT);
  digitalWrite(Enable, LOW);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//Hàm khai báo Timer
void timer_INI() 
{
  TCCR2A = 0;              
  TCCR2B = 0;               
  TCCR2B |= (1 << CS21);    
  OCR2A = 39;               
  TCCR2A |= (1 << WGM21);   
  TIMSK2 |= (1 << OCIE2A);  
}

//////////////////////////////////////////////////////////////////////////////////////////////
//Chương trình ngắt của Timer
ISR(TIMER2_COMPA_vect) 
{
  //Tạo xung STEP cho Motor ben trai
  if (Dir_LMotor != 0) 
  {
    Count_time_LMotor++;
    if (Count_time_LMotor <= Count_TOP_LMotor)
      PORTD |= 0b00100000; 
    else
      PORTD &= 0b11011111;
    if (Count_time_LMotor > Count_BOT_LMotor) 
    {
      Count_time_LMotor = 0;
      if (Dir_LMotor > 0)
        Step_LMotor++;
      else if (Dir_LMotor < 0)
        Step_LMotor--;
    }
  }

  //Tạo xung STEP cho Motor ben phai
  if (Dir_RMotor != 0)
  {
    Count_timer_RMotor++;
    if (Count_timer_RMotor <= Count_TOP_RMotor)
      PORTD |= 0b01000000;
    else
      PORTD &= 0b10111111;
    if (Count_timer_RMotor > Count_BOT_RMotor) 
    {
      Count_timer_RMotor = 0;
      if (Dir_RMotor > 0)
        Step_RMotor++;
      else if (Dir_RMotor < 0)
        Step_RMotor--;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
//Hàm tốc độ di chuyển của Motor bên trái
void Speed_LMotor(int16_t x)  // cùng kiểu dữ liệu count_bot
{
  if (x < 0) {
    Dir_LMotor = -1;
    PORTD &= 0b01111111;
  } 
  else if (x > 0) {
    Dir_LMotor = 1;
    PORTD |= 0b10000000;
  } 
  else Dir_LMotor = 0;

  Count_BOT_LMotor = abs(x);  //lấy giá trị tuyệt đối
  Count_TOP_LMotor = Count_BOT_LMotor / 2;
}

//Hàm tốc độ di chuyển của Motor bên Phải
void Speed_RMotor(int16_t x) 
{
  if (x < 0) {
    Dir_RMotor = -1;
    PORTB &= 0b11111110;
  } 
  else if (x > 0) {
    Dir_RMotor = 1;
    PORTB |= 0b00000001;
  } 
  else Dir_RMotor = 0;

  Count_BOT_RMotor = abs(x);
  Count_TOP_RMotor = Count_BOT_RMotor / 2;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  mpu6050.init(0x68);
  Serial.begin(9600);
  Pins_Arduino();
  timer_INI();
}

///////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{
  float AngleX = mpu6050.getXAngle();  //mặt trước là mpu (số dương), mặt sau là adruino (số âm)
  //delay(500);
  //Speed_LMotor(50); // giá trị càng nhỏ thì tốc độ quay càng lớn
  //Speed_RMotor(50);
  //Serial.println(AngleX);
  //delay(100);

  //Dùng PID 
  input_MPU = AngleX + Offset;  
  I += Ki*input_MPU;
  I = constrain(I, -400, 400);
  
  output_MPU = Kp * input_MPU + I + Kd * (input_MPU - input_MPU_last);
  input_MPU_last = input_MPU;

  //khống chế Output theo sự phi tuyến của Motor22222
  if (output_MPU > -5 && output_MPU < 5) 
  {
    output_MPU = 0;
  }
  output_MPU = constrain(output_MPU, -400, 400);

  input_LMotor = output_MPU;
  input_RMotor = output_MPU;

  //Khắc phục sự phi tuyến của Motor
  if (output_MPU > 0) {
    M_LMotor = 405 - (1 / (input_LMotor + 9)) * 5500;  
    M_RMotor = 405 - (1 / (input_RMotor + 9)) * 5500;  
  } else if (output_MPU < 0) {                         
    M_LMotor = -405 - (1 / (input_LMotor - 9)) * 5500; 
    M_RMotor = -405 - (1 / (input_RMotor - 9)) * 5500;
  } else {
    M_LMotor = 0;
    M_RMotor = 0;
  }

  // điều chỉnh làm ngược giá trị truyền vào hàm Speed_LMotor
  if (M_LMotor > 0) {
    MotorL = 400 - M_LMotor;
  } else if (M_LMotor < 0) {
    MotorL = -400 - M_LMotor;
  } else {
    MotorL = 0;
  }

  // điều chỉnh làm ngược giá trị truyền vào hàm Speed_RMotor
  if (M_RMotor > 0) {
    MotorR = 400 - M_RMotor;
  } else if (M_RMotor < 0) {
    MotorR = -400 - M_RMotor;
  } else {
    MotorR = 0;
  }

  //2 Motor chạy
  Speed_LMotor(MotorL);
  Speed_RMotor(MotorR);
  //sendtopc(I, MotorL, MotorR);
}

//hàm gửi data lên máy tính
void sendtopc(float a, float b, float c)
{
  Serial.print(a);
  Serial.print(",");
  Serial.print(b);
  Serial.print(",");
  Serial.println(c);
  //delay(1);
}