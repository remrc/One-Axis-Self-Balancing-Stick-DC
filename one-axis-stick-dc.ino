#include <Wire.h>

#define MPU6050       0x68     // Device address
#define ACCEL_CONFIG  0x1C     // Accelerometer configuration address
#define GYRO_CONFIG   0x1B     // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C

#define   sensor      A3
#define   _2PI        6.28318530718f

#define   InA_m1      8        // INA right motor pin 
#define   InB_m1      10       // INB right motor pin
#define   m1PWM       5        // PWM right motor pin

#define   BUZZER     12
#define   VBAT       A7

#define   accSens   0          // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define   gyroSens  1          // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define   Gyro_amount 0.996    // percent of gyro in complementary filter

float pGain = 175.0;  
float iGain = 16.0;   
float aGain = 13.0;   
float sGain = 0.04;   
float loop_time = 8; 

int pwm;
int32_t motor_speed;  
float motor_speed_enc;
uint32_t timer;
long currentT, previousT_1, previousT_2 = 0; 
int16_t AcX, AcY, AcZ, GyZ, gyroZ;

float angle_prev = 0.0f;        // result of last call to getSensorAngle(), used for full rotations and velocity
float velocity = 0.0f;
long angle_prev_ts = 0;         // timestamp of last call to getAngle, used for velocity
float vel_angle_prev = 0.0f;    // angle at last call to getVelocity, used for velocity
long vel_angle_prev_ts = 0;     // last velocity calculation timestamp
int32_t full_rotations = 0;     // full rotation tracking
int32_t vel_full_rotations = 0; // previous full rotation value for velocity calculation
int max_raw_count = 1; 
int min_raw_count = 1023;

//IMU offset values
int16_t  AcX_offset = 1730;
int16_t  AcY_offset = 410;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;

float alpha = 0.40; // low pass 
float gyroZfilt;

float robot_angle;
float Acc_angle;

bool vertical = false; 

uint8_t i2cData[14];

void setup() {
  Serial.begin(115200);

  pinMode(InA_m1, OUTPUT); 
  pinMode(InB_m1, OUTPUT); 
  pinMode(m1PWM, OUTPUT);  
  
  digitalWrite(BUZZER, LOW);
  pinMode(BUZZER, OUTPUT);

  delay(2000);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  angle_setup();
}

void loop() {

  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning(); 
    angle_calc();
    if (vertical) {
      gyroZ = GyZ / 131.0; // Convert to deg/s
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      pwm = constrain(pGain * robot_angle + iGain * gyroZfilt + aGain * motor_speed_enc + sGain * motor_speed, -255, 255);
      motor_speed_enc = getVelocity();
      Motor_control(pwm);
      motor_speed += motor_speed_enc;
    } else {
      Motor_control(0);
      motor_speed = 0;
    }
    previousT_1 = currentT;
  }
  if (currentT - previousT_2 >= 1000) {
    battVoltage((double)analogRead(VBAT) / 74); 
    previousT_2 = currentT;
  }
}

