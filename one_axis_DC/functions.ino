void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay (5);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(80);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
}

void angle_calc() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);  // request a total of 4 registers
  AcX = Wire.read() << 8 | Wire.read(); 
  AcY = Wire.read() << 8 | Wire.read(); 

  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  
  GyZ = Wire.read() << 8 | Wire.read(); 

  AcXc = AcX - offsets.X;
  AcYc = AcY - offsets.Y;    
  GyZ -= GyZ_offset;

  robot_angle += GyZ * loop_time / 1000 / 65.536; 
  Acc_angle = atan2(AcYc, -AcXc) * 57.2958;
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);
  
  if (abs(robot_angle) > 6) vertical = false;
  if (abs(robot_angle) < 0.3) vertical = true;
}

void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); //debug
  if ((voltage > 5.5 && voltage < 6.4) || (voltage > 8.8 && voltage < 9.5)) {   // 2S and 3S
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void Motor_control(int pwm) { 
    if (pwm <= 0) {
      digitalWrite(InA_m1, LOW);                        
      digitalWrite(InB_m1, HIGH);
    } else {
      digitalWrite(InA_m1, HIGH);                       
      digitalWrite(InB_m1, LOW);
    } 
    analogWrite(m1PWM, abs(pwm)); 
}

int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'i':
      if (cmd == '+')    K2Gain += 0.5;
      if (cmd == '-')    K2Gain -= 0.5;
      printValues();
      break;
    case 'p':
      if (cmd == '+')    K1Gain += 1;
      if (cmd == '-')    K1Gain -= 1;
      printValues();
      break;
    case 's':
      if (cmd == '+')    K4Gain += 1;
      if (cmd == '-')    K4Gain -= 1;
      printValues();
      break;    
    case 'a':
      if (cmd == '+')    K3Gain += 0.005;
      if (cmd == '-')    K3Gain -= 0.005;
      printValues();
      break;  
    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
         Serial.println("calibrating on");
      }
      if (cmd == '-' && calibrating)  {
        Serial.println("calibrating off");
        Serial.print("X: "); Serial.print(AcX + 16384); Serial.print(" Y: "); Serial.println(AcY);
        if (abs(AcY) < 3000) {
          offsets.ID = 78;
          offsets.X = AcX + 16384;
          offsets.Y = AcY;
          digitalWrite(BUZZER, HIGH);
          delay(70);
          digitalWrite(BUZZER, LOW);
          EEPROM.put(0, offsets);
          calibrating = false;
          calibrated = true;
        } else {
          Serial.println("The angle are wrong!!!");
          calibrating = false;
          digitalWrite(BUZZER, HIGH);
          delay(50);
          digitalWrite(BUZZER, LOW);
          delay(70);
          digitalWrite(BUZZER, HIGH);
          delay(50);
          digitalWrite(BUZZER, LOW);
        }
      }
      break;              
  }
}

void printValues() {
  Serial.print("K1: "); Serial.print(K1Gain);
  Serial.print(" K2: "); Serial.print(K2Gain);
  Serial.print(" K3: "); Serial.println(K3Gain, 3);
  Serial.print(" K4: "); Serial.println(K4Gain);
}


