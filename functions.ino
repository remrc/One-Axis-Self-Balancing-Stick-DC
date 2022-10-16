void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

// ---------- from Simple FOC library --------
float getSensorAngle() {
  // raw data from the sensor
  int cpr = max_raw_count - min_raw_count;
  int raw_count = analogRead(sensor);  
  return ((float) (raw_count) / (float)cpr) * _2PI;
}

float getVelocity() {
  float val = getSensorAngle();
  angle_prev_ts = micros();
  float d_angle = val - angle_prev;
  // if overflow happened track it as full rotation
  if (abs(d_angle) > (0.8f * _2PI)) full_rotations += (d_angle > 0) ? -1 : 1; 
  angle_prev = val;
    
  // calculate sample time
  float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;

  velocity = ((float)(full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev) ) / Ts;
  vel_angle_prev = angle_prev;
  vel_full_rotations = full_rotations;
  vel_angle_prev_ts = angle_prev_ts;
  return velocity;
}
// -----------------------------------------

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
  Wire.requestFrom(MPU6050, 4, true);  
  AcX = Wire.read() << 8 | Wire.read(); 
  AcY = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  
  GyZ = Wire.read() << 8 | Wire.read(); 

  AcX -= AcX_offset;
  AcY -= AcY_offset;  
  GyZ -= GyZ_offset;

  robot_angle += GyZ * loop_time / 1000 / 65.536;
  Acc_angle = atan2(AcY, -AcX) * 57.2958;         // angle from acc. values * 57.2958 (deg/rad)
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);
  if (abs(robot_angle) > 6) vertical = false;
  if (abs(robot_angle) < 0.3) vertical = true;
}

void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); //debug
  if (voltage > 8 && voltage <= 9.5) {
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
     case 'p':
      if (cmd == '+')    pGain += 1;
      if (cmd == '-')    pGain -= 1;
      printPIDValues();
      break;
    case 'i':
      if (cmd == '+')    iGain += 0.5;
      if (cmd == '-')    iGain -= 0.5;
      printPIDValues();
      break;
    case 'a':
      if (cmd == '+')    aGain += 1;
      if (cmd == '-')    aGain -= 1;
      printPIDValues();
      break;    
    case 's':
      if (cmd == '+')    sGain += 0.005;
      if (cmd == '-')    sGain -= 0.005;
      printPIDValues();
      break;  
  }
}

void printPIDValues() {
  Serial.print("P: "); Serial.print(pGain);
  Serial.print(" I: "); Serial.print(iGain);
  Serial.print(" A: "); Serial.print(aGain, 3);
  Serial.print(" S: "); Serial.println(sGain, 3);
}


