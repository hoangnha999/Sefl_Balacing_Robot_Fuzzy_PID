#include <IRremote.h>
#include <Wire.h>
#include <FastPID.h>
#include <PID_v1_bc.h>

double input, output;
float DEFAULT_SETPOINT = 1.0;
#define LIMIT 70
#define TILT_ANGLE 1.0
double setpoint = DEFAULT_SETPOINT;
float Kp=15.08, Ki=30.0, Kd=0.55, Hz=5;   
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


double input_velocity, output_targetAngle;
double setPointSpeed = 0.0;
float Ksp=0.35, Ksi=0.01, Ksd=0.039;   
// float Ksp=8.5, Ksi=3.0, Ksd=0.00;   
// 0.025 0.002 0.0015
FastPID speedPID(Ksp, Ksi, Ksd, Hz, 16, true);
// PID speedPID(&input_velocity, &output_targetAngle, &setPointSpeed, Ksp, Ksi, Ksd, DIRECT);

#define PWM_1         9
#define DIR_1         4
#define PWM_2         10
#define DIR_2         13 
#define BRAKE2        12
#define BRAKE         11
#define ENC_1         2
#define ENC_2         3

#define DECODE_LG
#define IRPin    8
const int ledPin = 7;

int ctrM1 = 0;
int ctrM2 = 0;
#define LEFT_CMD 0x07
#define RIGHT_CMD 0x06
#define FORWARD_CMD 0x40
#define BACKWARD_CMD 0x41

#define MPU6050 0x68              // Device address
#define ACCEL_CONFIG 0x1C         // Accelerometer configuration address
#define GYRO_CONFIG 0x1B          // Gyro configuration address
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define _2PI (2 * M_PI)

//Sensor output scaling
#define accSens 0                 // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1                // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

int16_t  AcX, AcZ, GyY, gyroY;

//IMU offset values
int16_t  AcX_offset = 0;
int16_t  AcZ_offset = 0;
int16_t  GyY_offset = 0;

double speed_ctrl = 0;
float robot_angle;
float Acc_angle;            // angle calculated from acc. measurments
float angle_offset = 10.02 - 4.54; //Thay doi gia tri de robot can bang nhat khi dung yen
bool vertical = false; 

#define Gyro_amount 0.996   // percent of gyro in complementary filter

volatile byte pos;
volatile int enc_count = 0;
long currentT, previousT_1, previousT_2 = 0; 


void ENC_READ() {
  byte cur = (!digitalRead(ENC_1) << 1) + !digitalRead(ENC_2);
  byte old = pos & B00000011;
  byte dir = (pos & B00110000) >> 4;
 
  if (cur == 3) cur = 2;
  else if (cur == 2) cur = 3;
 
  if (cur != old) {
  if (dir == 0) {
    if (cur == 1 || cur == 3) dir = cur;
    } else {
      if (cur == 0) {
        if (dir == 1 && old == 3) enc_count--;
        else if (dir == 3 && old == 1) enc_count++;
        dir = 0;
      }
    }
    pos = (dir << 4) + (old << 2) + cur;
  }
}

void Motor1_control(int sp) {
  if (sp > 0) digitalWrite(DIR_1, LOW);
    else digitalWrite(DIR_1, HIGH);
  analogWrite(PWM_1, abs(sp));
}

void Motor2_control(int sp) {
  if (sp > 0) digitalWrite(DIR_2, HIGH);
    else digitalWrite(DIR_2, LOW);
  analogWrite(PWM_2, abs(sp));
}

// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void blink()
{
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 100) {
        previousMillis = currentMillis; // Save the last time the LED was updated
        // Toggle the LED state
        digitalWrite(ledPin, !digitalRead(ledPin));
   }
}

void angle_setup() {
  Wire.begin();
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay(100);
  // int32_t  AcX_offset_sum = 0;
  // int32_t  AcZ_offset_sum = 0;
  // int32_t  GyY_offset_sum = 0;
  // for (int i = 0; i < 1024; i++) {
  //   blink();
  //   angle_calc();
  //   GyY_offset_sum += GyY;
  //   AcX_offset_sum += AcX;
  //   AcZ_offset_sum += AcZ;
  //   delay(3);
  // }
  // GyY_offset = GyY_offset_sum >> 10;
  AcX_offset = -60;
  AcZ_offset = 300;
  // Serial.print("GyY offset value = ");  Serial.println(GyY_offset);

  int i;
  long value = 0;
  float dev;
  int16_t values[100];
  bool gyro_cal_ok = false;
  
  delay(500);
  while (!gyro_cal_ok){
    Serial.println("Gyro calibration... DONT MOVE!");
    // we take 100 measurements in 4 seconds
    for (i = 0; i < 100; i++)
    {
      blink();
      angle_calc();
      values[i] = GyY;
      value += GyY;
      delay(25);
    }
    // mean value
    value = value / 100;
    // calculate the standard deviation
    dev = 0;
    for (i = 0; i < 100; i++)
      dev += (values[i] - value) * (values[i] - value);
    dev = sqrt((1 / 100.0) * dev);
    Serial.print("offset: ");
    Serial.print(value);
    Serial.print("  stddev: ");
    Serial.println(dev);
    if (dev < 50.0)
      gyro_cal_ok = true;
    else
      Serial.println("Repeat, DONT MOVE!");
  }
  GyY_offset = value;
}

void angle_calc() {

   // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                     // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   // request a total of 2 registers
  AcX = Wire.read() << 8 | Wire.read(); 
  //Serial.println(AcX);
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3F);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   // request a total of 2 registers
  AcZ = Wire.read() << 8 | Wire.read(); 
  //Serial.println(AcZ);
  Wire.beginTransmission(MPU6050);
  Wire.write(0x45);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   // request a total of 2 registers
  GyY = Wire.read() << 8 | Wire.read(); 

  // AcX += AcX_offset;
  // AcZ += AcZ_offset;
  GyY += GyY_offset;

  // robot_angle += GyY * loop_time / 1000 / 65.536;    
  robot_angle += GyY * 6.07968E-5;                      
  Acc_angle =  atan2(-AcX, AcZ) * 57.2958 + angle_offset ;    
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);

  int16_t correction = constrain(GyY, GyY_offset - 10, GyY_offset + 10); // limit corrections...
  GyY_offset = GyY_offset * Gyro_amount + correction * (1-Gyro_amount); // Time constant of this correction is around 20 sec.
  
  if (abs(robot_angle) > 40) vertical = false;
  if (abs(robot_angle) < 1) vertical = true;

}

void MPU6050_getAngle(float dt)
{
   // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                     // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   // request a total of 2 registers
  AcX = Wire.read() << 8 | Wire.read(); 
  //Serial.println(AcX);
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3F);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   // request a total of 2 registers
  AcZ = Wire.read() << 8 | Wire.read(); 
  //Serial.println(AcZ);
  Wire.beginTransmission(MPU6050);
  Wire.write(0x45);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   // request a total of 2 registers
  GyY = Wire.read() << 8 | Wire.read(); 

  Acc_angle =  atan2(-AcX, AcZ) * 57.2958 + angle_offset ;
  GyY = (GyY - GyY_offset) / 65.5;  // Accel scale at 500deg/seg  => 65.5 LSB/deg/s

  // Complementary filter
  // We integrate the gyro rate value to obtain the angle in the short term and we take the accelerometer angle with a low pass filter in the long term...
  robot_angle = Gyro_amount * (robot_angle + GyY * dt) + (1.0 - Gyro_amount) * Acc_angle;  // Time constant = 0.99*0.01(100hz)/(1-0.99) = 0.99, around 1 sec.

  // Gyro bias correction
  // We supose that the long term mean of the gyro_value should tend to zero (gyro_offset). This means that the robot is not continuosly rotating.
  int16_t correction = constrain(GyY, GyY_offset - 10, GyY_offset + 10); // limit corrections...
  GyY_offset = GyY_offset * Gyro_amount + correction * (1-Gyro_amount); // Time constant of this correction is around 20 sec.
  
  if (abs(robot_angle) > 40) vertical = false;
  if (abs(robot_angle) < 1) vertical = true;

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // // Set Timer2 to Fast PWM mode
  // TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  // TCCR2B = _BV(CS21); 

  // Pins D9 and D10 - 7.8 kHz
  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00001010; // x8 fast pwm

  pinMode(DIR_1, OUTPUT);  
  pinMode(DIR_2, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(BRAKE2, OUTPUT);
  pinMode(ENC_1, INPUT);
  pinMode(ENC_2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_1), ENC_READ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_2), ENC_READ, CHANGE);

  digitalWrite(BRAKE, LOW);
  digitalWrite(BRAKE2, LOW);
  Motor1_control(0);
  Motor2_control(0);

  IrReceiver.begin(IRPin);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(Hz);
  myPID.SetOutputLimits(-LIMIT, LIMIT); 

  // speedPID.SetMode(AUTOMATIC);
  // speedPID.SetSampleTime(Hz);
  // speedPID.SetOutputLimits(-LIMIT, LIMIT); 
  speedPID.setOutputRange(-LIMIT, LIMIT); 

  Serial.println("Calibrating gyro...");
  angle_setup();  
  digitalWrite(ledPin, HIGH);
}
bool move = false;
bool rotate = false;
void handleIR(int &ctr_mt1, int &ctr_mt2)
{
  if (IrReceiver.decode()) {
       IrReceiver.resume();

      /*
        * Finally, check the received data and perform actions according to the received command
        */
      if (IrReceiver.decodedIRData.command == FORWARD_CMD) {
          ctr_mt1 = 0;
          ctr_mt2 = 0;
          speed_ctrl = 615;
          Serial.println("FORWARD_CMD");
          move = true;
      } else if (IrReceiver.decodedIRData.command == BACKWARD_CMD) {
          ctr_mt1 = 0;
          ctr_mt2 = 0;
          speed_ctrl = -615;
          Serial.println("BACKWARD_CMD");
          move = true;
      }
      else if (IrReceiver.decodedIRData.command == LEFT_CMD) {
          speed_ctrl = 0;
          ctr_mt1 = 0;
          ctr_mt2 = 20;
          Serial.println("LEFT_CMD");
          rotate = true;
      }
      else if (IrReceiver.decodedIRData.command == RIGHT_CMD) {
          speed_ctrl = 0;
          ctr_mt1 = 20;
          ctr_mt2 = 0;
          Serial.println("RIGHT_CMD");
          rotate = true;
      }
      else
      {
        ctr_mt1 = 0;
        ctr_mt2 = 0;
        speed_ctrl = 0;
        rotate = false;
        move = false;
      }
  }
}

void tuning_speed()
{
  // Check for serial input
  if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      
      // Check for set speed command
      if (input.startsWith("P")) {
          Ksp = input.substring(1).toFloat();
      }
      else if (input.startsWith("I")) {
          Ksi = input.substring(1).toFloat();
      }
      else if (input.startsWith("D")) {
          Ksd = input.substring(1).toFloat();
      }
      else if (input.startsWith("A")) {
          DEFAULT_SETPOINT = input.substring(1).toFloat();
      }
      // Check for get speed command
      else if (input.equals("S")) {
          Serial.println("setpoint: " + String(DEFAULT_SETPOINT) + " P: " + String(Ksp) + " I: " + String(Ksi) + " D: " + String(Ksd));
      }
      
      // Handle invalid commands
      else {
          Serial.println("Unknown command.");
      }
      // speedPID.SetTunings (Ksp, Ksi, Ksd);
      speedPID.configure (Ksp, Ksi, Ksd, Hz, 16, true);       
  }
}

void tuning_balance()
{
  // Check for serial input
  if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      
      // Check for set speed command
      if (input.startsWith("P")) {
          Kp = input.substring(1).toFloat();
      }
      else if (input.startsWith("I")) {
          Ki = input.substring(1).toFloat();
      }
      else if (input.startsWith("D")) {
          Kd = input.substring(1).toFloat();
      }
      else if (input.startsWith("A")) {
          DEFAULT_SETPOINT = input.substring(1).toFloat();
      }
      // Check for get speed command
      else if (input.equals("S")) {
          Serial.println("setpoint: " + String(DEFAULT_SETPOINT) + " P: " + String(Kp) + " I: " + String(Ki) + " D: " + String(Kd));
      }
      
      // Handle invalid commands
      else {
          Serial.println("Unknown command.");
      }

      myPID.SetTunings (Kp, Ki, Kd);         
  }
}

void loop() {

  // put your main code here, to run repeatedly:
  // int adcValue = analogRead(A0);  // Read the ADC value (0-1023)
  // int servoValue = map(adcValue, 0, 1023, 1000, 2000);
  // left.writeMicroseconds(servoValue);

  // Serial.print(" -> Servo Value: ");
  // Serial.println(servoValue);
  float now = micros();
  float dt = (now - previousT_2) / 1000000.0f; //in seconds
  previousT_2 = now;

    // angle_calc();
    MPU6050_getAngle(dt);
    handleIR(ctrM1, ctrM2);
    // tuning_balance();
    tuning_speed();

    // Serial.println(robot_angle);
    
    if (now - previousT_1 >= (50000)) 
    {
      input_velocity = -enc_count/0.05;
      enc_count = 0;
      previousT_1 = now;
      output_targetAngle = speedPID.step(speed_ctrl, input_velocity);  
    }

    if(vertical == true)
    {
      input = robot_angle;
      // if(move)
      // {
        setpoint = map(output_targetAngle, -LIMIT, LIMIT, -3.5, 3.5);
      //   // speedPID.configure (10.5, 3.0, Ksd, Hz, 16, true); 
      // }
      // else
      {
        // speedPID.configure (Ksp, Ksi, Ksd, Hz, 16, true); 
        // setpoint = map(output_targetAngle, -LIMIT, LIMIT, -4.5, 4.5);
      }
      myPID.Compute();
      Motor1_control(output + ctrM1);
      Motor2_control(output + ctrM2);

      // Serial.print("input_velocity: ");
      // Serial.print(input_velocity);
      // Serial.print(" output:");
      // Serial.print(setpoint);
      // Serial.print(" robot_angle: ");
      // Serial.print(robot_angle);
      // Serial.print(" output: ");
      // Serial.println(output);
    }
    else
    {
      Motor1_control(0);
      Motor2_control(0);
    }
    setpoint = DEFAULT_SETPOINT;
    
}