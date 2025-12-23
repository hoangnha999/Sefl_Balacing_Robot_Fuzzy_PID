#include <Wire.h>   
#include <PID_v1.h> 
#include <math.h>   
#include <Encoder.h> 

#define MPU6050 0x68    // Địa chỉ IMU

// Được sử dụng để đưa IMU ra khỏi chế độ ngủ
#define PWR_MGMT_1 0x6B   

// BIẾN TOÀN CỤC

long accel_x, accel_y, accel_z, acc_total_vector; // Các giá trị gia tốc
int16_t gyro_x, gyro_y, gyro_z; // Các giá trị con quay hồi chuyển
long gyro_x_cal, gyro_y_cal, gyro_z_cal; // Giá trị hiệu chỉnh con quay hồi chuyển
int16_t tempReading; // Giá trị nhiệt độ
float angle_pitch, angle_roll, acc_angle_pitch, acc_angle_roll; // Góc nghiêng và góc cuộn
float angle_pitch_output, angle_roll_output; // Giá trị đầu ra góc nghiêng và góc cuộn
unsigned long looptime; // Thời gian vòng lặp
boolean set_gyro_angles; // Cờ thiết lập góc con quay hồi chuyển
boolean first_up = true; // Cờ kiểm tra lần đầu tiên

// CÁC THIẾT LẬP GIA TỐC KẾ

#define ACCEL_CONFIG 0x1C   

#define ACCEL_XOUT_15_8 0x3B   
#define ACCEL_YOUT_15_8 0x3D
#define ACCEL_YOUT_7_0 0x3E
#define ACCEL_ZOUT_15_8 0x3F
#define ACCEL_ZOUT_7_0 0x40

// CÁC THIẾT LẬP CON QUAY HỒI CHUYỂN

#define GYRO_CONFIG 0x1B

// Các thanh ghi cảm biến con quay hồi chuyển 
#define GYRO_XOUT_15_8 0x43
#define GYRO_XOUT_7_0 0x44
#define GYRO_YOUT_15_8 0x45
#define GYRO_YOUT_7_0 0x46
#define GYRO_ZOUT_15_8 0x47
#define GYRO_ZOUT_7_0 0x48

// Các thanh ghi cảm biến nhiệt độ. 
#define TEMP_OUT_15_8 0x41
#define TEMP_OUT_7_0 0x42

// Chân động cơ trái
#define  pwmPin_1 = 6;   
#define  dirPin_1 = 7; 
#define  left_motor_output_a = 2; 
#define left_motor_output_b = 4; 

// Chân động cơ phải
#define dirPin_2 = 8;   
#define pwmPin_2 = 9; 
#define right_motor_output_a = 3; 
#define right_motor_output_b = 5; 
  
#define left_minimum_motor_speed = 0;
#define right_minimum_motor_speed = 4;

// CÁC THIẾT LẬP ENCODER 
Encoder left_encoder(left_motor_output_a, left_motor_output_b);   
Encoder right_encoder(right_motor_output_a, right_motor_output_b); 

long x_final_left = 0;
long x_final_right = 0;
long x_initial_left = 0;
long x_initial_right = 0;

float velocity_left = 0;
float velocity_right = 0;
float average_velocity = 0;
float filtered_velocity_left = 0;
float filtered_velocity_right = 0;
float filtered_velocity_average = 0;

unsigned int time_multiple_right = 1;
unsigned int time_multiple_left = 1;

boolean first_pass = true;

unsigned int loopcount = 0;


// Điều khiển góc PID 
double angle_setpoint, angle_input, angle_output, angle_error;  

/* Giá trị điều chỉnh */
double angle_Kp, angle_Ki, angle_Kd; // phải lớn hơn 0

/* Khởi tạo đối tượng PID */
PID angle_PID(&angle_input, &angle_output, &angle_setpoint, angle_Kp, angle_Ki, angle_Kd, DIRECT);

// Điều khiển vận tốc PID 

/* Định nghĩa các biến kết nối. Setpoint = giá trị mong muốn, Input = giá trị đầu vào PID, Output = giá trị đầu ra để đạt được setpoint.*/
double velocity_setpoint,smoothed_velocity_setpoint, velocity_input, velocity_output, velocity_error; // double giống float cho atmega 2560 nhưng thư viện PID chỉ chấp nhận double 

/* Giá trị điều chỉnh */
double velocity_Kp, velocity_Ki, velocity_Kd; // phải lớn hơn 0

/* Khởi tạo đối tượng PID */
PID velocity_PID(&velocity_input, &velocity_output, &smoothed_velocity_setpoint, velocity_Kp, velocity_Ki, velocity_Kd, DIRECT); 

unsigned int velocity_loopcount = 0;
unsigned int velocity_timer = 0;


void setup() {
delay(500);  //Delay to give user a chance to get robot on its back before gyro calibration begins

startup();    //Initialize Robot

pinMode(13, OUTPUT); // Set status LED as output

imu_setup();    //Initialize IMU

init_angle_PID();    //Initialize angle PID
init_velocity_PID(); //Initialize velocity PID

digitalWrite(13, LOW); // turn led off while calibrating
gyro_calibration();
digitalWrite(13, HIGH); //Turn led on after calibration is complete 

looptime = micros();    //start keeping track of when the loop starts


}

void loop() {
  angle_calc();                                                                                                  //calculate the angle of the robot

  balance();                                                                                                     // Run the balancing routine which consists of the angle and velocity PID loops and associated logic
      
  //Serial.println(float((micros() - looptime))/1000);                                                           //code that prints out the length of time the loop is taking in milliseconds 
  while(micros() - looptime < 4000);                                                                             //Make sure the loop runs consistently at 4ms before preceding, 4000 microseconds = 4 ms. Loop should take <4ms no matter what phase of operation robot is in, otherwise there could be issues.
  looptime = micros(); 

}



void balance(){

  if (first_up == true){                    //if the robot has not been pushed up yet,
    angle_PID.SetMode(MANUAL);              //Make sure both PID calculations are off and not accumulating error while robot is tipped over
    velocity_PID.SetMode(MANUAL);
    velocity_output = 0;
    angle_output = 0;                       // zero out "angle_output" value
  
    if(angle_pitch_output >-1 && angle_pitch_output <1 ){    //if angle is close to zero
       first_up = false;                                       //break robot from initialization loop
      }
    else{   
       motorsOff();                                            //If robot is just laying on its back keep the motors off
  }
    }
     else if(first_up == false){                         //if robot has been pushed up, run all the necessary balancing functions.

      move_profile();                                    //subroutine to move robot forward and backwards
     //tune_PID_w_pots(1, 100, 100, 100);

     
    angle_input = angle_pitch_output;                   //get input value for angle PID using the output from the angle_calc(); function
   angle_PID.Compute();                                 //Compute the result of PID which gives us our new motor Output
  angle_error = (angle_setpoint-angle_input);           //Calculate the error to decide whether the robot has fallen over to far or not, and what direction to run motors. PID algorithm on its own "knows" which direction to go, but the motor drivers don't know so we have to decide for them one way or another  

      if (abs(angle_error) < 40){                       //If Robot hasn't fallen too far over, ie it is in a stable balancing position. 
        angle_PID.SetMode(AUTOMATIC);                   // start PID calcs and start moving motors to balance robot
        
        if(angle_output < 0){                            //Choose Direction of motor based on angle_PID output
          angle_output = int(angle_output);             // PWM values can only be written as whole integer numbers
          leftMotorSpeed(LOW, abs(angle_output));       // go " backwards" if robot leans backwards
          rightMotorSpeed(LOW, abs(angle_output));
             }
             
        else if(angle_output > 0){
          angle_output = int(angle_output);              //PWM values can only be written as whole integer numbers.
          leftMotorSpeed(HIGH, abs(angle_output));      //go "forwards" if robot leans "forwards"
          rightMotorSpeed(HIGH, abs(angle_output));
        }
        
  }   else {                                            //If robot has fallen over / gone past allowable angle
        angle_PID.SetMode(MANUAL);                     //Make sure robot fallen over doesn't run up I term, turn off PID 
        motorsOff();                                   //Turn off motors
        angle_output = 0;                              //Reset angle_output values
        first_up = true;                               //reset robot "first up" flag
  }
   
    
//Velocity Calculation Stuff 
     velocity_PID.SetMode(AUTOMATIC);                              //Make sure PID is on
     velocity_calculations();
     velocity_input = filtered_velocity_average;                   //the input into the PID controller is the average velocity of the two wheels. 
     velocity_error = velocity_setpoint - velocity_input;          // calculate the error between the desired and current velocity
     velocity_PID.Compute();                                       //Compute the PID loop. 
     angle_setpoint = velocity_output;                             //make the new angle setpoint the negative output of the velocity PID loop.      
       }
}

void velocity_calculations(){
 
   x_final_right = right_encoder.read();                                                          //check encoder position
    if (x_final_right != x_initial_right){                                                        // check to make sure new encoder value is not the same as previous one 
      velocity_right = ((x_final_right - x_initial_right)*.0114)/(.004 * time_multiple_right) ;    //delta x just change in time in seconds between encoder readings
      x_initial_right = x_final_right;
      time_multiple_right = 1;                                                                    //if this bit of code has been run reset delta t to be multiplied by 1
    
    }else{
      time_multiple_right++;
    }

    x_final_left = left_encoder.read();
     if (x_final_left != x_initial_right){                                                      // check to make sure new encoder value is not the same as previous one
       velocity_left = ((x_final_left - x_initial_left)*.0114)/(.004 * time_multiple_left);      //calculate the final velocities for left and right encoders. formula is: v = delta x / delta t. delta x is distance between two encoder readings, with .0114 being distance between single encoder "tick" roughly. Derived from 1920 CPR and diameter of wheels
       x_initial_left = x_final_left;                                                          //set final values to the initial values so when program loops around in 4ms the current "final" becomes the "initial" values and we fetch new final values
       time_multiple_left = 1;                                                                 //if this bit of code has been run reset delta t to be multiplied by 1
     
     
     
     } 
     else{
      time_multiple_left++;
     }
                                                                         
   filtered_velocity_left = (filtered_velocity_left * .95) + (velocity_left * .05);            //filter the velocity output with a low and high pass filter to get a smooth transition from value to value
   filtered_velocity_right = (filtered_velocity_right * .95) + (velocity_right * .05);
   filtered_velocity_average = (filtered_velocity_left + (filtered_velocity_right * -1) ) /2 ;
 

  
  
}

void readout_velocity_calulation_data(){                                                       //Trouble Shooting Function
  Serial.print(filtered_velocity_left);
   Serial.print(" ");
   Serial.print(filtered_velocity_right *-1);
   Serial.print(" ");
   Serial.println(filtered_velocity_average); 
}
void move_profile(){                                                                          //This function makes the robot move forward and backward a little bit.
  
  if(velocity_timer >= 750 && velocity_timer <= 1000){
    velocity_setpoint = 50; 
  smoothed_velocity_setpoint = (smoothed_velocity_setpoint * .98) + (velocity_setpoint * .02);
  }
  else if ( velocity_timer >= 1750 && velocity_timer <= 2000 ){     //from 13 seconds to 18 seconds drive backwards
  velocity_setpoint = -50;  
  smoothed_velocity_setpoint = (smoothed_velocity_setpoint * .98) + (velocity_setpoint * .02);
  }
  else if (velocity_timer > 2750){  // restart at 22 seconds
  velocity_timer = 0;
  smoothed_velocity_setpoint = (smoothed_velocity_setpoint * .98) + (velocity_setpoint * .02);
 }
 else {
  velocity_setpoint = 0;
  smoothed_velocity_setpoint = (smoothed_velocity_setpoint * .98) + (velocity_setpoint * .02);
 }



velocity_timer++;

}

void set_angle_PID(){
   angle_Kp = 28;
   angle_Ki = 13;
   angle_Kd = .35;   //Keep angle_Kd relatively small compared to the other values
   angle_PID.SetTunings(angle_Kp,angle_Ki,angle_Kd);
}

void set_velocity_PID(){
  velocity_Kp = .15;    
  velocity_Ki = .08;   
  velocity_Kd = 0;    
  velocity_PID.SetTunings(velocity_Kp,velocity_Ki,velocity_Kd);
}


// Hàm điều khiển hướng và tốc độ động cơ trái
void leftMotorSpeed(byte dir, int pwm){    //dir nên là HIGH hoặc LOW, điều khiển hướng. pwm từ 0-255, điều khiển tốc độ động cơ. 
  if(pwm > 255){
    pwm = 255;
  }
  else if (pwm < left_minimum_motor_speed){
   pwm = left_minimum_motor_speed;
  }
  else{
  digitalWrite(dirPin_1, dir);    
  analogWrite(pwmPin_1, pwm);
  }
}

// Hàm điều khiển tốc độ và hướng động cơ phải
void rightMotorSpeed(byte dir, int pwm){
   if(pwm > 255){
    pwm = 255;
  }
 else if (pwm < right_minimum_motor_speed){
    pwm = right_minimum_motor_speed;
  }
  else{
  digitalWrite(dirPin_2, dir);
  analogWrite(pwmPin_2, pwm);
  }
}

// Hàm tắt động cơ
void motorsOff(){
  digitalWrite(dirPin_1, LOW);
  analogWrite(pwmPin_1,LOW);
  digitalWrite(dirPin_2, LOW);
  analogWrite(pwmPin_2,LOW);
}


void startup(){                 // Khởi tạo robot
  
  Serial.begin(115200);
  pinMode(dirPin_1, OUTPUT);
  pinMode(pwmPin_1, OUTPUT);
  pinMode(dirPin_2, OUTPUT);
  pinMode(pwmPin_2, OUTPUT);
  pinMode(13, OUTPUT);        // Đặt đèn LED trạng thái làm đầu ra
}



void init_angle_PID(){
  
  // Khởi tạo các biến liên kết
 //float a;
  angle_input = 0;
  angle_setpoint = 0;   // Giá trị mong muốn
 // angle_PID.SetMode(AUTOMATIC);
  angle_PID.SetSampleTime(4);   // Tính toán PID mỗi 4 ms, mặc định thư viện là 200ms, quá lâu
  angle_PID.SetOutputLimits(-255,255);    // Đặt phạm vi đầu ra của vòng lặp PID
  set_angle_PID();  // Ghi và đặt giá trị PID góc
  
}

void init_velocity_PID(){
  // Khởi tạo các biến liên kết
// float a;
  velocity_input = 0;
  velocity_setpoint = 0;   // Giá trị mong muốn
  smoothed_velocity_setpoint = 0;
  velocity_PID.SetMode(AUTOMATIC);
  velocity_PID.SetSampleTime(4);   // Tính toán PID mỗi 4 ms, mặc định thư viện là 200ms, quá lâu
  velocity_PID.SetOutputLimits(-1000,1000);    // Đặt phạm vi đầu ra của vòng lặp PID
  set_velocity_PID(); // Ghi và đặt giá trị PID vận tốc
  
}



void imu_setup(){ // Khởi tạo gia tốc kế, con quay hồi chuyển; đặt các bit đúng trên IMU
  Wire.begin(MPU6050);                // Thiết lập IMU làm slave
  
  Wire.beginTransmission(MPU6050);    // Gửi địa chỉ IMU
  Wire.write(PWR_MGMT_1);             // Cần ghi gì đó vào vector quản lý nguồn để có giá trị sử dụng được, và bật IMU
  Wire.write(0x00);                   // Đặt 0 vào bộ đệm, không quan trọng đặt gì ở đây
  Wire.endTransmission();             // Gửi 0 đến thanh ghi quản lý nguồn, bật IMU và nhận giá trị có thể đọc được  
  
  Wire.beginTransmission(MPU6050);    // Đặt các bit cấu hình của MPU6050
  Wire.write(ACCEL_CONFIG);           // Chọn bit hiệu chuẩn gia tốc kế
  Wire.write(0x10);                   // Ghi cài đặt phạm vi gia tốc kế, +/- 8g
  Wire.endTransmission();             // Gửi cài đặt phạm vi gia tốc kế

  Wire.beginTransmission(MPU6050);    // Địa chỉ MPU6050
  Wire.write(GYRO_CONFIG);            // Thanh ghi địa chỉ con quay hồi chuyển
  Wire.write(0x08);                   // Thanh ghi tỷ lệ nhạy cảm con quay hồi chuyển, +/- 500 độ/s
  Wire.endTransmission();



}

void gyro_calibration(){    //Calibration Class

for(int cycle_count = 0; cycle_count < 2000; cycle_count++){    //This loop takes 2000 readings of the gyro x,y, and z axes
  read_MPU6050();                       //Grab raw MPU6050 values  
  gyro_x_cal += gyro_x;               //Add new raw value to total gyro value 
  gyro_y_cal += gyro_y;
  gyro_z_cal += gyro_z;
  delay(4);                //add 3us delay to simulate 250hz loop.
}
gyro_x_cal /= 2000;   //divide by 2000 to get average calibration values
gyro_y_cal /= 2000;
gyro_z_cal /= 2000;

}

 
void read_MPU6050(){
  Wire.beginTransmission(MPU6050);   //Set up IMU as a slave 
  Wire.write(ACCEL_XOUT_15_8);    //Send requested starting register
  Wire.endTransmission();
  Wire.requestFrom(MPU6050,14);  //request 14 bytes of data to be read, don't kill connection. This will automatically call up the next registers after ACCEL_XOUT_15_8 for "14" cycles, so that the 14 next sequential registers' data                                       //is read and placed into the buffer to be sent. In this case that is convienent because the next 14 registers are all data registers for the accelerometer and gyroscope 
  while(Wire.available() < 14);  
  accel_x = (Wire.read()<<8) | Wire.read();    //Burst Read Data, int16_t handles the 2's complement which is output by the IMU. We need to read all the data at once to ensure our readings are from the same instance in time
                                                     //We are sequentially reading out the 14 bytes requested at the beginning of the loop, and combining two registers together to get the full 16 bit value for each axis 
  accel_y = (Wire.read()<<8) | Wire.read();    //Y axis
  accel_z = (Wire.read()<<8) | Wire.read();    //Z axis
 
  tempReading = (Wire.read()<<8) | Wire.read();    

  gyro_x = (Wire.read()<<8) | Wire.read();
  gyro_y = (Wire.read()<<8) | Wire.read();
  gyro_z= (Wire.read()<<8) | Wire.read();

 
}

void angle_calc(){   // pass the accel and gyro addresses.

read_MPU6050();

gyro_x -= gyro_x_cal;   // Subtract the gyro calibration value from the current gyro value. 
gyro_y -= gyro_y_cal;
gyro_z -= gyro_z_cal;

angle_pitch += gyro_x * .0000611;   //Calculate the angle traveled over the last 4ms period and add it to the angle_pitch value
angle_roll += gyro_y * .0000611;    //Calculate the angle traveled over the last 4ms period and add it to the angle_roll value

//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
angle_pitch += angle_roll * sin(gyro_z * 0.000001066);   //If the IMU has yawed transfer the roll angle to the pitch angle 
angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);   //If the IMU has yawed transfer the pitch angle to the roll angle

//Accelerometer Angle Calculations:
acc_total_vector = sqrt((accel_x*accel_x) + (accel_y*accel_y) + (accel_z*accel_z));   //Calculate the total Accelerometer vector (Gravity Vector)
//57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
acc_angle_pitch = asin((float)accel_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
acc_angle_roll = asin((float)accel_x/acc_total_vector)* -57.296;       //Calculate the roll angle

//Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  acc_angle_pitch -= 0;                                              //Accelerometer calibration value for pitch , -.5
  acc_angle_roll -= 0;                                               //Accelerometer calibration value for roll, -3

if(set_gyro_angles){                                                 //If the IMU is already started
    //Complimentary Filter
    angle_pitch = angle_pitch * 0.9996 + acc_angle_pitch * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + acc_angle_roll * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    
  }
  else{                                                                //At first start, set gyro pitch and roll values to gyro values to correct for uneven terrain
    angle_pitch = acc_angle_pitch;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = acc_angle_roll;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }


angle_pitch_output = angle_pitch; 
angle_roll_output = angle_roll;  


}




