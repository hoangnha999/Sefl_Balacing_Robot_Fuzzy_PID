#include <Wire.h>   
#include <PID_v1.h> 
#include <math.h>   
#include <Encoder.h> 

#define MPU6050 0x68    // Địa chỉ IMU

// Được sử dụng để đưa IMU ra khỏi chế độ ngủ
#define PWR_MGMT_1 0x6B   

// Forward declarations - Khai báo trước các hàm
void startup();
void imu_setup();
void init_angle_PID();
void init_velocity_PID();
void gyro_calibration();
void angle_calc();
void balance();
void velocity_calculations();
void move_profile();
void motorsOff();
void leftMotorSpeed(byte dir, int pwm);
void rightMotorSpeed(byte dir, int pwm);
void read_MPU6050();   

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


#define GYRO_CONFIG 0x1B

// Các thanh ghi cảm biến con quay hồi chuyển 
#define GYRO_XOUT_15_8 0x43
#define GYRO_XOUT_7_0 0x44
#define GYRO_YOUT_15_8 0x45
#define GYRO_YOUT_7_0 0x46
#define GYRO_ZOUT_15_8 0x47
#define GYRO_ZOUT_7_0 0x48

 
#define TEMP_OUT_15_8 0x41
#define TEMP_OUT_7_0 0x42

// Chân động cơ trái
#define  pwmPin_1  6
#define  dirPin_1  7
#define  left_motor_output_a  2
#define left_motor_output_b  4

// Chân động cơ phải
#define dirPin_2  8
#define pwmPin_2  9
#define right_motor_output_a  3
#define right_motor_output_b  5
  
const int  left_minimum_motor_speed = 0;
const int  right_minimum_motor_speed = 10;

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

double velocity_setpoint,smoothed_velocity_setpoint, velocity_input, velocity_output, velocity_error; // double giống float cho atmega 2560 nhưng thư viện PID chỉ chấp nhận double 

/* Giá trị điều chỉnh */
double velocity_Kp, velocity_Ki, velocity_Kd; // phải lớn hơn 0

/* Khởi tạo đối tượng PID */
PID velocity_PID(&velocity_input, &velocity_output, &smoothed_velocity_setpoint, velocity_Kp, velocity_Ki, velocity_Kd, DIRECT); 

unsigned int velocity_loopcount = 0;
unsigned int velocity_timer = 0;


void setup() {
Serial.begin(9600);
startup();    // Khởi tạo Robot


imu_setup();    // Khởi tạo IMU

init_angle_PID();    // Khởi tạo PID góc
init_velocity_PID(); // Khởi tạo PID vận tốc

gyro_calibration();

looptime = micros();    // bắt đầu theo dõi thời gian bắt đầu vòng lặp


}

void loop() {
  angle_calc();                                                                                                  // tính toán góc của robot

  balance();  
  //Serial.println(angle_pitch_output);                                                                                                  // Chạy quy trình cân bằng bao gồm các vòng lặp PID góc và vận tốc và logic liên quan
      
  while(micros() - looptime < 4000);                                                                             // Đảm bảo vòng lặp chạy đều đặn ở 4ms trước khi tiếp tục, 4000 micro giây = 4 ms. Vòng lặp nên mất <4ms bất kể giai đoạn hoạt động của robot, nếu không có thể xảy ra vấn đề.
  looptime = micros(); 

}



void balance(){

  if (first_up == true){                    // nếu robot chưa được đẩy lên,
    angle_PID.SetMode(MANUAL);              // Đảm bảo cả hai tính toán PID đều tắt và không tích lũy lỗi khi robot bị nghiêng
    velocity_PID.SetMode(MANUAL);
    velocity_output = 0;
    angle_output = 0;                       // đặt giá trị "angle_output" về 0
  
    if(angle_pitch_output >-20 && angle_pitch_output <20 ){    // nếu góc gần bằng không
       first_up = false;                                       // thoát khỏi vòng lặp khởi tạo robot
      }
    else{   
       motorsOff();                                            // Nếu robot chỉ nằm ngửa, giữ động cơ tắt
  }
    }
     else if(first_up == false){                         // nếu robot đã được đẩy lên, chạy tất cả các chức năng cân bằng cần thiết.

      move_profile();                                    // chương trình con để di chuyển robot tiến và lùi
    
     
    angle_input = angle_pitch_output;                   // lấy giá trị đầu vào cho PID góc sử dụng đầu ra từ hàm angle_calc();
   angle_PID.Compute();                                 // Tính toán kết quả của PID, cho chúng ta đầu ra động cơ mới
  angle_error = (angle_setpoint-angle_input);           // Tính toán lỗi để quyết định xem robot đã ngã quá xa hay chưa, và hướng nào để chạy động cơ. Thuật toán PID tự nó "biết" hướng nào để đi, nhưng trình điều khiển động cơ không biết nên chúng ta phải quyết định cho chúng một cách nào đó  

      if (abs(angle_error) < 40){                       // Nếu Robot chưa ngã quá xa, tức là nó đang ở vị trí cân bằng ổn định. 
        angle_PID.SetMode(AUTOMATIC);                   // bắt đầu tính toán PID và bắt đầu di chuyển động cơ để cân bằng robot
        
        if(angle_output < 0){                            // Chọn hướng động cơ dựa trên đầu ra PID góc
          angle_output = int(angle_output);             // Giá trị PWM chỉ có thể được viết dưới dạng số nguyên
          leftMotorSpeed(LOW, abs(angle_output));       // đi "lùi" nếu robot nghiêng về phía sau
          rightMotorSpeed(LOW, abs(angle_output));
             }
             
        else if(angle_output > 0){
          angle_output = int(angle_output);              // Giá trị PWM chỉ có thể được viết dưới dạng số nguyên.
          leftMotorSpeed(HIGH, abs(angle_output));      // đi "tiến" nếu robot nghiêng về phía trước
          rightMotorSpeed(HIGH, abs(angle_output));
        }
        
  }   else {                                            // Nếu robot đã ngã / vượt quá góc cho phép
        angle_PID.SetMode(MANUAL);                     // Đảm bảo robot ngã không chạy lên I term, tắt PID 
        motorsOff();                                   // Tắt động cơ
        angle_output = 0;                              // Đặt lại giá trị đầu ra góc
        first_up = true;                               // đặt lại cờ "first up" của robot
  }
   
    
// Tính toán vận tốc 
     velocity_PID.SetMode(AUTOMATIC);                              // Đảm bảo PID đang bật
     velocity_calculations();
     velocity_input = filtered_velocity_average;                   // đầu vào vào bộ điều khiển PID là vận tốc trung bình của hai bánh xe. 
     velocity_error = velocity_setpoint - velocity_input;          // tính toán lỗi giữa vận tốc mong muốn và hiện tại
     velocity_PID.Compute();                                       // Tính toán vòng lặp PID. 
     angle_setpoint = velocity_output;                             // đặt giá trị mong muốn góc mới là đầu ra âm của vòng lặp PID vận tốc.      
       }
}

void velocity_calculations(){
 
   x_final_right = right_encoder.read();                                                          //check encoder position
    if (x_final_right != x_initial_right){                                                        // check to make sure new encoder value is not the same as previous one 
      velocity_right = ((x_final_right - x_initial_right)*.0227)/(.004 * time_multiple_right) ;    //delta x just change in time in seconds between encoder readings
      x_initial_right = x_final_right;
      time_multiple_right = 1;                                                                    //if this bit of code has been run reset delta t to be multiplied by 1
    
    }else{
      time_multiple_right++;
    }

    x_final_left = left_encoder.read();
     if (x_final_left != x_initial_right){                                                      // check to make sure new encoder value is not the same as previous one
       velocity_left = ((x_final_left - x_initial_left)*.0227)/(.004 * time_multiple_left);      //calculate the final velocities for left and right encoders. formula is: v = delta x / delta t. delta x is distance between two encoder readings, with .0114 being distance between single encoder "tick" roughly. Derived from 1920 CPR and diameter of wheels
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
   angle_Kp = 48;
   angle_Ki = 170;
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


void startup(){               
  pinMode(dirPin_1, OUTPUT);
  pinMode(pwmPin_1, OUTPUT);
  pinMode(dirPin_2, OUTPUT);
  pinMode(pwmPin_2, OUTPUT);
}



void init_angle_PID(){

  angle_input = 0;
  angle_setpoint = 0;   
 // angle_PID.SetMode(AUTOMATIC);
  angle_PID.SetSampleTime(4);   // Tính toán PID mỗi 4 ms, mặc định thư viện là 200ms, quá lâu
  angle_PID.SetOutputLimits(-255,255);    // Đặt phạm vi đầu ra của vòng lặp PID
  set_angle_PID();  // Ghi và đặt giá trị PID góc
  
}

void init_velocity_PID(){

  velocity_input = 0;
  velocity_setpoint = 0;   // Giá trị mong muốn
  smoothed_velocity_setpoint = 0;
  velocity_PID.SetMode(AUTOMATIC);
  velocity_PID.SetSampleTime(4);   // Tính toán PID mỗi 4 ms, mặc định thư viện là 200ms, quá lâu
  velocity_PID.SetOutputLimits(-100,100);    // Đặt phạm vi đầu ra của vòng lặp PID
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

void gyro_calibration(){    // Hàm hiệu chỉnh con quay hồi chuyển

for(int cycle_count = 0; cycle_count < 2000; cycle_count++){    // Vòng lặp này lấy 2000 giá trị đọc từ các trục x, y, và z của con quay hồi chuyển
  read_MPU6050();                       // Lấy giá trị thô từ MPU6050  
  gyro_x_cal += gyro_x;               // Thêm giá trị thô mới vào tổng giá trị con quay hồi chuyển 
  gyro_y_cal += gyro_y;
  gyro_z_cal += gyro_z;
  delay(4);                // thêm độ trễ 3 micro giây để mô phỏng vòng lặp 250Hz.
}
gyro_x_cal /= 2000;   // chia cho 2000 để lấy giá trị trung bình hiệu chỉnh
gyro_y_cal /= 2000;
gyro_z_cal /= 2000;

}

 
void read_MPU6050(){
  Wire.beginTransmission(MPU6050);   // Thiết lập IMU làm slave 
  Wire.write(ACCEL_XOUT_15_8);    // Gửi thanh ghi bắt đầu được yêu cầu
  Wire.endTransmission();
  Wire.requestFrom(MPU6050,14);  // yêu cầu đọc 14 byte dữ liệu, không ngắt kết nối. Điều này sẽ tự động gọi các thanh ghi tiếp theo sau ACCEL_XOUT_15_8 trong "14" chu kỳ, vì vậy dữ liệu của 14 thanh ghi tiếp theo sẽ được đọc và đặt vào bộ đệm để gửi. Trong trường hợp này, điều này thuận tiện vì 14 thanh ghi tiếp theo đều là thanh ghi dữ liệu cho gia tốc kế và con quay hồi chuyển 
  while(Wire.available() < 14);  
  accel_x = (Wire.read()<<8) | Wire.read();    // Đọc dữ liệu theo kiểu Burst, int16_t xử lý bù 2 được xuất bởi IMU. Chúng ta cần đọc tất cả dữ liệu cùng một lúc để đảm bảo các giá trị đọc được từ cùng một thời điểm
                                                     // Chúng ta đang đọc tuần tự 14 byte được yêu cầu ở đầu vòng lặp, và kết hợp hai thanh ghi lại để lấy giá trị 16 bit đầy đủ cho mỗi trục 
  accel_y = (Wire.read()<<8) | Wire.read();    // Trục Y
  accel_z = (Wire.read()<<8) | Wire.read();    // Trục Z
 

  gyro_x = (Wire.read()<<8) | Wire.read();
  gyro_y = (Wire.read()<<8) | Wire.read();
  gyro_z= (Wire.read()<<8) | Wire.read();

 
}

void angle_calc(){   // Hàm tính toán góc, sử dụng giá trị từ gia tốc kế và con quay hồi chuyển

read_MPU6050();

gyro_x -= gyro_x_cal;   // Trừ giá trị hiệu chỉnh con quay hồi chuyển khỏi giá trị hiện tại. 
gyro_y -= gyro_y_cal;
gyro_z -= gyro_z_cal;

angle_pitch += gyro_x * .0000611;   // Tính toán góc đã di chuyển trong khoảng thời gian 4ms vừa qua và cộng vào giá trị góc nghiêng

//0.000001066 = 0.0000611 * (3.142(PI) / 180 độ) Hàm sin của Arduino sử dụng đơn vị radian
angle_pitch += angle_roll * sin(gyro_z * 0.000001066);   // Nếu IMU đã quay, chuyển góc cuộn sang góc nghiêng 

// Tính toán góc từ gia tốc kế:
acc_total_vector = sqrt((accel_x*accel_x) + (accel_y*accel_y) + (accel_z*accel_z));   // Tính toán tổng vector gia tốc (Vector trọng lực)
//57.296 = 1 / (3.142 / 180) Hàm asin của Arduino sử dụng đơn vị radian
acc_angle_pitch = asin((float)accel_y/acc_total_vector)* 57.296;       // Tính toán góc nghiêng

// Đặt MPU-6050 ở mức cân bằng và ghi lại các giá trị trong hai dòng sau để hiệu chỉnh
  acc_angle_pitch -= 3;                                              // Giá trị hiệu chỉnh gia tốc kế cho góc nghiêng , -.5

if(set_gyro_angles){                                                 // Nếu IMU đã được khởi động
    // Bộ lọc bổ sung
    angle_pitch = angle_pitch * 0.9996 + acc_angle_pitch * 0.0004;     // Sửa lỗi trôi của góc nghiêng con quay hồi chuyển bằng góc nghiêng gia tốc kế
    
  }
  else{                                                                // Lần khởi động đầu tiên, đặt giá trị góc nghiêng và cuộn của con quay hồi chuyển bằng giá trị của gia tốc kế để sửa lỗi địa hình không bằng phẳng
    angle_pitch = acc_angle_pitch;                                     // Đặt góc nghiêng con quay hồi chuyển bằng góc nghiêng gia tốc kế 
    set_gyro_angles = true;                                            // Đặt cờ IMU đã khởi động
  }


angle_pitch_output = angle_pitch; 


}



