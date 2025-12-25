#include <Wire.h>   
#include <math.h>   
#include <TimerOne.h> 

#define MPU6050 0x68    // Địa chỉ IMU

// Được sử dụng để đưa IMU ra khỏi chế độ ngủ
#define PWR_MGMT_1 0x6B   

// Forward declarations - Khai báo trước các hàm
void startup();
void imu_setup();
void init_angle_PID();
void gyro_calibration();
void kalman_init();
float kalman_update(float newAngle, float newRate, float dt);
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

// ===== KALMAN FILTER CHO GÓC PITCH =====
/* Kalman filter 1D cho góc pitch
 * State vector: [angle, bias]^T
 * - angle: góc pitch ước lượng
 * - bias: bias của gyroscope
 */
float kalman_angle = 0.0;      // Góc ước lượng từ Kalman
float kalman_bias = 0.0;       // Bias gyro ước lượng
float kalman_rate = 0.0;       // Tốc độ góc không có bias

// Ma trận hiệp phương sai lỗi P (2x2)
float P[2][2] = {{0, 0}, {0, 0}};

// Nhiễu process (Q matrix)
float Q_angle = 0.001;    // Độ tin cậy vào process model cho góc
float Q_bias = 0.003;     // Độ tin cậy vào process model cho bias

// Nhiễu measurement (R matrix)
float R_measure = 0.03;   // Độ tin cậy vào phép đo từ accelerometer

// Biến tạm cho tính toán
float kalman_y;      // Innovation (sai số đo)
float kalman_S;      // Innovation covariance
float kalman_K[2];   // Kalman gain
const float dt = 0.010;  // Chu kỳ lấy mẫu: 10ms

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

// CÁC THIẾT LẬP ENCODER - Sử dụng interrupt
volatile long encoder_pulse_left = 0;   // Đếm xung encoder trái
volatile long encoder_pulse_right = 0;  // Đếm xung encoder phải

float velocity_left = 0;     // Tốc độ bánh trái (pulse/s)
float velocity_right = 0;    // Tốc độ bánh phải (pulse/s)
float average_velocity = 0;  // Tốc độ trung bình

const int PULSES_PER_REV = 200;  // Số xung trên 1 vòng encoder (điều chỉnh theo encoder của bạn)
const float WHEEL_DIAMETER = 0.065; // Đường kính bánh xe (m)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159; // Chu vi bánh xe

boolean first_pass = true;
unsigned int loopcount = 0;


// Điều khiển góc PID - RỜI RẠC (Discrete PID)
double angle_setpoint = 0;
double angle_input = 0;
double angle_output = 0;
double angle_last_output = 0;
double angle_error = 0;
double angle_error_prev1 = 0;  // E(k-1)
double angle_error_prev2 = 0;  // E(k-2)

/* Tham số PID góc */
double angle_Kp = 48.0;
double angle_Ki = 170.0;
double angle_Kd = 0.35;

/* Hệ số PID rời rạc - Velocity form */
double angle_alpha, angle_beta, angle_gamma;

// Điều khiển vận tốc
double velocity_setpoint = 0;
double smoothed_velocity_setpoint = 0;
double velocity_input = 0;

/* Chu kỳ lấy mẫu PID */
const float T_SAMPLE = 0.010; // 10ms = 0.010s 

unsigned int velocity_loopcount = 0;
unsigned int velocity_timer = 0;

// ===== ENCODER INTERRUPT HANDLERS =====
// ISR cho encoder trái (chân 2)
void leftEncoderISR() {
  if(digitalRead(left_motor_output_b) == LOW)
    encoder_pulse_left++;
  else
    encoder_pulse_left--;
}

// ISR cho encoder phải (chân 3)
void rightEncoderISR() {
  if(digitalRead(right_motor_output_b) == LOW)
    encoder_pulse_right++;
  else
    encoder_pulse_right--;
}

// ===== HÀM ĐIỀU KHIỂN HOÀN CHỈNH (GỌI BỞI TIMER 10ms) =====
void PID_Control() {
  // ===== BƯỚC 1: ĐỌC IMU VÀ TÍNH GÓC KALMAN =====
  read_MPU6050();
  
  // Trừ giá trị hiệu chỉnh gyro
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  
  // Chuyển đổi gyro_x từ raw value sang độ/giây (±500°/s: 65.5 LSB/(°/s))
  float gyro_rate = (float)gyro_x / 65.5;
  
  // Tính góc từ accelerometer
  acc_total_vector = sqrt((accel_x*accel_x) + (accel_y*accel_y) + (accel_z*accel_z));
  if(acc_total_vector < 100) acc_total_vector = 100;
  acc_angle_pitch = asin((float)accel_y / acc_total_vector) * 57.296;
  acc_angle_pitch -= 3.0;  // Hiệu chỉnh offset
  
  // Khởi tạo lần đầu
  if(!set_gyro_angles){
    kalman_angle = acc_angle_pitch;
    angle_pitch = acc_angle_pitch;
    set_gyro_angles = true;
    kalman_init();
  }
  else{
    // Cập nhật Kalman filter
    kalman_angle = kalman_update(acc_angle_pitch, gyro_rate, dt);
    angle_pitch = kalman_angle;
  }
  angle_pitch_output = angle_pitch;
  
  // ===== BƯỚC 2: TÍNH VẬN TỐC TỪ ENCODER =====
  velocity_left = (encoder_pulse_left / (float)PULSES_PER_REV) * (1.0 / T_SAMPLE) * 60.0;
  velocity_right = (encoder_pulse_right / (float)PULSES_PER_REV) * (1.0 / T_SAMPLE) * 60.0;
  encoder_pulse_left = 0;
  encoder_pulse_right = 0;
  average_velocity = (velocity_left - velocity_right) / 2.0;
  
  // ===== BƯỚC 3: KIỂM TRA TRẠNG THÁI ROBOT =====
  if (first_up == true){
    // Robot chưa đứng lên - reset tất cả
    angle_output = 0;
    angle_error = 0;
    angle_error_prev1 = 0;
    angle_error_prev2 = 0;
    angle_last_output = 0;
    
    // Kiểm tra góc có gần thẳng đứng không
    if(angle_pitch_output > -20 && angle_pitch_output < 20){
      first_up = false;  // Bắt đầu cân bằng
    }
    else{
      motorsOff();
      return;  // Thoát, không tính PID
    }
  }
  
  // ===== BƯỚC 4: TÍNH ANGLE SETPOINT TỪ VẬN TỐC =====
  double vel_to_angle_gain = 0.10;
  angle_setpoint = smoothed_velocity_setpoint * vel_to_angle_gain;
  
  // ===== BƯỚC 5: TÍNH PID =====
  angle_input = angle_pitch_output;
  angle_error = angle_setpoint - angle_input;
  
  // Kiểm tra robot có ngã không
  if (abs(angle_error) > 40){
    // Robot đã ngã - tắt động cơ và reset
    motorsOff();
    angle_output = 0;
    angle_error = 0;
    angle_error_prev1 = 0;
    angle_error_prev2 = 0;
    angle_last_output = 0;
    first_up = true;
    return;
  }
  
  // Tính hệ số PID rời rạc (Velocity form)
  angle_alpha = 2.0 * T_SAMPLE * angle_Kp + angle_Ki * T_SAMPLE * T_SAMPLE + 2.0 * angle_Kd;
  angle_beta = T_SAMPLE * T_SAMPLE * angle_Ki - 4.0 * angle_Kd - 2.0 * T_SAMPLE * angle_Kp;
  angle_gamma = 2.0 * angle_Kd;
  
  // PID Velocity form
  angle_output = (angle_alpha * angle_error + angle_beta * angle_error_prev1 + 
                  angle_gamma * angle_error_prev2 + 2.0 * T_SAMPLE * angle_last_output) / (2.0 * T_SAMPLE);
  
  // Giới hạn output
  if(angle_output > 255) angle_output = 255;
  if(angle_output < -255) angle_output = -255;
  
  // Cập nhật lịch sử
  angle_last_output = angle_output;
  angle_error_prev2 = angle_error_prev1;
  angle_error_prev1 = angle_error;
  
  // ===== BƯỚC 6: XUẤT PWM ĐỘNG CƠ =====
  if(angle_output < 0){
    // Robot nghiêng ra sau → đi lùi
    leftMotorSpeed(LOW, abs((int)angle_output));
    rightMotorSpeed(LOW, abs((int)angle_output));
  }
  else if(angle_output > 0){
    // Robot nghiêng ra trước → đi tiến
    leftMotorSpeed(HIGH, abs((int)angle_output));
    rightMotorSpeed(HIGH, abs((int)angle_output));
  }
  else{
    // Output = 0
    motorsOff();
  }
}


void setup() {
  Serial.begin(9600);
  startup();    // Khởi tạo Robot
  
  // Thiết lập encoder interrupts
  pinMode(left_motor_output_a, INPUT_PULLUP);
  pinMode(left_motor_output_b, INPUT_PULLUP);
  pinMode(right_motor_output_a, INPUT_PULLUP);
  pinMode(right_motor_output_b, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(left_motor_output_a), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(right_motor_output_a), rightEncoderISR, FALLING);
  
  imu_setup();    // Khởi tạo IMU
  gyro_calibration();
  
  // Khởi tạo PID rời rạc
  init_angle_PID();
  
  // Thiết lập Timer1 để gọi PID mỗi 10ms (10000 microseconds)
  Timer1.initialize(10000);
  Timer1.attachInterrupt(PID_Control);
  
  looptime = micros();    // bắt đầu theo dõi thời gian bắt đầu vòng lặp
}

void loop() {
  // ===== CHỈ XỬ LÝ LOGIC CẤP CAO =====
  move_profile();   // Cập nhật velocity setpoint
  
  // ===== DEBUG: IN GIÁ TRỊ RA SERIAL =====
  static unsigned long last_print = 0;
  if(millis() - last_print > 100){  // Print mỗi 100ms
    last_print = millis();
    
    Serial.print("Angle:");
    Serial.print(angle_pitch_output);
    Serial.print(" | Error:");
    Serial.print(angle_error);
    Serial.print(" | PWM:");
    Serial.print((int)angle_output);
    Serial.print(" | Vel:");
    Serial.print(average_velocity);
    Serial.print(" | Status:");
    Serial.println(first_up ? "WAIT" : "BALANCE");
  }
  
  delay(1);  // Giảm tải CPU
}



// Hàm balance() đã được gộp vào PID_Control() ISR

void move_profile(){
  /* Tạo profile chuyển động tự động
   * Gọi mỗi 1ms trong loop() → velocity_timer đếm từng ms
   * Timeline:
   *   0-750ms: Đứng yên (v=0)
   *   750-1000ms: Tiến (v=50)
   *   1000-1750ms: Đứng yên
   *   1750-2000ms: Lùi (v=-50)
   *   2000-2750ms: Đứng yên
   *   >2750ms: Reset và lặp lại
   */
  
  if(velocity_timer >= 750 && velocity_timer <= 1000){
    velocity_setpoint = 50; 
    smoothed_velocity_setpoint = (smoothed_velocity_setpoint * 0.98) + (velocity_setpoint * 0.02);
  }
  else if (velocity_timer >= 1750 && velocity_timer <= 2000){
    velocity_setpoint = -50;  
    smoothed_velocity_setpoint = (smoothed_velocity_setpoint * 0.98) + (velocity_setpoint * 0.02);
  }
  else if (velocity_timer > 2750){
    velocity_timer = 0;  // Reset timer
    velocity_setpoint = 0;
    smoothed_velocity_setpoint = (smoothed_velocity_setpoint * 0.98) + (velocity_setpoint * 0.02);
  }
  else {
    velocity_setpoint = 0;
    smoothed_velocity_setpoint = (smoothed_velocity_setpoint * 0.98) + (velocity_setpoint * 0.02);
  }

  velocity_timer++;  // Tăng 1ms mỗi lần gọi
}

void set_angle_PID(){
   // Các giá trị đã được đặt trong khai báo biến toàn cục
   // angle_Kp = 48.0;
   // angle_Ki = 170.0;
   // angle_Kd = 0.35;
}

// Không cần set_velocity_PID nữa vì không dùng velocity PID

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


// ===== KALMAN FILTER FUNCTIONS =====

void kalman_init(){
  /* Khởi tạo ma trận hiệp phương sai P
   * P = [1 0]
   *     [0 1]
   */
  P[0][0] = 1.0;
  P[0][1] = 0.0;
  P[1][0] = 0.0;
  P[1][1] = 1.0;
  
  kalman_angle = 0.0;
  kalman_bias = 0.0;
}

float kalman_update(float newAngle, float newRate, float dt){
  /* Kalman Filter cho góc pitch
   * Sử dụng công thức chuẩn của Kalman Filter 1D
   * 
   * State: x = [angle, bias]^T
   * Measurement: z = angle từ accelerometer
   * Process model: 
   *   angle(k) = angle(k-1) + dt * (rate - bias)
   *   bias(k) = bias(k-1)
   * 
   * Measurement model:
   *   z = angle + noise
   */
  
  // ===== BƯỚC 1: PREDICT (Dự đoán) =====
  // Cập nhật state estimate
  kalman_rate = newRate - kalman_bias;           // Tốc độ góc đã loại bỏ bias
  kalman_angle += dt * kalman_rate;              // angle(k|k-1) = angle(k-1|k-1) + dt * rate
  
  // Cập nhật ma trận hiệp phương sai P
  // P(k|k-1) = A * P(k-1|k-1) * A^T + Q
  // Với A = [1  -dt]
  //         [0   1 ]
  
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  // ===== BƯỚC 2: UPDATE (Cập nhật với measurement) =====
  // Tính innovation (sai số giữa đo và dự đoán)
  kalman_y = newAngle - kalman_angle;           // y = z - H*x
  
  // Tính innovation covariance
  kalman_S = P[0][0] + R_measure;               // S = H*P*H^T + R
  
  // Tính Kalman gain
  kalman_K[0] = P[0][0] / kalman_S;             // K = P*H^T / S
  kalman_K[1] = P[1][0] / kalman_S;
  
  // Cập nhật state estimate với measurement
  kalman_angle += kalman_K[0] * kalman_y;       // x = x + K*y
  kalman_bias += kalman_K[1] * kalman_y;
  
  // Cập nhật ma trận hiệp phương sai P
  // P = (I - K*H) * P
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  
  P[0][0] -= kalman_K[0] * P00_temp;
  P[0][1] -= kalman_K[0] * P01_temp;
  P[1][0] -= kalman_K[1] * P00_temp;
  P[1][1] -= kalman_K[1] * P01_temp;
  
  // Trả về góc đã được ước lượng
  return kalman_angle;
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
  angle_output = 0;
  angle_last_output = 0;
  angle_error = 0;
  angle_error_prev1 = 0;
  angle_error_prev2 = 0;
  
  // Tham số PID đã được set trong khai báo biến toàn cục
  set_angle_PID();  // Ghi và đặt giá trị PID góc
}

// Không cần init_velocity_PID nữa


void imu_setup(){ // Khởi tạo gia tốc kế, con quay hồi chuyển; đặt các bit đúng trên IMU
  Wire.begin();                       // Khởi tạo I2C làm master
  
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
  delay(4);                // Độ trễ 4ms để ổn định đọc dữ liệu từ IMU
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

// Hàm angle_calc() đã được gộp vào PID_Control() ISR



