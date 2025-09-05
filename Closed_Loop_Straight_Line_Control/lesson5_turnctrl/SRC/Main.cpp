#include <Arduino.h>
#include "Wire.h"
#include "SF_Servo.h"   // Thư viện điều khiển servo
#include "SF_BLDC.h"    // Thư viện điều khiển motor BLDC
#include "SF_IMU.h"     // Thư viện IMU (MPU6050)
#include "pid.h"        // Thư viện PID
#include "bipedal_data.h" // Struct và dữ liệu robot

// ------------------------ Khởi tạo đối tượng ------------------------
SF_Servo servos = SF_Servo(Wire);   // Đối tượng servo, dùng I2C
SF_IMU mpu6050 = SF_IMU(Wire);      // Đối tượng IMU, dùng I2C
SF_BLDC motors = SF_BLDC(Serial2);  // Đối tượng motor BLDC, dùng Serial2

PIDController PID_VEL{0.2,0,0,1000,50}; // PID điều tốc motor

// ------------------------ Hàm thiết lập độ cao robot ------------------------
void setRobotHeight(uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4){
  servos.setPWM(3, 0, val1); // Chân trái trước
  servos.setPWM(4, 0, val2); // Chân trái sau
  servos.setPWM(5, 0, val3); // Chân phải trước
  servos.setPWM(6, 0, val4); // Chân phải sau
}

// ------------------------ Biến lưu trạng thái robot ------------------------
robotposeparam robotPose;      // Tư thế robot: pitch, roll, yaw, gyro, ...
robotmotionparam robotMotion;  // Tham số chuyển động: turn, forward, roll, ...

// Mảng độ cao chân robot (4 chân)
uint16_t height0[4] = {275,168,244,178};
uint16_t height1[4] = {294,152,260,162};
uint16_t height2[4] = {320,170,265,110};

// Hằng số PID hoặc P điều khiển
float kp1 = 0.38; // Cần tinh chỉnh
float kp2 = 0.4;
float kp3 = 0.45;
float kpVel;

float pitch;
SF_BLDC_DATA  BLDCData; // Dữ liệu motor BLDC
int M0Dir,M1Dir;        // Hướng motor 0 và 1
float targetSpeed;      // Tốc độ mục tiêu

// ------------------------ Lấy giá trị từ IMU ------------------------
void getMPUValue(){
  mpu6050.update(); // Cập nhật dữ liệu IMU

  // Lấy góc pitch, roll, yaw và gyro
  robotPose.pitch = -mpu6050.angle[0]; // Đổi chiều do vị trí lắp đặt
  robotPose.roll  = mpu6050.angle[1];
  robotPose.yaw   = mpu6050.angle[2];
  robotPose.GyroX = mpu6050.gyro[1]; 
  robotPose.GyroY = -mpu6050.gyro[0];
  robotPose.GyroZ = -mpu6050.gyro[2];
}

// ------------------------ Đọc dữ liệu từ Serial ------------------------
void read(){
  static String receivedChars; // Lưu buffer dữ liệu nhận
  String command = "";

  while (Serial.available())
  {
    char inChar = (char)Serial.read(); // Đọc từng ký tự
    receivedChars += inChar;
    if (inChar == '\n') // Khi gặp ký tự kết thúc dòng
    {
      command = receivedChars;
      const char *d = command.c_str();
      sscanf(d,"%f", &robotMotion.turn); // Chuyển dữ liệu thành float và gán vào turn
      receivedChars = ""; // Xóa buffer
    }
  }   
}

// ------------------------ Setup ------------------------
void setup() {
  Serial.begin(921600);           // Khởi tạo serial debug
  Wire.begin(1, 2, 400000UL);     // Khởi tạo I2C
  servos.init();                   // Khởi tạo servo
  mpu6050.init();                  // Khởi tạo IMU
  motors.init();                   // Khởi tạo motor BLDC
  motors.setModes(4,4);            // Thiết lập mode motor

  // Thiết lập độ cao mặc định của 4 chân
  setRobotHeight(height0[0],height0[1],height0[2],height0[3]);

  M0Dir = 1; // Hướng motor 0
  M1Dir = -1; // Hướng motor 1
}

// ------------------------ Biến loop ------------------------
uint8_t loopCnt;
float turnTorque, turnTarget;
float turnKp;

// ------------------------ Loop chính ------------------------
void loop() 
{

  read();                  // Đọc lệnh từ Serial
  BLDCData = motors.getBLDCData(); // Lấy dữ liệu motor
  getMPUValue();           // Lấy tư thế từ IMU

  targetSpeed = 15;        // Tốc độ mục tiêu cố định (có thể thay đổi sau)

  // Tính tốc độ trung bình 2 motor
  float speedAvg = (M0Dir*BLDCData.M0_Vel + M1Dir*BLDCData.M1_Vel)/2;

  // PID điều chỉnh góc pitch
  float targetAngle = PID_VEL(targetSpeed - speedAvg);
  float turnTorque = turnKp * (robotMotion.turn - robotPose.GyroZ); // PID điều chỉnh góc quay
  float torque1 = kp1*(targetAngle - pitch) + turnTorque; // Torque motor 0
  float torque2 = kp1*(targetAngle - pitch) - turnTorque; // Torque motor 1

  // Gửi lệnh điều khiển motor
  motors.setTargets(M0Dir*torque1, M1Dir*torque2);

  loopCnt++;
  // Debug serial, nếu cần mở:
  // if(loopCnt>=100){
  //   Serial.printf("status: %.2f,%.2f,%.2f,%.2f\n",speedAvg,pitch,targetAngle,turnTorque);
  //   loopCnt=0;
  // }
}
