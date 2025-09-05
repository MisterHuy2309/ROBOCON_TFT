#include <Arduino.h>
#include "Wire.h"
#include "SF_Servo.h"
#include "SF_BLDC.h"
#include "SF_IMU.h"
#include "pid.h"

// Khởi tạo các thiết bị
SF_Servo servos = SF_Servo(Wire);    // Servo PCA9685
SF_IMU mpu6050 = SF_IMU(Wire);      // Cảm biến IMU MPU6050
SF_BLDC motors = SF_BLDC(Serial2);  // Động cơ BLDC

// Hàm thiết lập chiều cao robot bằng 4 servo
void setRobotHeight(uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4){
  servos.setPWM(3, 0, val1);
  servos.setPWM(4, 0, val2);
  servos.setPWM(5, 0, val3);
  servos.setPWM(6, 0, val4);
}

// Biến lưu góc pitch (nghiêng) của robot
float pitch;
// Biến lưu lệnh nhận từ Serial
int command;
// Biến PID và mục tiêu điều khiển
float target,kp1,kp2,kp3;
// Hướng động cơ
int M0Dir,M1Dir;

// Các mức chiều cao khác nhau của robot
uint16_t height0[4] = {275,168,244,178};
uint16_t height1[4] = {294,152,260,162};
uint16_t height2[4] = {320,170,265,110};

// Hàm đọc lệnh từ Serial
void read(){
  if (Serial.available() > 0) {
    command = Serial.parseInt(); // Đọc số nguyên từ Serial
  }
}

void setup() {
  Serial.begin(921600);        // Khởi tạo Serial
  Wire.begin(1, 2, 400000UL);  // Khởi tạo I2C (SDA=1, SCL=2, tốc độ 400kHz)
  
  servos.init();   // Khởi tạo servo
  mpu6050.init();  // Khởi tạo IMU
  motors.init();   // Khởi tạo động cơ BLDC
  motors.setModes(4,4); // Thiết lập chế độ điều khiển cho 2 động cơ
  setRobotHeight(height0[0],height0[1],height0[2],height0[3]); // Đặt chiều cao ban đầu

  M0Dir = 1;  // Hướng động cơ 0
  M1Dir = -1; // Hướng động cơ 1
  kp1 = 0.38; // Hệ số P cần tinh chỉnh
  kp2 = 0.4;
  kp3 = 0.45;
}

void loop() {
  mpu6050.update();       // Cập nhật giá trị IMU
  pitch = mpu6050.angle[0]; // Lấy góc pitch (nghiêng về trước/sau)

  read(); // Đọc lệnh từ Serial

  switch (command) {
    case 1:
      setRobotHeight(height0[0],height0[1],height0[2],height0[3]);
      target = kp1*(0-pitch); // Tính giá trị điều khiển dựa trên góc pitch
      break;
      
    case 2:
      setRobotHeight(height1[0],height1[1],height1[2],height1[3]);
      target = kp3*(0-pitch);
      break;
      
    case 3:
      setRobotHeight(height2[0],height2[1],height2[2],height2[3]);
      target = kp3*(0-pitch);
      break;
      
    default:
      Serial.println("Lệnh không hợp lệ"); // In thông báo khi lệnh sai
      break;
  }

  // Gửi lệnh điều khiển tới động cơ BLDC, có nhân với hướng
  motors.setTargets(M0Dir*target, M1Dir*target);
}
