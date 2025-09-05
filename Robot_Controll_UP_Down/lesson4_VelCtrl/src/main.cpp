#include <Arduino.h>
#include "Wire.h"
#include "SF_Servo.h"
#include "SF_BLDC.h"
#include "SF_IMU.h"
#include "pid.h"

// Khởi tạo các đối tượng
SF_Servo servos = SF_Servo(Wire);      // Servo chân robot
SF_IMU mpu6050 = SF_IMU(Wire);         // Cảm biến IMU (MPU6050)
SF_BLDC motors = SF_BLDC(Serial2);     // Động cơ BLDC

PIDController PID_VEL{0.2,0,0,1000,50}; // PID điều khiển tốc độ

// Hàm điều chỉnh độ cao các chân robot
void setRobotHeight(uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4){
  servos.setPWM(3, 0, val1);
  servos.setPWM(4, 0, val2);
  servos.setPWM(5, 0, val3);
  servos.setPWM(6, 0, val4);
}

// Các mức cao của chân robot
uint16_t height0[4] = {275,168,244,178};
uint16_t height1[4] = {294,152,260,162};
uint16_t height2[4] = {320,170,265,110};

// Hệ số P dùng điều chỉnh pitch
float kp1 = 0.38; // Cần hiệu chỉnh chính xác
float kp2 = 0.4;
float kp3 = 0.45;
float kpVel;

float pitch;                // Góc nghiêng của robot
SF_BLDC_DATA  BLDCData;     // Dữ liệu từ động cơ BLDC
int M0Dir,M1Dir;            // Chiều quay của động cơ
float targetSpeed;           // Tốc độ mục tiêu nhận từ Serial

// Hàm đọc dữ liệu Serial để nhận tốc độ mục tiêu
void read(){
  static String receivedChars;
  String command = "";
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    receivedChars += inChar;
    if (inChar == '\n')
    {
      command = receivedChars;
      const char *d = command.c_str();
      sscanf(d,"%f", &targetSpeed); // Chuyển dữ liệu nhận được thành số thực
      receivedChars = "";
    }
  }
}

void setup() {
  Serial.begin(921600);
  Wire.begin(1, 2, 400000UL);
  servos.init();        // Khởi tạo servo
  mpu6050.init();       // Khởi tạo IMU
  motors.init();        // Khởi tạo BLDC
  motors.setModes(4,4); // Cấu hình chế độ động cơ
  setRobotHeight(height0[0],height0[1],height0[2],height0[3]); // Đặt chân robot ở mức cao ban đầu

  M0Dir = 1;  // Chiều quay động cơ M0
  M1Dir = -1; // Chiều quay động cơ M1
}

uint8_t loopCnt;

void loop() {
  read();              // Đọc tốc độ mục tiêu từ Serial
  targetSpeed = 0;     // Mặc định tốc độ mục tiêu bằng 0

  BLDCData = motors.getBLDCData(); // Lấy dữ liệu BLDC hiện tại
  float speedAvg = (M0Dir*BLDCData.M0_Vel + M1Dir*BLDCData.M1_Vel)/2; // Tính tốc độ trung bình

  mpu6050.update();    // Cập nhật dữ liệu IMU
  setRobotHeight(height0[0],height0[1],height0[2],height0[3]); // Giữ chân robot ở mức ban đầu
  pitch = mpu6050.angle[0]; // Lấy góc nghiêng hiện tại

  float targetAngle = PID_VEL(targetSpeed - speedAvg);   // Tính góc mục tiêu từ PID tốc độ
  float torque = kp1*(targetAngle - pitch);             // Tính mô-men điều khiển dựa trên chênh lệch góc

  motors.setTargets(M0Dir*torque, M1Dir*torque);       // Gửi lệnh điều khiển tới động cơ

  // In dữ liệu trạng thái mỗi 100 vòng lặp
  if(loopCnt>=100){
    Serial.printf("status: %.2f,%.2f,%.2f,%.2f\n",speedAvg,pitch,targetAngle,torque);
    loopCnt=0;
  }
  loopCnt++;
}
