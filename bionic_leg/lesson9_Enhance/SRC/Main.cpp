#include <Arduino.h>
#include "SF_Servo.h"      // Thư viện điều khiển servo
#include "sbus.h"          // Thư viện nhận tín hiệu RC SBUS
#include "SF_BLDC.h"       // Thư viện điều khiển motor BLDC
#include "SF_IMU.h"        // Thư viện IMU (MPU6050)
#include "pid.h"           // Thư viện PID
#include "bipedal_data.h"  // Dữ liệu và struct robot

#define ROLL_OFFSET 0      // Offset góc roll

SF_Servo servos = SF_Servo(Wire);      // Khởi tạo đối tượng servo
bfs::SbusRx sbusRx(&Serial1);          // Khởi tạo đối tượng SBUS

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // Giới hạn giá trị

// Khai báo trước các hàm
void getRCValue();
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear);
void inverseKinematics();

// Biến toàn cục
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbusData;  // Lưu dữ liệu SBUS
IKparam IKParam;          // Tham số IK
float height;
uint8_t lowest = 70;
uint8_t highest = 130;
float X,Y;
float Y_demand;
float Kp_Y=0.1;          // Hằng số PID ngang Y
float Kp_roll = 0.05;    // Hằng số PID roll
float Phi;
float turn, forward;
float L = 100;           // Chiều dài cơ thể robot
float rollLimit=20;      // Giới hạn góc roll
PIDController PID_VEL{0.2,0,0,1000,50}; // PID điều tốc

// Biến lưu góc servo
int16_t alphaLeftToAngle,betaLeftToAngle,alphaRightToAngle,betaRightToAngle;

// Hằng số điều khiển
float kp1 = 0.38;       // P cần tinh chỉnh
float kp2 = 0.4;
float kp3 = 0.45;
float kpVel;
float Kp_x = 1.1;

float pitch;
SF_BLDC_DATA  BLDCData;         // Dữ liệu motor BLDC
SF_IMU mpu6050 = SF_IMU(Wire);  // Khởi tạo IMU
SF_BLDC motors = SF_BLDC(Serial2); // Khởi tạo motor BLDC
robotposeparam robotPose;        // Trạng thái tư thế robot
robotmotionparam robotMotion;    // Trạng thái chuyển động
int M0Dir,M1Dir;
float targetSpeed;
float stab_roll = 0;

// ------------------------ Lấy giá trị từ IMU ------------------------
void getMPUValue(){
  mpu6050.update(); // Cập nhật dữ liệu IMU
  robotPose.pitch = -mpu6050.angle[0]; // Đổi chiều để phù hợp lắp đặt
  robotPose.roll  = mpu6050.angle[1];
  robotPose.yaw   = mpu6050.angle[2];
  robotPose.GyroX = mpu6050.gyro[1]; 
  robotPose.GyroY = -mpu6050.gyro[0];
  robotPose.GyroZ = -mpu6050.gyro[2];
}

// ------------------------ Setup robot ------------------------
void setup() {
  Serial.begin(921600);        // Khởi tạo serial debug
  Wire.begin(1,2,400000UL);    // Khởi tạo I2C
  mpu6050.init();               // Khởi tạo IMU
  servos.init();                // Khởi tạo servo
  servos.setAngleRange(0,300);  // Thiết lập góc servo
  servos.setPluseRange(500,2500); // Thiết lập tín hiệu PWM servo
  sbusRx.Begin(SBUSPIN,-1);    // Bắt đầu nhận SBUS
  motors.init();                // Khởi tạo motor BLDC
  motors.setModes(4,4);         // Thiết lập mode motor
  M0Dir = -1;                   // Hướng motor 0
  M1Dir = -1;                   // Hướng motor 1
}

// ------------------------ Biến loop ------------------------
uint8_t loopCnt;
float turnTorque,turnTarget;
float turnKp;

// ------------------------ Loop chính ------------------------
void loop() {
  BLDCData = motors.getBLDCData();  // Lấy dữ liệu motor
  getMPUValue();                     // Lấy tư thế từ IMU
  getRCValue();                      // Lấy tín hiệu từ RC

  // Map dữ liệu RC sang tham số robot
  robotMotion.turn = map(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX, -5, 5);
  targetSpeed = map(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX, -20, 20);
  Y_demand = ((int)map(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX, lowest, highest));
  Phi = map(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*rollLimit, rollLimit);

  float speedAvg = (M0Dir*BLDCData.M0_Vel + M1Dir*BLDCData.M1_Vel)/2; // Tính tốc độ trung bình 2 motor
  float targetAngle = PID_VEL(targetSpeed - speedAvg); // PID điều chỉnh góc pitch
  float turnTorque = turnKp * (robotMotion.turn-robotPose.GyroZ); // PID điều chỉnh roll
  float torque1 = kp1*(targetAngle - robotPose.pitch) + turnTorque;
  float torque2 = kp1*(targetAngle - robotPose.pitch) - turnTorque;

  motors.setTargets(M0Dir*torque1, M1Dir*torque2); // Gửi lệnh motor

  loopCnt++;
  if(loopCnt>=100){
    Serial.printf("status: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      robotPose.pitch,robotPose.GyroZ,targetAngle,turnTorque,torque1,torque2);
    loopCnt=0;
  }
  
  X = -Kp_x * (targetSpeed - speedAvg); 
  Y = Y + Kp_Y * (Y_demand - Y);

  uint16_t Remoter_Input = Y;
  float E_H = (L/2) * sin(Phi*(PI/180));
  stab_roll = stab_roll + Kp_roll * (0 - robotPose.roll);
  float L_Height = Remoter_Input + stab_roll;
  float R_Height = Remoter_Input - stab_roll;

  IKParam.XLeft = X;
  IKParam.XRight = X;
  IKParam.YLeft = L_Height;
  IKParam.YRight = R_Height;

  inverseKinematics(); // Tính toán IK và gửi góc servo
}

// ------------------------ Lấy dữ liệu từ RC ------------------------
void getRCValue(){
  if(sbusRx.Read()){
    sbusData = sbusRx.ch();
    for(int i=0;i<6;i++){
      RCValue[i] = sbusData[i];
    }
    RCValue[0] = _constrain(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX);
    RCValue[1] = _constrain(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX);
    RCValue[2] = _constrain(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX);
    RCValue[3] = _constrain(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX);
  }
}

// ------------------------ Gửi góc servo ------------------------
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear){
  servos.setAngle(3, servoLeftFront); 
  servos.setAngle(4, servoLeftRear);  
  servos.setAngle(5, servoRightFront); 
  servos.setAngle(6, servoRightRear);  
}

// ------------------------ Tính toán IK ------------------------
void inverseKinematics(){
  float alpha1,alpha2,beta1,beta2;
  uint16_t servoLeftFront,servoLeftRear,servoRightFront,servoRightRear;

  // Tính toán IK chân trái
  float aLeft = 2 * IKParam.XLeft * L1;
  float bLeft = 2 * IKParam.YLeft * L1;
  float cLeft = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
  float dLeft = 2 * L4 * (IKParam.XLeft - L5);
  float eLeft = 2 * L4 * IKParam.YLeft;
  float fLeft = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);

  alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
  beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

  alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

  if(alpha1 >= PI/4) IKParam.alphaLeft = alpha1;
  else IKParam.alphaLeft = alpha2;
  if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaLeft = beta1;
  else IKParam.betaLeft = beta2;
  
  // Tính toán IK chân phải
  float aRight = 2 * IKParam.XRight * L1;
  float bRight = 2 * IKParam.YRight * L1;
  float cRight = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
  float dRight = 2 * L4 * (IKParam.XRight - L5);
  float eRight = 2 * L4 * IKParam.YRight;
  float fRight = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

  IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
  beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

  if(alpha1 >= PI/4) IKParam.alphaRight = alpha1;
  else IKParam.alphaRight = alpha2;
  if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaRight = beta1;
  else IKParam.betaRight = beta2;

  // Chuyển radian sang độ
  alphaLeftToAngle = (int)((IKParam.alphaLeft / 6.28) * 360);
  betaLeftToAngle  = (int)((IKParam.betaLeft / 6.28) * 360);
  alphaRightToAngle = (int)((IKParam.alphaRight / 6.28) * 360);
  betaRightToAngle  = (int)((IKParam.betaRight / 6.28) * 360);

  // Tính góc servo cuối cùng
  servoLeftFront = 90 + betaLeftToAngle;
  servoLeftRear  = 90 + alphaLeftToAngle;
  servoRightFront = 270 - betaRightToAngle;
  servoRightRear  = 270 - alphaRightToAngle;

  setServoAngle(servoLeftFront,servoLeftRear,servoRightFront,servoRightRear);
}
