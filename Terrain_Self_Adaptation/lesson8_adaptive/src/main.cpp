#include <Arduino.h>
#include "SF_Servo.h"
#include "sbus.h"
#include "SF_BLDC.h"
#include "SF_IMU.h"
#include "pid.h"
#include "bipedal_data.h"

#define ROLL_OFFSET 0

SF_Servo servos = SF_Servo(Wire); // Khởi tạo servo
bfs::SbusRx sbusRx(&Serial1);// Khởi tạo thu phát SBUS

// Hàm giới hạn giá trị
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

void getRCValue(); // Hàm đọc dữ liệu từ remote
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear); // Hàm set góc servo
void inverseKinematics(); // Hàm tính toán ngược động học

std::array<int16_t, bfs::SbusRx::NUM_CH()> sbusData;
IKparam IKParam;
float height;
uint8_t lowest = 70; // Chiều cao chân thấp nhất
uint8_t highest = 130; // Chiều cao chân cao nhất
float X,Y;
float Y_demand;
float Kp_Y=0.1; // Hệ số PID điều khiển chiều cao chân
float Kp_roll = 0.05; // Hệ số PID điều khiển roll
float Phi;
float turn, forward;
float L = 100;// Chiều dài thân robot
float rollLimit=20; // Giới hạn góc roll
PIDController PID_VEL{0.2,0,0,1000,50}; // PID điều khiển tốc độ motor

int16_t alphaLeftToAngle,betaLeftToAngle,alphaRightToAngle,betaRightToAngle;

float kp1 = 0.38;// Hệ số P cần điều chỉnh
float kp2 = 0.4;
float kp3 = 0.45;
float kpVel;

float pitch;
SF_BLDC_DATA  BLDCData;
SF_IMU mpu6050 = SF_IMU(Wire); // Khởi tạo cảm biến IMU
SF_BLDC motors = SF_BLDC(Serial2); // Khởi tạo motor BLDC
robotposeparam robotPose; // Thông số tư thế robot
robotmotionparam robotMotion; // Thông số chuyển động robot
int M0Dir,M1Dir; // Chiều quay motor
float targetSpeed;
float stab_roll = 0; // Biến hỗ trợ ổn định roll

// Hàm đọc dữ liệu từ IMU
void getMPUValue(){
  mpu6050.update();
  // Gán dữ liệu IMU vào robotPose
  robotPose.pitch = -mpu6050.angle[0];// do cách đặt IMU nên đổi dấu
  robotPose.roll = mpu6050.angle[1];// do cách đặt IMU nên đổi dấu
  robotPose.yaw = mpu6050.angle[2];
  robotPose.GyroX = mpu6050.gyro[1]; 
  robotPose.GyroY = -mpu6050.gyro[0];
  robotPose.GyroZ = -mpu6050.gyro[2];
}

void setup() {
  Serial.begin(921600);
  Wire.begin(1,2,400000UL);
  mpu6050.init(); // Khởi tạo IMU
  servos.init(); // Khởi tạo servo
  servos.setAngleRange(0,300); // Giới hạn góc servo
  servos.setPluseRange(500,2500); // Giới hạn xung servo
  sbusRx.Begin(SBUSPIN,-1); // Khởi tạo SBUS
  motors.init(); // Khởi tạo BLDC
  motors.setModes(4,4); // Chế độ điều khiển motor
  M0Dir = -1;
  M1Dir = -1;
}

uint8_t loopCnt;
float turnTorque,turnTarget;
float turnKp;

void loop() {
  BLDCData = motors.getBLDCData(); // Lấy dữ liệu motor
  getMPUValue(); // Cập nhật IMU
  getRCValue(); // Đọc dữ liệu remote

  // Map dữ liệu remote sang giá trị điều khiển
  robotMotion.turn = map(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX, -5, 5);
  targetSpeed = map(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX, -20, 20);
  Y_demand = ((int)map(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX, lowest, highest));// Chiều cao chân mong muốn
  Phi = map(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*rollLimit, rollLimit);// Góc roll mong muốn

  float speedAvg = (M0Dir*BLDCData.M0_Vel + M1Dir*BLDCData.M1_Vel)/2; // Tốc độ trung bình motor
  float targetAngle = PID_VEL(targetSpeed - speedAvg); // PID điều khiển tốc độ motor
  float turnTorque = turnKp * (robotMotion.turn-robotPose.GyroZ); // Lực moment để quay
  float torque1 = kp1*(targetAngle - robotPose.pitch) + turnTorque; // Torque motor M0
  float torque2 = kp1*(targetAngle - robotPose.pitch) - turnTorque; // Torque motor M1

  motors.setTargets(M0Dir*torque1, M1Dir*torque2); // Gửi lệnh tới motor

  // Serial debug
  loopCnt++;
  if(loopCnt>=100){
    Serial.printf("status: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",robotPose.pitch,robotPose.GyroZ,targetAngle,turnTorque,torque1,torque2);
    loopCnt=0;
  }
  
  X = 0;
  Y = Y + Kp_Y * (Y_demand - Y); // PID điều khiển chiều cao chân

  uint16_t Remoter_Input = Y;
  float E_H = (L/2) * sin(Phi*(PI/180));
  stab_roll = stab_roll + Kp_roll * (0 - robotPose.roll); // PID điều khiển roll để thích nghi địa hình
  float L_Height = Remoter_Input + stab_roll;
  float R_Height = Remoter_Input - stab_roll;

  IKParam.XLeft = X;
  IKParam.XRight = X;
  IKParam.YLeft = L_Height;
  IKParam.YRight = R_Height;

  inverseKinematics(); // Tính toán góc servo từ IK
}

// Hàm đọc remote
void getRCValue(){
  if(sbusRx.Read()){
    sbusData = sbusRx.ch();
    RCValue[0] = sbusData[0];
    RCValue[1] = sbusData[1];
    RCValue[2] = sbusData[2];
    RCValue[3] = sbusData[3];
    RCValue[4] = sbusData[4];
    RCValue[5] = sbusData[5];

    RCValue[0] = _constrain(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX);
    RCValue[1] = _constrain(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX);
    RCValue[2] = _constrain(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX);
    RCValue[3] = _constrain(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX);
  }
}

// Hàm set góc servo
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear){
  servos.setAngle(3, servoLeftFront);// Chân trái trước
  servos.setAngle(4, servoLeftRear);// Chân trái sau
  servos.setAngle(5, servoRightFront);// Chân phải trước
  servos.setAngle(6, servoRightRear);// Chân phải sau
}

// Hàm tính Inverse Kinematics
void inverseKinematics(){
  float alpha1,alpha2,beta1,beta2;
  uint16_t servoLeftFront,servoLeftRear,servoRightFront,servoRightRear;

  // Tính toán chân trái
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
  
  // Tính toán chân phải
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

  // Chuyển radian sang góc độ
  alphaLeftToAngle = (int)((IKParam.alphaLeft / 6.28) * 360);
  betaLeftToAngle = (int)((IKParam.betaLeft / 6.28) * 360);
  alphaRightToAngle = (int)((IKParam.alphaRight / 6.28) * 360);
  betaRightToAngle = (int)((IKParam.betaRight / 6.28) * 360);

  // Tính góc servo cuối cùng
  servoLeftFront = 90 + betaLeftToAngle;
  servoLeftRear = 90 + alphaLeftToAngle;
  servoRightFront = 270 - betaRightToAngle;
  servoRightRear = 270 - alphaRightToAngle;

  setServoAngle(servoLeftFront,servoLeftRear,servoRightFront,servoRightRear); // Gửi góc servo
}
