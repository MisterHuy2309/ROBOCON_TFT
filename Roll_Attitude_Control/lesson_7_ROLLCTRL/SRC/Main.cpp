#include <Arduino.h>
#include "SF_Servo.h"
#include "sbus.h"
#include "bipedal_data.h"

#define ROLL_OFFSET 0

SF_Servo servos = SF_Servo(Wire); // Khởi tạo đối tượng servo
bfs::SbusRx sbusRx(&Serial1);     // Khởi tạo đối tượng nhận dữ liệu từ remote

// Hàm giới hạn giá trị trong khoảng low đến high
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) 

// Khai báo các hàm
void getRCValue();
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear);
void inverseKinematics();

std::array<int16_t, bfs::SbusRx::NUM_CH()> sbusData; // Mảng lưu giá trị kênh SBUS
IKparam IKParam;      // Biến lưu thông số cho inverse kinematics
float height;
uint8_t lowest = 70;  // Chiều cao chân min
uint8_t highest = 130; // Chiều cao chân max
float X,Y;
float Y_demand;
float Kp_Y=0.1;       // Hằng số điều khiển vị trí Y
float Phi;
float turn, forward;
float L = 100;        // Chiều dài thân
float rollLimit=20;   // Giới hạn góc lật

int16_t alphaLeftToAngle,betaLeftToAngle,alphaRightToAngle,betaRightToAngle;

// Thiết lập ban đầu
void setup() {
  Serial.begin(921600);          // Bắt đầu Serial
  Wire.begin(1,2,400000UL);      // Bắt đầu I2C
  servos.init();                  // Khởi tạo servo
  servos.setAngleRange(0,300);   // Đặt góc min-max cho servo
  servos.setPluseRange(500,2500);// Đặt xung PWM min-max cho servo
  sbusRx.Begin(SBUSPIN,-1);      // Khởi tạo SBUS
}

// Vòng lặp chính
void loop() {
  getRCValue(); // Lấy giá trị từ remote

  // Tính chiều cao chân theo tín hiệu RC (scale từ min-max)
  Y_demand = ((int)map(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX, lowest, highest));
  // Tính góc lật mong muốn từ RC
  Phi = map(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*rollLimit, rollLimit);

  X = 0;
  Y = Y + Kp_Y * (Y_demand - Y); // Bộ điều khiển P đơn giản cho Y

  uint16_t Remoter_Input = Y;
  float E_H = (L/2) * sin(Phi*(PI/180)); // Tính hiệu ứng roll
  float L_Height = Remoter_Input + E_H;  // Chiều cao chân trái
  float R_Height = Remoter_Input - E_H;  // Chiều cao chân phải

  IKParam.XLeft = X;
  IKParam.XRight = X;
  IKParam.YLeft = L_Height;
  IKParam.YRight = R_Height;

  inverseKinematics(); // Tính toán góc servo từ chiều cao chân
}

// Hàm đọc giá trị RC từ SBUS
void getRCValue(){
  if(sbusRx.Read()){
    sbusData = sbusRx.ch();
    RCValue[0] = sbusData[0];
    RCValue[1] = sbusData[1];
    RCValue[2] = sbusData[2];
    RCValue[3] = sbusData[3];
    RCValue[4] = sbusData[4];
    RCValue[5] = sbusData[5];

    // Giới hạn giá trị RC trong khoảng min-max
    RCValue[0] = _constrain(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX);
    RCValue[1] = _constrain(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX);
    RCValue[2] = _constrain(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX);
    RCValue[3] = _constrain(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX);
  }
}

// Gửi góc tính được đến servo
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear){
  servos.setAngle(3, servoLeftFront); // Chân trái trước
  servos.setAngle(4, servoLeftRear);  // Chân trái sau
  servos.setAngle(5, servoRightFront);// Chân phải trước
  servos.setAngle(6, servoRightRear); // Chân phải sau
}

// Tính inverse kinematics
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

  alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI); // Chuyển sang radian dương
  alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

  if(alpha1 >= PI/4) IKParam.alphaLeft = alpha1;  // Chọn nghiệm thích hợp
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

  // Chuyển radian sang góc độ servo
  alphaLeftToAngle = (int)((IKParam.alphaLeft / 6.28) * 360);
  betaLeftToAngle = (int)((IKParam.betaLeft / 6.28) * 360);

  alphaRightToAngle = (int)((IKParam.alphaRight / 6.28) * 360);
  betaRightToAngle = (int)((IKParam.betaRight / 6.28) * 360);

  // Điều chỉnh servo theo góc tính toán
  servoLeftFront = 90 + betaLeftToAngle;
  servoLeftRear = 90 + alphaLeftToAngle;
  servoRightFront = 270 - betaRightToAngle;
  servoRightRear = 270 - alphaRightToAngle;

  setServoAngle(servoLeftFront,servoLeftRear,servoRightFront,servoRightRear);
}
