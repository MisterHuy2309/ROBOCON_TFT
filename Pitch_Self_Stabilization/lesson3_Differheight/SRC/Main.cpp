#include <Arduino.h>
#include "SF_Servo.h"
#include "Wire.h"

// Khởi tạo đối tượng servo, dùng bus I2C Wire
SF_Servo servos = SF_Servo(Wire);

// Hàm thiết lập chiều cao bốn chân robot
void setRobotHeight(uint16_t val1,uint16_t val2,uint16_t val3,uint16_t val4){
  servos.setPWM(3, 0, val1); // Chân trái trước
  servos.setPWM(4, 0, val2); // Chân trái sau
  servos.setPWM(5, 0, val3); // Chân phải trước
  servos.setPWM(6, 0, val4); // Chân phải sau
}

unsigned long currTime, prevTime;

void setup(){
  Serial.begin(115200);           // Khởi tạo Serial để debug
  Wire.begin(1, 2, 400000UL);     // Khởi tạo I2C, SDA=1, SCL=2, tốc độ 400kHz
  servos.init();                   // Khởi tạo servo
  prevTime = millis();             // Lưu thời gian hiện tại
}

void loop() {
  // Thay đổi độ cao từng trạng thái
  setRobotHeight(275,168,244,178);
  delay(5000); // Giữ 5 giây

  setRobotHeight(294,154,260,162);
  delay(5000); // Giữ 5 giây

  setRobotHeight(313,139,277,146);
  delay(5000); // Giữ 5 giây
}
