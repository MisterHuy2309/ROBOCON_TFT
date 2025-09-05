#include <Arduino.h>
#include <Wire.h>
#include "SF_IMU.h"

// Khởi tạo đối tượng IMU với bus I2C
SF_IMU mpu6050 = SF_IMU(Wire);

// Biến lưu góc và tốc độ góc
float roll, pitch, yaw;
float gyroX, gyroY, gyroZ;

void setup(){
  Serial.begin(115200);           // Khởi động Serial
  Wire.begin(1, 2, 400000UL);     // Khởi động I2C (SDA=1, SCL=2, tần số 400kHz)
  mpu6050.init();                  // Khởi tạo MPU6050
}

void loop() {
  mpu6050.update();                // Cập nhật dữ liệu mới từ MPU6050

  // Lấy góc (degree) và tốc độ góc (°/s)
  roll  = mpu6050.angle[0];        // Roll
  pitch = mpu6050.angle[1];        // Pitch
  yaw   = mpu6050.angle[2];        // Yaw
  gyroX = mpu6050.gyro[0];         // Gyro X
  gyroY = mpu6050.gyro[1];         // Gyro Y
  gyroZ = mpu6050.gyro[2];         // Gyro Z

  // In ra Serial theo định dạng CSV
  Serial.printf("%f,%f,%f,%f,%f,%f\n", roll, pitch, yaw, gyroX, gyroY, gyroZ);

  delay(100); // Cập nhật 10 lần/giây
}
