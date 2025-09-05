#ifndef _SF_SERVO_h
#define _SF_SERVO_h

#include <Arduino.h>
#include <Wire.h>  // Thư viện I2C cho Arduino

// ------------------------ Chân và địa chỉ ------------------------
#define SERVO_ENABLE_PIN 42        // Chân bật/tắt servo
#define PCA9685_ADDR 0x40          // Địa chỉ I2C mặc định của PCA9685

// ------------------------ Các thanh ghi PCA9685 ------------------------
#define PCA9685_MODE1 0x00         // Mode Register 1
#define PCA9685_MODE2 0x01         // Mode Register 2
#define PCA9685_SUBADR1 0x02       // I2C subaddress 1
#define PCA9685_SUBADR2 0x03       // I2C subaddress 2
#define PCA9685_SUBADR3 0x04       // I2C subaddress 3
#define PCA9685_ALLCALLADR 0x05    // LED All Call I2C address

// LED0
#define PCA9685_LED0_ON_L 0x06     // Giá trị ON thấp byte
#define PCA9685_LED0_ON_H 0x07     // Giá trị ON cao byte
#define PCA9685_LED0_OFF_L 0x08    // Giá trị OFF thấp byte
#define PCA9685_LED0_OFF_H 0x09    // Giá trị OFF cao byte

// Tất cả LED
#define PCA9685_ALLLED_ON_L 0xFA
#define PCA9685_ALLLED_ON_H 0xFB
#define PCA9685_ALLLED_OFF_L 0xFC
#define PCA9685_ALLLED_OFF_H 0xFD

#define PCA9685_PRESCALE 0xFE      // Prescaler tần số PWM
#define PCA9685_TESTMODE 0xFF      // Chế độ test

// ------------------------ Mode1 ------------------------
#define MODE1_ALLCAL 0x01  // Phản hồi LED All Call I2C
#define MODE1_SUB3 0x02    // Phản hồi subaddress 3
#define MODE1_SUB2 0x04    // Phản hồi subaddress 2
#define MODE1_SUB1 0x08    // Phản hồi subaddress 1
#define MODE1_SLEEP 0x10   // Chế độ ngủ, tắt oscillator
#define MODE1_AI 0x20      // Auto-Increment
#define MODE1_EXTCLK 0x40  // Dùng EXTCLK
#define MODE1_RESTART 0x80 // Restart

// ------------------------ Mode2 ------------------------
#define MODE2_OUTNE_0 0x01 // Active LOW output enable
#define MODE2_OUTNE_1 0x02 // Active LOW output enable - high impedence
#define MODE2_OUTDRV 0x04  // Totem pole structure vs open-drain
#define MODE2_OCH 0x08     // Outputs thay đổi trên ACK vs STOP
#define MODE2_INVRT 0x10   // Đảo logic output

#define PCA9685_I2C_ADDRESS 0x40      // Địa chỉ mặc định PCA9685
#define FREQUENCY_OSCILLATOR 25000000 // Tần số oscillator nội

#define PCA9685_PRESCALE_MIN 3    // Giá trị prescale nhỏ nhất
#define PCA9685_PRESCALE_MAX 255  // Giá trị prescale lớn nhất

// ------------------------ Lớp SF_Servo ------------------------
class SF_Servo
{
  public:
    SF_Servo(TwoWire &i2c);  // Constructor: truyền đối tượng I2C
    void init();              // Khởi tạo servo
    void enable();            // Bật servo
    void disable();           // Tắt servo
    void setAngle(uint8_t num, uint16_t val);  // Thiết lập góc servo
    void setAngleRange(uint8_t min, uint8_t max); // Thiết lập dải góc
    void setPWMFreq(float freq);                 // Thiết lập tần số PWM
    void setPin(uint8_t num, uint16_t val, bool invert = false); // Thiết lập pin PWM
    void setPWM(uint8_t num, uint16_t on, uint16_t off);         // Thiết lập PWM chi tiết
    void reset();             // Reset PCA9685
    void sleep();             // Chế độ ngủ
    void wakeup();            // Thức dậy

  private:
    uint8_t _min;   // Giá trị min cho góc
    uint8_t _max;   // Giá trị max cho góc
    float cal_min;  // Giá trị min sau khi hiệu chuẩn
    float cal_max;  // Giá trị max sau khi hiệu chuẩn

    TwoWire *_i2c;  // Con trỏ đến đối tượng I2C

    void writeToPCA(uint8_t addr, uint8_t data); // Ghi dữ liệu vào PCA9685
    uint8_t readFromPCA(uint8_t addr);           // Đọc dữ liệu từ PCA9685
};

#endif
