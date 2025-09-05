#ifndef _SF_SERVO_h
#define _SF_SERVO_h

#include <Arduino.h>
#include <Wire.h>

// Chân bật/tắt servo
#define SERVO_ENABLE_PIN 42
// Địa chỉ I2C của PCA9685
#define PCA9685_ADDR 0x40

// Các thanh ghi PCA9685
#define PCA9685_MODE1 0x00      /**< Thanh ghi chế độ 1 */
#define PCA9685_MODE2 0x01      /**< Thanh ghi chế độ 2 */
#define PCA9685_SUBADR1 0x02    /**< Địa chỉ phụ I2C 1 */
#define PCA9685_SUBADR2 0x03    /**< Địa chỉ phụ I2C 2 */
#define PCA9685_SUBADR3 0x04    /**< Địa chỉ phụ I2C 3 */
#define PCA9685_ALLCALLADR 0x05 /**< Địa chỉ gọi tất cả LED I2C */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 bật, byte thấp */
#define PCA9685_LED0_ON_H 0x07  /**< LED0 bật, byte cao */
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 tắt, byte thấp */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 tắt, byte cao */
// v.v cho tất cả 16 LED: LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< Load tất cả thanh ghi LEDn_ON, byte thấp */
#define PCA9685_ALLLED_ON_H 0xFB  /**< Load tất cả thanh ghi LEDn_ON, byte cao */
#define PCA9685_ALLLED_OFF_L 0xFC /**< Load tất cả thanh ghi LEDn_OFF, byte thấp */
#define PCA9685_ALLLED_OFF_H 0xFD /**< Load tất cả thanh ghi LEDn_OFF, byte cao */
#define PCA9685_PRESCALE 0xFE     /**< Thanh ghi điều chỉnh tần số PWM */
#define PCA9685_TESTMODE 0xFF     /**< Chế độ kiểm tra */

// Các cờ chế độ 1
#define MODE1_ALLCAL 0x01  /**< Phản hồi địa chỉ gọi tất cả LED */
#define MODE1_SUB3 0x02    /**< Phản hồi địa chỉ phụ 3 */
#define MODE1_SUB2 0x04    /**< Phản hồi địa chỉ phụ 2 */
#define MODE1_SUB1 0x08    /**< Phản hồi địa chỉ phụ 1 */
#define MODE1_SLEEP 0x10   /**< Chế độ tiết kiệm năng lượng, tắt dao động */
#define MODE1_AI 0x20      /**< Tự động tăng địa chỉ */
#define MODE1_EXTCLK 0x40  /**< Dùng đồng hồ từ chân EXTCLK */
#define MODE1_RESTART 0x80 /**< Cho phép khởi động lại */

// Các cờ chế độ 2
#define MODE2_OUTNE_0 0x01 /**< Bật đầu ra LOW */
#define MODE2_OUTNE_1 0x02 /**< Bật đầu ra LOW, high impedence */
#define MODE2_OUTDRV 0x04  /**< Cấu trúc totem pole vs open-drain */
#define MODE2_OCH 0x08     /**< Thay đổi khi ACK vs STOP */
#define MODE2_INVRT 0x10   /**< Đảo logic đầu ra */

#define PCA9685_I2C_ADDRESS 0x40      // Địa chỉ mặc định của PCA9685
#define FREQUENCY_OSCILLATOR 25000000 /**< Tần số dao động bên trong */

#define PCA9685_PRESCALE_MIN 3    // Giá trị điều chỉnh tối thiểu
#define PCA9685_PRESCALE_MAX 255  // Giá trị điều chỉnh tối đa

class SF_Servo
{
  public:
    SF_Servo(TwoWire &i2c);
    void init();                       // Khởi tạo servo
    void enable();                     // Bật servo
    void disable();                    // Tắt servo
    void setAngle(uint8_t num, uint16_t val); // Đặt góc servo
    void setAngleRange(uint8_t min, uint8_t max); // Đặt dải góc servo
    void setPWMFreq(float freq);       // Đặt tần số PWM
    void setPin(uint8_t num, uint16_t val, bool invert = false); // Đặt giá trị PWM cho chân
    void setPWM(uint8_t num, uint16_t on, uint16_t off); // Cấu hình PWM trực tiếp
    void reset();                      // Reset PCA9685
    void sleep();                      // Chế độ ngủ
    void wakeup();                     // Thức dậy
      
  private:
    uint8_t _min;  // Giá trị min
    uint8_t _max;  // Giá trị max
    float cal_min; // Giá trị min đã hiệu chỉnh
    float cal_max; // Giá trị max đã hiệu chỉnh

    TwoWire *_i2c; // Con trỏ tới I2C

    void writeToPCA(uint8_t addr, uint8_t data); // Ghi dữ liệu tới PCA9685
    uint8_t readFromPCA(uint8_t addr);           // Đọc dữ liệu từ PCA9685
};

#endif
