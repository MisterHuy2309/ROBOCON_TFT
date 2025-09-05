#ifndef _SF_SERVO_h
#define _SF_SERVO_h

#include <Arduino.h>
#include <Wire.h>

// Chân bật servo
#define SERVO_ENABLE_PIN 42
// Địa chỉ I2C mặc định của PCA9685
#define PCA9685_ADDR 0x40

// Các thanh ghi điều khiển PCA9685
#define PCA9685_MODE1 0x00      /**< Thanh ghi chế độ 1 */
#define PCA9685_MODE2 0x01      /**< Thanh ghi chế độ 2 */
#define PCA9685_SUBADR1 0x02    /**< Địa chỉ phụ I2C 1 */
#define PCA9685_SUBADR2 0x03    /**< Địa chỉ phụ I2C 2 */
#define PCA9685_SUBADR3 0x04    /**< Địa chỉ phụ I2C 3 */
#define PCA9685_ALLCALLADR 0x05 /**< Địa chỉ LED All Call I2C */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 bật, byte thấp */
#define PCA9685_LED0_ON_H 0x07  /**< LED0 bật, byte cao */
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 tắt, byte thấp */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 tắt, byte cao */
// Các LED khác: LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< Bật tất cả LEDn, byte thấp */
#define PCA9685_ALLLED_ON_H 0xFB  /**< Bật tất cả LEDn, byte cao */
#define PCA9685_ALLLED_OFF_L 0xFC /**< Tắt tất cả LEDn, byte thấp */
#define PCA9685_ALLLED_OFF_H 0xFD /**< Tắt tất cả LEDn, byte cao */
#define PCA9685_PRESCALE 0xFE     /**< Bộ chia tần số PWM */
#define PCA9685_TESTMODE 0xFF     /**< Chế độ test */

// Các chế độ MODE1
#define MODE1_ALLCAL 0x01  /**< Phản hồi LED All Call I2C */
#define MODE1_SUB3 0x02    /**< Phản hồi địa chỉ phụ 3 */
#define MODE1_SUB2 0x04    /**< Phản hồi địa chỉ phụ 2 */
#define MODE1_SUB1 0x08    /**< Phản hồi địa chỉ phụ 1 */
#define MODE1_SLEEP 0x10   /**< Chế độ tiết kiệm năng lượng, tắt Oscillator */
#define MODE1_AI 0x20      /**< Bật tự động tăng địa chỉ (Auto-Increment) */
#define MODE1_EXTCLK 0x40  /**< Sử dụng clock từ chân EXTCLK */
#define MODE1_RESTART 0x80 /**< Bật chế độ restart */

// Các chế độ MODE2
#define MODE2_OUTNE_0 0x01 /**< Chân output active LOW */
#define MODE2_OUTNE_1 0x02 /**< Chân output active LOW - high impedance */
#define MODE2_OUTDRV 0x04  /**< Totem pole thay vì open-drain */
#define MODE2_OCH 0x08     /**< Output thay đổi khi ACK vs STOP */
#define MODE2_INVRT 0x10   /**< Đảo trạng thái logic đầu ra */

#define PCA9685_I2C_ADDRESS 0x40      // Địa chỉ I2C mặc định
#define FREQUENCY_OSCILLATOR 25000000 /**< Tần số oscillator nội bộ trong datasheet */

#define PCA9685_PRESCALE_MIN 3    // Giá trị prescale nhỏ nhất
#define PCA9685_PRESCALE_MAX 255  // Giá trị prescale lớn nhất

// Lớp điều khiển servo sử dụng PCA9685
class SF_Servo
{
public:
    SF_Servo(TwoWire &i2c);           // Constructor, nhận I2C
    void init();                       // Khởi tạo
    void enable();                     // Bật servo
    void disable();                    // Tắt servo
    void setAngle(uint8_t num, uint16_t angle);   // Đặt góc servo
    void setAngleRange(uint16_t min, uint16_t max); // Đặt giới hạn góc
    void setPluseRange(uint16_t min, uint16_t max); // Đặt giới hạn xung PWM
    void setPWMFreq(float freq);       // Đặt tần số PWM
    void setPin(uint8_t num, uint16_t val, bool invert = false); // Đặt PWM cho 1 kênh
    void setPWM(uint8_t num, uint16_t on, uint16_t off);         // Cài đặt trực tiếp giá trị PWM
    void reset();                      // Reset PCA9685
    void sleep();                      // Chế độ ngủ
    void wakeup();                     // Thức dậy

private:
    uint16_t angleMin, angleMax;       // Giới hạn góc servo
    uint16_t pluseMin, pluseMax;      // Giới hạn xung PWM
    uint16_t angleRange, pluseRange;  // Khoảng góc và xung
    uint8_t freq;                      // Tần số PWM
    float cal_min;                     // Giá trị hiệu chỉnh tối thiểu
    float cal_max;                     // Giá trị hiệu chỉnh tối đa

    TwoWire *_i2c;                     // Con trỏ đến bus I2C

    void writeToPCA(uint8_t addr, uint8_t data); // Ghi dữ liệu vào PCA9685
    uint8_t readFromPCA(uint8_t addr);           // Đọc dữ liệu từ PCA9685
};

#endif
