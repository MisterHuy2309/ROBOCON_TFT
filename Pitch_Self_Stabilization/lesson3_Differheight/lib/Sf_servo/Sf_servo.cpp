#include "SF_Servo.h"

// ------------------------ Constructor ------------------------
SF_Servo::SF_Servo(TwoWire &i2c)
    : _i2c(&i2c) {}

// ------------------------ Khởi tạo servo ------------------------
void SF_Servo::init(){
    _i2c->begin();   // Khởi tạo bus I2C
    reset();         // Reset PCA9685
    setPWMFreq(50);  // Thiết lập tần số PWM mặc định 50Hz
    enable();        // Bật servo
}

// ------------------------ Thiết lập tần số PWM ------------------------
void SF_Servo::setPWMFreq(float freq){
    if (freq < 1) freq = 1;
    if (freq > 3500) freq = 3500;  // Giới hạn tần số tối đa theo datasheet
    
    float prescaleval = ((FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1; // Tính prescale
    if (prescaleval < PCA9685_PRESCALE_MIN) prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX) prescaleval = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = readFromPCA(PCA9685_MODE1);              // Đọc mode hiện tại
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // Sleep trước khi set prescale
    writeToPCA(PCA9685_MODE1, newmode);                         // Ghi mode sleep
    writeToPCA(PCA9685_PRESCALE, prescale);                     // Ghi prescale
    writeToPCA(PCA9685_MODE1, oldmode);                         // Quay lại mode cũ
    delay(5);
    writeToPCA(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI); // Bật auto-increment
}

// ------------------------ Bật / tắt servo ------------------------
void SF_Servo::enable(){
    pinMode(SERVO_ENABLE_PIN, OUTPUT);
    digitalWrite(SERVO_ENABLE_PIN, HIGH);
}

void SF_Servo::disable(){
    pinMode(SERVO_ENABLE_PIN, OUTPUT);
    digitalWrite(SERVO_ENABLE_PIN, LOW);
}

// ------------------------ Reset PCA9685 ------------------------
void SF_Servo::reset(){
    writeToPCA(PCA9685_MODE1, MODE1_RESTART);
    delay(10);
}

// ------------------------ Chế độ ngủ / thức dậy ------------------------
void SF_Servo::sleep(){
    uint8_t awake = readFromPCA(PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP; // Set sleep bit
    writeToPCA(PCA9685_MODE1, sleep);
    delay(5);
}

void SF_Servo::wakeup(){
    uint8_t sleep = readFromPCA(PCA9685_MODE1);
    uint8_t wakeup = sleep & ~MODE1_SLEEP; // Clear sleep bit
    writeToPCA(PCA9685_MODE1, wakeup);
}

// ------------------------ Thiết lập góc servo ------------------------
void SF_Servo::setAngle(uint8_t num, uint16_t val){
    // Map góc từ dải min-max sang giá trị PWM 0~4095
    uint16_t angle = (int)map(val, _min, _max, cal_min / 20 * 4095, cal_max / 20 * 4095);
    setPWM(num, 0, angle);
}

// Thiết lập dải góc min-max và hiệu chuẩn
void SF_Servo::setAngleRange(uint8_t min, uint8_t max){
    _min = min;
    _max = max;

    cal_min = 0.5 / 45 * min + 0.5; // Hiệu chuẩn min
    cal_max = 0.5 / 45 * max + 0.5; // Hiệu chuẩn max
}

// ------------------------ Thiết lập PWM trực tiếp ------------------------
void SF_Servo::setPWM(uint8_t num, uint16_t on, uint16_t off){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(PCA9685_LED0_ON_L + 4 * num); // Tính offset thanh ghi LED
    _i2c->write(on);
    _i2c->write(on >> 8);
    _i2c->write(off);
    _i2c->write(off >> 8);
    _i2c->endTransmission();
}

// Thiết lập pin PWM với tùy chọn đảo
void SF_Servo::setPin(uint8_t num, uint16_t val, bool invert){
    val = min(val, (uint16_t)4095); // Giới hạn 0~4095
    if (invert) {
        if (val == 0) setPWM(num, 4096, 0);
        else if (val == 4095) setPWM(num, 0, 4096);
        else setPWM(num, 0, 4095 - val);
    } else {
        if (val == 4095) setPWM(num, 4096, 0);
        else if (val == 0) setPWM(num, 0, 4096);
        else setPWM(num, 0, val);
    }
}

// ------------------------ Ghi / đọc dữ liệu I2C ------------------------
void SF_Servo::writeToPCA(uint8_t addr, uint8_t data){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(addr);
    _i2c->write(data);
    _i2c->endTransmission();
}

uint8_t SF_Servo::readFromPCA(uint8_t addr){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(addr);
    _i2c->endTransmission();

    _i2c->requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1);
    return _i2c->read();
}
