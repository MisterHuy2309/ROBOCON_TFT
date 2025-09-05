#include "SF_Servo.h"

// Constructor: nhận vào bus I2C và đặt tần số mặc định là 50Hz
SF_Servo::SF_Servo(TwoWire &i2c)
    :  _i2c(&i2c), freq(50){}

// Khởi tạo servo
void SF_Servo::init(){
    _i2c->begin();   // Bắt đầu giao tiếp I2C
    reset();         // Reset PCA9685

    setPWMFreq(freq); // Đặt tần số PWM

    enable();        // Bật servo
}

// Đặt tần số PWM
void SF_Servo::setPWMFreq(float freq){
    if (freq < 1)
        freq = 1;             // Giới hạn tần số nhỏ nhất
    if (freq > 3500)
        freq = 3500;          // Giới hạn tần số lớn nhất (3052 = 50MHz/(4*4096))
    
    float prescaleval = ((FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1; // Tính giá trị prescaler
    if (prescaleval < PCA9685_PRESCALE_MIN)
        prescaleval = PCA9685_PRESCALE_MIN; // Giới hạn min
    if (prescaleval > PCA9685_PRESCALE_MAX)
        prescaleval = PCA9685_PRESCALE_MAX; // Giới hạn max
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = readFromPCA(PCA9685_MODE1); // Đọc chế độ hiện tại
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;  // Chế độ ngủ
    writeToPCA(PCA9685_MODE1, newmode);                         // Vào chế độ ngủ
    writeToPCA(PCA9685_PRESCALE, prescale);                     // Cài prescaler
    writeToPCA(PCA9685_MODE1, oldmode);
    delay(5);
    // Bật chế độ tự động tăng địa chỉ (Auto-increment)
    writeToPCA(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

// Bật servo (chân enable)
void SF_Servo::enable(){
    pinMode(SERVO_ENABLE_PIN, OUTPUT);
    digitalWrite(SERVO_ENABLE_PIN, HIGH);
}

// Tắt servo (chân enable)
void SF_Servo::disable(){
    pinMode(SERVO_ENABLE_PIN, OUTPUT);
    digitalWrite(SERVO_ENABLE_PIN, LOW);
}

// Reset PCA9685
void SF_Servo::reset(){
    writeToPCA(PCA9685_MODE1, MODE1_RESTART);
    delay(10);
}

// Chế độ ngủ
void SF_Servo::sleep(){
    uint8_t awake = readFromPCA(PCA9685_MODE1); // Đọc trạng thái hiện tại
    uint8_t sleep = awake | MODE1_SLEEP;        // Đặt bit sleep
    writeToPCA(PCA9685_MODE1, sleep);
    delay(5);
}

// Thức dậy từ chế độ ngủ
void SF_Servo::wakeup(){
  uint8_t sleep = readFromPCA(PCA9685_MODE1);   // Đọc trạng thái hiện tại
  uint8_t wakeup = sleep & ~MODE1_SLEEP;        // Xóa bit sleep
  writeToPCA(PCA9685_MODE1, wakeup);
}

// Đặt góc servo
void SF_Servo::setAngle(uint8_t num, uint16_t angle){
    if(angle < angleMin || angle > angleMax)      // Kiểm tra giới hạn góc
        return;
    uint16_t offTime = (int)(pluseMin + pluseRange * angle / angleRange); // Tính thời gian xung
    uint16_t off = (int)(offTime * 4096 / 20000);  // Chuyển sang giá trị 12-bit
    setPWM(num, 0, off);                          // Gửi PWM
}

// Cài đặt giới hạn góc servo
void SF_Servo::setAngleRange(uint16_t min, uint16_t max){
    angleMin = (max > min) ? min : max;           // Chọn min thực tế
    angleMax = (max > min) ? max : min;           // Chọn max thực tế
    angleRange = angleMax - angleMin;            // Tính khoảng cách góc
}

// Cài đặt giới hạn xung PWM
void SF_Servo::setPluseRange(uint16_t min, uint16_t max){
    pluseMin = (max > min) ? min : max;           // Chọn min thực tế
    pluseMax = (max > min) ? max : min;           // Chọn max thực tế
    pluseRange = pluseMax - pluseMin;            // Khoảng cách xung
}

// Gửi giá trị PWM trực tiếp cho kênh
void SF_Servo::setPWM(uint8_t num, uint16_t on, uint16_t off){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(PCA9685_LED0_ON_L + 4 * num); // Địa chỉ kênh
    _i2c->write(on);      // Giá trị ON
    _i2c->write(on >> 8); // Giá trị ON cao
    _i2c->write(off);     // Giá trị OFF
    _i2c->write(off >> 8);// Giá trị OFF cao
    _i2c->endTransmission();
}

// Gửi giá trị PWM với tùy chọn đảo chiều
void SF_Servo::setPin(uint8_t num, uint16_t val, bool invert){
    val = min(val, (uint16_t)4095); // Giới hạn 12-bit
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

// Ghi dữ liệu vào thanh ghi PCA9685
void SF_Servo::writeToPCA(uint8_t addr, uint8_t data){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(addr);    // Địa chỉ thanh ghi
    _i2c->write(data);    // Dữ liệu
    _i2c->endTransmission();
}

// Đọc dữ liệu từ thanh ghi PCA9685
uint8_t SF_Servo::readFromPCA(uint8_t addr){
    _i2c->beginTransmission(PCA9685_ADDR);
    _i2c->write(addr);
    _i2c->endTransmission();

    _i2c->requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1);
    return _i2c->read(); // Trả về giá trị đọc được
}
