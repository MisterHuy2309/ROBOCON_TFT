#ifndef _BIPEDAL_DATA_h
#define _BIPEDAL_DATA_h

#define SBUSPIN 41 // Chân nối tín hiệu SBUS từ remote
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // Hàm giới hạn giá trị

#define RCCHANNEL_MIN 600      // Giá trị tối thiểu kênh RC
#define RCCHANNEL_MAX 1400     // Giá trị tối đa kênh RC
#define RCCHANNEL3_MIN 200     // Giá trị tối thiểu kênh 3
#define RCCHANNEL3_MID 1000    // Giá trị trung gian kênh 3
#define RCCHANNEL3_MAX 1800    // Giá trị tối đa kênh 3

#define LFSERVO_CH 3  // Kênh servo chân trái trước
#define LRSERVO_CH 4  // Kênh servo chân trái sau
#define RFSERVO_CH 5  // Kênh servo chân phải trước
#define RRSERVO_CH 6  // Kênh servo chân phải sau

#define ROBOTMODE_DIABLE 0       // Chế độ tắt robot
#define ROBOTMODE_MOTION 1       // Chế độ di chuyển
#define ROBOTMODE_CALIBRATION 2  // Chế độ calibrate

#define ROBOT_HIGHEST 130        // Chiều cao tối đa
#define ROBOT_LOWEST_FOR_MOT 70  // Chiều cao tối thiểu khi di chuyển
#define ROBOT_LOWESR_FOR_CAL 70  // Chiều cao tối thiểu khi calibrate

// Chiều dài các khớp chân (mm)
#define L1  60
#define L2  100
#define L3  100
#define L4  60
#define L5  40
#define L6  0

// Offset servo để bù cơ khí
#define LFSERVO_OFFSET 0
#define LRSERVO_OFFSET 10
#define RFSERVO_OFFSET -8
#define RRSERVO_OFFSET 10

int RCValue[6]; // Mảng lưu tín hiệu từ 6 kênh RC

// -------------------- Struct --------------------

// Trạng thái tư thế robot
typedef struct {
    float pitch;     // Góc nghiêng tới/lui (deg)
    float roll;      // Góc nghiêng trái/phải (deg)
    float yaw;       // Góc quay (deg)
    float GyroX;     // Gia tốc góc X từ IMU
    float GyroY;     // Gia tốc góc Y từ IMU
    float GyroZ;     // Gia tốc góc Z từ IMU
    float height;    // Chiều cao robot
    float speedAvg;  // Tốc độ trung bình
} robotposeparam;

// Tham số chuyển động robot
typedef struct {
    uint8_t mode;       // Chế độ robot

    float forward;      // Dùng cho điều khiển bánh xe và giải bài toán IK
    float turn;         // Chỉ dùng cho giải IK
    float updown;       // Chỉ dùng cho giải IK
    float roll;         // Chỉ dùng cho giải IK

    float throttleLimit; // Giới hạn ga (tốc độ đầu vào)
    float turnLimit;     // Giới hạn góc quay
    float forwardLimit;  // Giới hạn nghiêng tới/lui
    float rollLimit;     // Giới hạn nghiêng trái/phải
    uint8_t heightest;   // Chiều cao tối đa
    uint8_t lowest;      // Chiều cao tối thiểu
} robotmotionparam;

// Chế độ hoạt động robot
typedef struct {
    uint8_t mode;        // Chế độ hiện tại
    bool printFlag;      // Bật/tắt in debug
    bool motorEnable;    // Bật/tắt motor
    bool servoEnable;    // Bật/tắt servo
} robotmode;

// Thông số PID
typedef struct {
    float Kp;  // Hằng số P
    float Ki;  // Hằng số I
    float Kd;  // Hằng số D
} PIDIncrement;

// Mục tiêu servo và motor
typedef struct {
    uint16_t servoLeftFront;  // Góc servo trái trước
    uint16_t servoLeftRear;   // Góc servo trái sau
    uint16_t servoRightFront; // Góc servo phải trước
    uint16_t servoRightRear;  // Góc servo phải sau
    float motorLeft;           // Tốc độ motor trái
    float motorRight;          // Tốc độ motor phải
} motorstarget;

// Trạng thái motor
typedef struct {
    float M0Speed;     // Tốc độ motor 0
    float M1Speed;     // Tốc độ motor 1
    int M0Dir;         // Hướng motor 0
    int M1Dir;         // Hướng motor 1
    int M0SpdDir;      // Hướng tốc độ motor 0
    int M1SpdDir;      // Hướng tốc độ motor 1
} motorstatus;

// Tham số điều khiển tổng thể
typedef struct {
    float height;        // Chiều cao robot
    float legRollLeft;   // Góc nghiêng chân trái
    float legRollRight;  // Góc nghiêng chân phải
    float legLeft;       // Vị trí chân trái
    float legRight;      // Vị trí chân phải
    float roll;          // Góc nghiêng tổng thể
    float forward;       // Góc nghiêng tới/lui tổng thể
    float pitch;         // Góc pitch tổng thể
    float velocity;      // Vận tốc hiện tại
    float differVel;     // Sai khác vận tốc
    float centerAngleOffset; // Offset góc trung tâm
} controlparam;

// Tọa độ chân
typedef struct {
    float x;       // Tọa độ X trung tâm
    float xLeft;   // Tọa độ X chân trái
    float xRight;  // Tọa độ X chân phải
    float yLeft;   // Tọa độ Y chân trái
    float yRight;  // Tọa độ Y chân phải
} coordinate;

// Tham số động học ngược (IK)
typedef struct {
    float alphaLeft;   // Góc khớp α chân trái
    float betaLeft;    // Góc khớp β chân trái
    float alphaRight;  // Góc khớp α chân phải
    float betaRight;   // Góc khớp β chân phải
    float XLeft;       // Tọa độ X chân trái
    float YLeft;       // Tọa độ Y chân trái
    float XRight;      // Tọa độ X chân phải
    float YRight;      // Tọa độ Y chân phải
} IKparam;

#endif // _BIPEDAL_DATA_h
