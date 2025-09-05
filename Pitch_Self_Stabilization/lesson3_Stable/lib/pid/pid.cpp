#include "pid.h"
#include <Arduino.h>

// Macro giới hạn giá trị giữa low và high
#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Constructor khởi tạo PID
PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // Giới hạn tốc độ thay đổi output [volts/giây]
    , limit(limit)         // Giới hạn giá trị output tối đa [volts]
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = micros(); // Lưu thời điểm khởi tạo
}

// Hàm PID: truyền vào lỗi và nhận giá trị output
float PIDController::operator() (float error){
    // Tính khoảng thời gian từ lần gọi trước
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f; // chuyển sang giây
    // Khắc phục trường hợp lạ (micros overflow)
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // Phần tỉ lệ
    float proportional = P * error;

    // Phần tích phân (Tustin transform)
    float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);

    // Anti-windup: giới hạn giá trị tích phân
    integral = _constrain(integral, -limit/3, limit/3);

    // Phần vi phân
    float derivative = D * (error - error_prev) / Ts;

    // Tổng hợp output
    float output = proportional + integral + derivative;

    // Giới hạn output
    output = _constrain(output, -limit, limit);

    // Nếu định nghĩa ramp (giới hạn tăng tốc output)
    if(output_ramp > 0){
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;
    }

    // Lưu giá trị lần trước để lần sau tính toán
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;

    return output;
}
