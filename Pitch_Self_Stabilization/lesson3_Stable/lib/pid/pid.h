#ifndef PID_H
#define PID_H

/**
 * PID controller class
 * Lớp điều khiển PID cơ bản
 */
class PIDController
{
public:
    /**
     * Constructor
     * @param P - Hệ số tỉ lệ (Proportional gain)
     * @param I - Hệ số tích phân (Integral gain)
     * @param D - Hệ số vi phân (Derivative gain)
     * @param ramp - Tốc độ thay đổi tối đa của giá trị output
     * @param limit - Giá trị output tối đa
     */
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    // Cho phép gọi đối tượng PID như một hàm, truyền lỗi vào để nhận output
    float operator() (float error);

    float P; //!< Hệ số tỉ lệ
    float I; //!< Hệ số tích phân
    float D; //!< Hệ số vi phân
    float output_ramp; //!< Tốc độ thay đổi tối đa của output
    float limit;       //!< Output tối đa

protected:
    float error_prev;     //!< Giá trị lỗi lần trước
    float output_prev;    //!< Giá trị output lần trước
    float integral_prev;  //!< Giá trị tích phân lần trước
    unsigned long timestamp_prev; //!< Thời điểm thực thi lần trước
};

#endif // PID_H
