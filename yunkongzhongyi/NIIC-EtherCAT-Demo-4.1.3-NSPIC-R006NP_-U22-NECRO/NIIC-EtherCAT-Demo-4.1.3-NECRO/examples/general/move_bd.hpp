#pragma once

#include "axis.hpp"
#include <sys/time.h>

enum move_direction : int8_t {
    NEGATIVE = -1,
    POSITIVE = 1
};

class MoveBD
{
public:
    MoveBD() {}
    ~MoveBD() {}

    // EC任务周期内调用，在从站使能后
    void on_cycle();

    // 设置运动模式
    mode_of_operation_type setOpMode(mode_of_operation_type opMode);

    // 不同运动模式下的控制算法
    void move_CSP();
    void move_CSV();
    void move_CST();
    void move_HM();

    // 设置正转或反转
    void setMoveDireaction(move_direction md) { dir = md; } // 逻辑实现，也可以通过PDO改变极性实现反转

    /* ********** 单位换算的简单逻辑实现
    INPUTS
        cmd_pulse: 电机转一周的指令脉冲数
        work_distance: 工作台旋转一周的工作行程/距离
        use_gear: 是否使用变速箱
        gear_n: 工作齿轮比(减速比的分子)
        gear_d: 电机齿轮比(减速比的分母)
    */
    void setResolution(uint64_t cmd_pulse, double work_distance, bool use_gear = false, int gear_n = 1, int gear_d = 1);
public:
    axis_data* axis;
    bool execute = false;
private:
    move_direction dir = POSITIVE;
    double resolution = 0.0;
    bool executing_in_process = false;
    mode_of_operation_type last_op_mode = op_mode_no;

    int32_t curPos = 0;
    int32_t curVel = 0;
    int16_t curTor = 0;

    uint64_t move_cycle_count = 0;
    uint64_t new_mode_mask = 0;

    int32_t vel = 0;
    int32_t acc = 0;
    int16_t cycTor = 0;
};

