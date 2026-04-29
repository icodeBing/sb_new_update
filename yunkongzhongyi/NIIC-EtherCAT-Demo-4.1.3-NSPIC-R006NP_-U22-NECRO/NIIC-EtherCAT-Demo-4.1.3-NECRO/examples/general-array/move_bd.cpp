#include "move_bd.hpp"

#define USE_LOOP 0

move_direction dirArray[] = {
    NEGATIVE, POSITIVE
};

const char* to_string_mode(mode_of_operation_type mode)
{
    switch (mode)
    {
    case mode_of_operation_type::op_mode_no:
        return "NO";
    case mode_of_operation_type::op_mode_pp:
        return "PP";
    case mode_of_operation_type::op_mode_vl:
        return "VL";
    case mode_of_operation_type::op_mode_pv:
        return "PV";
    case mode_of_operation_type::op_mode_hm:
        return "HM";
    case mode_of_operation_type::op_mode_ip:
        return "IP";
    case mode_of_operation_type::op_mode_csp:
        return "CSP";
    case mode_of_operation_type::op_mode_csv:
        return "CSV";
    case mode_of_operation_type::op_mode_cst:
        return "CST";
    default:
        return "invalid";
    }
}


void MoveBD::on_cycle() {
    if (!execute)
        return;

    if (axis->error_code && *axis->error_code) {
        // TODO
        printf("Error code: 0x%x\n", *axis->error_code);
    }

    setMoveDireaction(dirArray[axis->direction]);
    setResolution((1 << axis->motorBit), 100.0);    // 设置电子齿轮比，包括变速箱
    mode_of_operation_type curOpMode = setOpMode(mode_of_operation_type(axis->mode));

    if (!executing_in_process || last_op_mode != curOpMode) {   // 模式发生变化时更新数据
        curPos = axis->position_actual_value == nullptr ? 0 : *axis->position_actual_value;
        curVel = axis->velocity_actual_value == nullptr ? 0 : *axis->velocity_actual_value;
        curTor = axis->torque_actual_value == nullptr ? 0 : *axis->torque_actual_value;
        new_mode_mask = move_cycle_count;
    }

    // if ((move_cycle_count - new_mode_mask) % 1000 == 0)
    //     __RT(printf("[%d] >>>>>>> mode: %s >>>>>> curPos=%d(%d), curVel=%d(%d), curTor=%d(%d)\n",
    //         axis->slave_pos,
    //         to_string_mode(curOpMode),
    //         *axis->position_actual_value, curPos,
    //         *axis->velocity_actual_value, curVel,
    //         *axis->torque_actual_value, curTor));

    switch (curOpMode)  // 不同运动模式使用不同控制算法
    {
    case op_mode_csp:   // CSP模式，使用目标位置
        move_CSP();
        break;
    case op_mode_csv:   // CSV模式，使用目标速度
        move_CSV();
        break;
    case op_mode_cst:   // CST模式，使用目标扭矩
        move_CST();
        break;
    case op_mode_hm:   // HM模式，当前位置回零
        move_HM();
        break;
    default:
        break;
    }

    move_cycle_count++;
    if (move_cycle_count%1000)
    {
        volatile uint16_t* byte_address = (volatile uint16_t*)(axis->position_actual_value);
        uint16_t value = byte_address[0];
        uint16_t value1 = byte_address[1];
        printf("actual_pos::[0]: %d, [1]: %d\n", value, value1);
    }
#if USE_LOOP
    move_cycle_count = move_cycle_count % (new_mode_mask + 600) == 0 ? new_mode_mask : move_cycle_count;
#endif
    last_op_mode = curOpMode;
    executing_in_process = execute;
}


void MoveBD::move_CSP() {
#if USE_LOOP
    int count = move_cycle_count - new_mode_mask;

    if (count < 100 || count >= 500 && count < 600)
    {
      vel += 1;
    }
    else if ((count >= 100 && count < 200) ||
             (count >= 400 && count < 500))
    {

    }
    else if (count >= 200 && count < 400)
    {
      vel -= 1;
    }
#else
    if ((move_cycle_count - new_mode_mask) < 50) //加速
    {
        vel += 2;
    }
#endif

    curPos += static_cast<int32_t>(vel * resolution / axis->freq * dir);
    *axis->target_position = curPos;

}


void MoveBD::move_CSV() {
    if ((move_cycle_count - new_mode_mask) < axis->freq) //加速
    {
        acc += 100;
        curVel = static_cast<int32_t>(acc * resolution / axis->freq * dir);
    }

    vel = axis->velocity_actual_value == nullptr ? curVel : *axis->velocity_actual_value;
    *axis->target_velocity = curVel;
}


void MoveBD::move_CST() {
    if ((move_cycle_count - new_mode_mask) < axis->motorBit)
    {
        curTor += 1;
    }
    *axis->target_torque = static_cast<int16_t>(curTor * dir);
}


void MoveBD::move_HM() {
    if ((*axis->status_word & 0x2000) == 0x2000) {  // 回零失败，自行处理
        // homing error
        *axis->control_word &= ~0x0018;
        printf("State: 0x%04x, control: 0x%04x\n", *axis->status_word, *axis->control_word);
        return;
    }

    *axis->control_word |= 0x0010;

    // if ((*axis->status_word & 0x1400) == 0x1400 && abs(*axis->position_actual_value) < 10) {
    //     // 回零成功后即可切回其他控制模式
    //     printf("State: 0x%04x, Current position: %d, next mode is %d\n",
    //         *axis->status_word, *axis->position_actual_value, axis->mode = 8);
    //     setOpMode(mode_of_operation_type(axis->mode));
    //     *axis->target_position = *axis->position_actual_value;
    // }
}


mode_of_operation_type MoveBD::setOpMode(mode_of_operation_type opMode) {
    if (axis->mode_of_operation) {
        *axis->mode_of_operation = opMode;
    }
    if (axis->mode_of_operation_display) {
        return mode_of_operation_type(*axis->mode_of_operation_display);
    }
    return opMode;
}


void MoveBD::setResolution(uint64_t cmd_pulse, double work_distance, bool use_gear, int gear_n, int gear_d) {
    if (use_gear)
        resolution = static_cast<double>(cmd_pulse * gear_d / (work_distance * gear_n));
    else
        resolution = static_cast<double>(cmd_pulse / work_distance);
}
