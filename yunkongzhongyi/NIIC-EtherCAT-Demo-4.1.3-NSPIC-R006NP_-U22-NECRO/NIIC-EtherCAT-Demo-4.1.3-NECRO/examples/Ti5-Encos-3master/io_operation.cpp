// 英克斯电机控制
#include "io_operation.hpp"
#include "math_ops.hpp"
#include "axis.hpp"

#include <chrono>
#include <iostream>

#include "interface.hpp"

encos_axis_data encos_axis;
Motor_Ctrl_Mode_type m_Motor_Ctrl_Mode = Motor_Ctrl_Mode_type::Force_Pos_Mode; // 默认力位混控模式，如果需要调整电机控制模式，修改改变量即可

void IoController::on_cycle_encos(int slave_pos)
{
    /**
     * @brief 力位混合模式
     *
     * 使用问答模式
     *
     * @param tx_msg 存储构建CAN消息的指针。
     * @param passage   电机通道编号，CAN1（1，2，3），CAN2（4，5，6）。
     * @param motor_id  电机唯一标识符（ID）。
     * @param kp        参数kp范围：0.0 ~ 500.0f。
     * @param kd        参数kd范围：0.0 ~ 5.0f。
     * @param pos       期望位置，范围：-12.5 ~ 12.5rad。
     * @param spd       期望速度，范围：-18.0 ~ 18.0rad/s。
     * @param tor       电机前馈扭矩，范围：-30 ~ 30Nm。
     *
     * @return None
     */
    // send_motor_ctrl_cmd(&Tx_Message, 1,1, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 2,2, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 3,3, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 4,4, 0, 0, 0, 0, 3, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 5,5, 0, 0, 0, 0, 6, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 6,6, 0, 0, 0, 0, 9, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message,pas ,id, kp, kd, pos, spd, torq, slave_pos);

    /**
     * @brief 伺服位置控制
     *
     * 使用问答模式
     *
     * @param tx_msg 存储构建CAN消息的指针。
     * @param passage   电机通道编号，CAN1（1，2，3），CAN2（4，5，6）。
     * @param motor_id  电机的唯一标识符（ID）。
     * @param pos       期望位置，单位为度。
     * @param spd       期望速度，范围 0 ~ 18000，对应 0 ~ 1800.0rpm，比例为10：1
     * @param cur       电流阈值，范围 0 ~ 3000，对应 0 ~ 300.0A，比例为10：1
     * @param ack_status 报文返回状态，0：不返回，1：返回报文类型1，2：返回报文类型2，3：返回报文类型3。详见技术文档-问答模式反馈报文章节。
     */

    // set_motor_position(tx_msg, passage,motor_id,  pos,  spd,  cur,  ack_status)
    // set_motor_position(&Tx_Message,1,1,1000,50,5,slave_pos);
    // set_motor_position(&Tx_Message,2,2,1000,50,5,slave_pos);
    // set_motor_position(&Tx_Message,3,3,1000,50,5,slave_pos);
    // set_motor_position(&Tx_Message,4,4,1000,50,5,slave_pos);
    // set_motor_position(&Tx_Message,5,5,180,120,5,1);  // di 3 wei biao shi jue dui jiao du 给定期望位置大于±180 度，即可多圈转，但是重启上电会重置圈数。
    // set_motor_position(&Tx_Message,6,6,1000,50,5,slave_pos);

    /**
     * @brief 伺服速度控制
     *
     * 使用问答模式
     *
     * @param tx_msg 存储构建CAN消息的指针。
     * @param passage   电机的通道编号，CAN1（1，2，3），CAN2（4，5，6）。
     * @param motor_id  电机的唯一标识符（ID）。
     * @param spd       期望速度，范围 -18000 ~ 18000rpm。
     * @param cur       电流阈值，范围 0 ~ 3000, 对应 0 ~ 300.0A，比例10：1。
     * @param ack_status 报文返回状态，0：不返回，1~3：返回报文类型1~3。详见技术文档-问答模式反馈报文章节。
     */

    // void set_motor_speed(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status)

    /**
     * @brief 电流、力矩控制和刹车控制。
     *
     * 需要使用问答模式
     *
     * @param tx_msg 存储构建CAN消息的指针。
     * @param passage   电机的通道编号，CAN1（1，2，3），CAN2（4，5，6）。
     * @param motor_id  电机的唯一标识符（ID）。
     * @param cur_tor   期望电流/力矩，范围 -3000 ~ 3000, 对应 -30.00A(Nm) ~ 30.00A(Nm)，比例100：1。
     * @param ctrl_status 控制状态，0：电流控制，1：力矩控制，2：变量阻尼制动控制（也叫全制动），3：能耗制动控制，4：再生制动控制。
     * @param ack_status 报文返回状态，0：不返回，1~3：返回报文类型1~3。详见技术文档-问答模式反馈报文章节。
     */

    // set_motor_cur_tor(tx_msg, pasage, motor_id, cur_tor, ctrl_status, ack_status)
    // set_motor_cur_tor(&Tx_Message, 1, 1, 600, 1, 1);
    // set_motor_cur_tor(&Tx_Message, 2, 2, 600, 1, 1);
    // set_motor_cur_tor(&Tx_Message, 3, 3, 600, 1, 1);

    static bool reading_motor_id_falg = false; // true
    static bool setting_motor_id_falg = false; // false

    if (reading_motor_id_falg | setting_motor_id_falg)
    { // 设置ID，如果是新电机，需要进行ID设置
        if (slave_pos == 0)
        {
            if (reading_motor_id_falg)
            {
                MotorIDReading(&Tx_Message, 1);
                MotorIDReading(&Tx_Message, 2);
                MotorIDReading(&Tx_Message, 3);

                for (auto &io : tx_)
                {
                    printIO(io.get());
                }
                NECRO_RT(printf("io_end!!!!!!\n"));
            }

            if (setting_motor_id_falg)
            {
                MotorIDSetting(&Tx_Message, 1, 3, 1);
                //    MotorIDSetting(&Tx_Message, 4, 1, 2);
                //    MotorIDSetting(&Tx_Message, 4, 15, 3);
                for (auto &io : tx_)
                {
                    printIO(io.get());
                }
                NECRO_RT(printf("io_end!!!!!!\n"));
            }
        }
    }
    else if (!reading_motor_id_falg && !setting_motor_id_falg) // control_motor
    {                                                          // ID设置好后，进行电机控制程序
        if (slave_pos == 0)
        { // slave_pos=0表示主站0连接的ethercat转can模块的第一块板子，带6个电机

            if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
            { // 采用位置控制模式控制电机
                set_motor_position(&Tx_Message, 1, 1, (motor_msg.master_id[0].motor_id[0].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 2, 2, (motor_msg.master_id[0].motor_id[1].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 3, 3, (motor_msg.master_id[0].motor_id[2].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 4, 4, (motor_msg.master_id[0].motor_id[3].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 5, 5, (motor_msg.master_id[0].motor_id[4].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 6, 6, (motor_msg.master_id[0].motor_id[5].position_cmd) / 1000, 50, 300, 1);
            }
            else if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Force_Pos_Mode)
            { // 采用力位混控模式控制电机
                // std::cout << "tor_4:" << motor_msg.master_id[0].motor_id[4].Tor_cmd / 1000.0f <<std::endl;
                // std::cout << "tor_5:" << motor_msg.master_id[0].motor_id[5].Tor_cmd / 1000.0f <<std::endl;

                for (int i = 0; i < 6; i++)
                {
                    // std::cout << "kp_cmd " << (motor_msg.master_id[0].motor_id[i].kp_cmd / 1000.0f) << "  kd_cmd " << (motor_msg.master_id[0].motor_id[i].kd_cmd / 1000.0f) << std::endl;
                    send_motor_ctrl_cmd(&Tx_Message, i + 1, i + 1,
                                        motor_msg.master_id[0].motor_id[i].kp_cmd / 1000.0f,
                                        motor_msg.master_id[0].motor_id[i].kd_cmd / 1000.0f,
                                        (motor_msg.master_id[0].motor_id[i].position_cmd) / 1000.0f,
                                        motor_msg.master_id[0].motor_id[i].velocity_cmd,
                                        (motor_msg.master_id[0].motor_id[i].Tor_cmd) / 1000.0f,
                                        slave_pos);
                }
            }
            // 读取电机实时值
            for (auto &io : tx_)
            {
                // printIO(io.get());
                encons_get_angle_slave0(io.get());
                encons_get_angular_velocity_slave0(io.get());
                encons_get_actual_current_slave0(io.get());
            }
            // printf("--------end--------\n");
            motor_msg.master_id[0].motor_id[0].position_actual = encos_axis.encos_angle_actual_value[0];
            motor_msg.master_id[0].motor_id[0].velocity_actual = encos_axis.encos_angular_velocity_actual_value[0];
            motor_msg.master_id[0].motor_id[0].current_actual = encos_axis.encos_current_actual_value[0];

            motor_msg.master_id[0].motor_id[1].position_actual = encos_axis.encos_angle_actual_value[1];
            motor_msg.master_id[0].motor_id[1].velocity_actual = encos_axis.encos_angular_velocity_actual_value[1];
            motor_msg.master_id[0].motor_id[1].current_actual = encos_axis.encos_current_actual_value[1];

            motor_msg.master_id[0].motor_id[2].position_actual = encos_axis.encos_angle_actual_value[2];
            motor_msg.master_id[0].motor_id[2].velocity_actual = encos_axis.encos_angular_velocity_actual_value[2];
            motor_msg.master_id[0].motor_id[2].current_actual = encos_axis.encos_current_actual_value[2];

            motor_msg.master_id[0].motor_id[3].position_actual = encos_axis.encos_angle_actual_value[3];
            motor_msg.master_id[0].motor_id[3].velocity_actual = encos_axis.encos_angular_velocity_actual_value[3];
            motor_msg.master_id[0].motor_id[3].current_actual = encos_axis.encos_current_actual_value[3];

            motor_msg.master_id[0].motor_id[4].position_actual = encos_axis.encos_angle_actual_value[4];
            motor_msg.master_id[0].motor_id[4].velocity_actual = encos_axis.encos_angular_velocity_actual_value[4];
            motor_msg.master_id[0].motor_id[4].current_actual = encos_axis.encos_current_actual_value[4];

            motor_msg.master_id[0].motor_id[5].position_actual = encos_axis.encos_angle_actual_value[5];
            motor_msg.master_id[0].motor_id[5].velocity_actual = encos_axis.encos_angular_velocity_actual_value[5];
            motor_msg.master_id[0].motor_id[5].current_actual = encos_axis.encos_current_actual_value[5];

            // NECRO_RT(printf("slave left start #################################### \n"));
            // std::cout << "torque_value 1: " << motor_msg.master_id[0].motor_id[0].current_actual << std::endl;
            // std::cout << "torque_value 2: " << motor_msg.master_id[0].motor_id[1].current_actual << std::endl;
            // std::cout << "torque_value 3: " << motor_msg.master_id[0].motor_id[2].current_actual << std::endl;
            // std::cout << "torque_value 4: " << motor_msg.master_id[0].motor_id[3].current_actual << std::endl;
            // std::cout << "torque_value 5: " << motor_msg.master_id[0].motor_id[4].current_actual << std::endl;
            // std::cout << "torque_value 6: " << motor_msg.master_id[0].motor_id[5].current_actual << std::endl;
            // std::cout << "  " << std::endl;
            // NECRO_RT(printf("slave left end #################################### \n"));

            // NECRO_RT(printf("slave left start #################################### \n"));
            // std::cout << "torque_value 1: " << get_actual_current_0_3(encos_axis.encos_current_actual_value[0]) << std::endl;
            // std::cout << "torque_value 2: " << get_actual_current_1(encos_axis.encos_current_actual_value[1]) << std::endl;
            // std::cout << "torque_value 3: " << get_actual_current_2(encos_axis.encos_current_actual_value[2]) << std::endl;
            // std::cout << "torque_value 4: " << get_actual_current_0_3(encos_axis.encos_current_actual_value[3]) << std::endl;
            // std::cout << "torque_value 5: " << get_actual_current_4_5(encos_axis.encos_current_actual_value[4]) << std::endl;
            // std::cout << "torque_value 6: " << get_actual_current_4_5(encos_axis.encos_current_actual_value[5]) << std::endl;
            // std::cout << "  " << std::endl;

            // NECRO_RT(printf("slave left end #################################### \n"));

            // NECRO_RT(printf("slave 0 start #################################### \n"));

            // std::cout<<"angular_velocity_0: "<<encos_axis.encos_angular_velocity_actual_value[0]<<std::endl;
            // std::cout<<"angular_velocity_1: "<<encos_axis.encos_angular_velocity_actual_value[1]<<std::endl;
            // std::cout<<"angular_velocity_2: "<<encos_axis.encos_angular_velocity_actual_value[2]<<std::endl;
            // std::cout<<"angular_velocity_3: "<<encos_axis.encos_angular_velocity_actual_value[3]<<std::endl;
            // std::cout<<"angular_velocity_4: "<<encos_axis.encos_angular_velocity_actual_value[4]<<std::endl;
            // std::cout<<"angular_velocity_5: "<<encos_axis.encos_angular_velocity_actual_value[5]<<std::endl;

            // std::cout<<"angle_value_0: "<<encos_axis.encos_angle_actual_value[0]<<std::endl;
            // std::cout<<"angle_value_1: "<<encos_axis.encos_angle_actual_value[1]<<std::endl;
            // std::cout<<"angle_value_2: "<<encos_axis.encos_angle_actual_value[2]<<std::endl;
            // std::cout<<"angle_value_3: "<<encos_axis.encos_angle_actual_value[3]<<std::endl;
            // std::cout<<"angle_value_4: "<<encos_axis.encos_angle_actual_value[4]<<std::endl;
            // std::cout<<"angle_value_5: "<<encos_axis.encos_angle_actual_value[5]<<std::endl;

            // NECRO_RT(printf("slave 0 end #################################### \n"));
        }
        if (slave_pos == 1)
        { // slave_pos=1表示主站1连接的ethercat转can模块的第二块板子，带6个电机

            if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
            { // 采用位置控制模式控制电机
                set_motor_position(&Tx_Message, 1, 1, (motor_msg.master_id[0].motor_id[6].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 2, 2, (motor_msg.master_id[0].motor_id[7].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 3, 3, (motor_msg.master_id[0].motor_id[8].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 4, 4, (motor_msg.master_id[0].motor_id[9].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 5, 5, (motor_msg.master_id[0].motor_id[10].position_cmd) / 1000, 50, 300, 1);
                set_motor_position(&Tx_Message, 6, 6, (motor_msg.master_id[0].motor_id[11].position_cmd) / 1000, 50, 300, 1);
            }
            else if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Force_Pos_Mode)
            { // 采用力位混控模式控制电机
                for (int i = 0; i < 6; i++)
                {
                    // 打印下发数据是否正确
                    // std::cout << "kp_cmd " << (motor_msg.master_id[0].motor_id[i + 6].kp_cmd / 1000.0f) << "  kd_cmd " << (motor_msg.master_id[0].motor_id[i + 6].kd_cmd / 1000.0f) << std::endl;
                    send_motor_ctrl_cmd(&Tx_Message, i + 1, i + 1,
                                        motor_msg.master_id[0].motor_id[i + 6].kp_cmd / 1000.0f,
                                        motor_msg.master_id[0].motor_id[i + 6].kd_cmd / 1000.0f,
                                        (motor_msg.master_id[0].motor_id[i + 6].position_cmd) / 1000.0f,
                                        motor_msg.master_id[0].motor_id[i + 6].velocity_cmd,
                                        (motor_msg.master_id[0].motor_id[i + 6].Tor_cmd) / 1000.0f,
                                        slave_pos);
                }
            }
            // 读取电机实时值
            for (auto &io : tx_)
            {
                //   printIO(io.get());
                encons_get_angle_slave1(io.get());
                encons_get_angular_velocity_slave1(io.get());
                encons_get_actual_current_slave1(io.get());
            }

            motor_msg.master_id[0].motor_id[6].position_actual = encos_axis.encos_angle_actual_value[6];
            motor_msg.master_id[0].motor_id[6].velocity_actual = encos_axis.encos_angular_velocity_actual_value[6];
            motor_msg.master_id[0].motor_id[6].current_actual = encos_axis.encos_current_actual_value[6];

            motor_msg.master_id[0].motor_id[7].position_actual = encos_axis.encos_angle_actual_value[7];
            motor_msg.master_id[0].motor_id[7].velocity_actual = encos_axis.encos_angular_velocity_actual_value[7];
            motor_msg.master_id[0].motor_id[7].current_actual = encos_axis.encos_current_actual_value[7];

            motor_msg.master_id[0].motor_id[8].position_actual = encos_axis.encos_angle_actual_value[8];
            motor_msg.master_id[0].motor_id[8].velocity_actual = encos_axis.encos_angular_velocity_actual_value[8];
            motor_msg.master_id[0].motor_id[8].current_actual = encos_axis.encos_current_actual_value[8];

            motor_msg.master_id[0].motor_id[9].position_actual = encos_axis.encos_angle_actual_value[9];
            motor_msg.master_id[0].motor_id[9].velocity_actual = encos_axis.encos_angular_velocity_actual_value[9];
            motor_msg.master_id[0].motor_id[9].current_actual = encos_axis.encos_current_actual_value[9];

            motor_msg.master_id[0].motor_id[10].position_actual = encos_axis.encos_angle_actual_value[10];
            motor_msg.master_id[0].motor_id[10].velocity_actual = encos_axis.encos_angular_velocity_actual_value[10];
            motor_msg.master_id[0].motor_id[10].current_actual = encos_axis.encos_current_actual_value[10];

            motor_msg.master_id[0].motor_id[11].position_actual = encos_axis.encos_angle_actual_value[11];
            motor_msg.master_id[0].motor_id[11].velocity_actual = encos_axis.encos_angular_velocity_actual_value[11];
            motor_msg.master_id[0].motor_id[11].current_actual = encos_axis.encos_current_actual_value[11];

            // NECRO_RT(printf("slave left start #################################### \n"));
            // std::cout << "torque_value 7: " << motor_msg.master_id[0].motor_id[6].current_actual << std::endl;
            // std::cout << "torque_value 8: " << motor_msg.master_id[0].motor_id[7].current_actual << std::endl;
            // std::cout << "torque_value 9: " << motor_msg.master_id[0].motor_id[8].current_actual << std::endl;
            // std::cout << "torque_value 10: " << motor_msg.master_id[0].motor_id[9].current_actual << std::endl;
            // std::cout << "torque_value 11: " << motor_msg.master_id[0].motor_id[10].current_actual << std::endl;
            // std::cout << "torque_value 12: " << motor_msg.master_id[0].motor_id[11].current_actual << std::endl;
            // std::cout << "  " << std::endl;
            // NECRO_RT(printf("slave left end #################################### \n"));

            //  NECRO_RT(printf("slave right start #################################### \n"));
            // std::cout << "torque_value 7: " << get_actual_current_0_3(encos_axis.encos_current_actual_value[6]) << std::endl;
            // std::cout << "torque_value 8: " << get_actual_current_1(encos_axis.encos_current_actual_value[7]) << std::endl;
            // std::cout << "torque_value 9: " << get_actual_current_2(encos_axis.encos_current_actual_value[8]) << std::endl;
            // std::cout << "torque_value 10: " << get_actual_current_0_3(encos_axis.encos_current_actual_value[9]) << std::endl;
            // std::cout << "torque_value 11: " << get_actual_current_4_5(encos_axis.encos_current_actual_value[10]) << std::endl;
            // std::cout << "torque_value 12: " << get_actual_current_4_5(encos_axis.encos_current_actual_value[11]) << std::endl;
            // std::cout << "  " << std::endl;

            // NECRO_RT(printf("slave right end #################################### \n"));
            // NECRO_RT(printf("slave 1 start #################################### \n"));

            // std::cout<<"angle_value_6: "<<encos_axis.encos_angle_actual_value[1]<<std::endl;
            // std::cout<<"angle_value_1: "<<encos_axis.encos_angle_actual_value[1]<<std::endl;
            // std::cout<<"angle_value_5: "<<encos_axis.encos_angle_actual_value[2]<<std::endl;
            // std::cout<<"angle_value_3: "<<encos_axis.encos_angle_actual_value[3]<<std::endl;
            // std::cout<<"angle_value_6: "<<encos_axis.encos_angle_actual_value[4]<<std::endl;
            // std::cout<<"  "<<std::endl;

            // NECRO_RT(printf("slave 1 end #################################### \n"));
        }
    }
    // encos
}

// encos
void IoController::send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor, int slave_pos)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;

    TxMessage->can_ide = 1;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 8;

    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    else if (kd < KD_MIN)
        kd = KD_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > SPD_MAX)
        spd = SPD_MAX;
    else if (spd < SPD_MIN)
        spd = SPD_MIN;
    if (tor > T_MAX)
        tor = T_MAX;
    else if (tor < T_MIN)
        tor = T_MIN;

    // // 不同电机型号对应不同tor范围
    // if (motor_id == 1 || motor_id == 4)
    // {
    //     if (tor > TMAX_10020)
    //         tor = TMAX_10020;
    //     else if (tor < TMIN_10020)
    //         tor = TMIN_10020;
    // }
    // else if (motor_id == 2)
    // {
    //     if (tor > IMAX_8116)
    //         tor = IMAX_8116;
    //     else if (tor < IMIN_8116)
    //         tor = IMIN_8116;
    // }
    // else if (motor_id == 3)
    // {
    //     if (tor > TMAX_6408)
    //         tor = TMAX_6408;
    //     else if (tor < TMIN_6408)
    //         tor = TMIN_6408;
    // }
    // else if (motor_id == 5 || motor_id == 6)
    // {
    //     if (tor > IMAX_4315)
    //         tor = IMAX_4315;
    //     else if (tor < IMIN_4315)
    //         tor = IMIN_4315;
    // }

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);

    // tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);
    if (motor_id == 1 || motor_id == 4)
    {
        tor_int = float_to_uint(tor, TMIN_10020, TMAX_10020, 12);
        kd_int  = float_to_uint(kd, KD_MIN_10020, KD_MAX_10020, 9);
    }
    else if (motor_id == 2)
    {
        tor_int = float_to_uint(tor, IMIN_8116, IMAX_8116, 12);
    }
    else if (motor_id == 3)
    {
        tor_int = float_to_uint(tor, TMIN_6408, TMAX_6408, 12);
    }
    else if (motor_id == 5 || motor_id == 6)
    {
        tor_int = float_to_uint(tor, IMIN_4315, IMAX_4315, 12);
    }

    // printf("\n");
    // printf("-----------------------send_motor_ctrl_cmd----------------start--------------------------\n");
    // printf("  tor_int_id %d：%d\ttor_float:%lf\n",motor_id,tor_int, tor);

    // // printf("-----------------------send_motor_ctrl_cmd----------------end--------------------------\n");
    // printf("\n");

    TxMessage->motor[passage - 1].data[0] = 0x00 | (kp_int >> 7);                             // kp5
    TxMessage->motor[passage - 1].data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    TxMessage->motor[passage - 1].data[2] = kd_int & 0xFF;
    TxMessage->motor[passage - 1].data[3] = pos_int >> 8;
    TxMessage->motor[passage - 1].data[4] = pos_int & 0xFF;
    TxMessage->motor[passage - 1].data[5] = spd_int >> 4;
    TxMessage->motor[passage - 1].data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    TxMessage->motor[passage - 1].data[7] = tor_int & 0xff;

    // only control motor1
    for (auto &io : rx_)
    {
        int64_t value = 0;
        float fvalue = 0.0;
        if (passage == 1)
        {
            switch (io->io_subIdx)
            {
            case 1:
                break;
            case 3:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 4:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 5:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 6:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 7:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 8:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 9:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 16:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 17:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 18:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 19:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 3 && io->io_subIdx <= 19) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 2)
        {
            switch (io->io_subIdx)
            {
            case 20:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 21:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 22:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 23:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 24:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 25:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 32:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 33:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 34:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 35:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 36:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 20 && io->io_subIdx <= 36) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 3)
        {
            switch (io->io_subIdx)
            {
            case 37:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 38:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 39:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 40:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 41:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 48:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 49:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 50:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 51:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 52:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 53:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 37 && io->io_subIdx <= 53) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 4)
        {
            switch (io->io_subIdx)
            {
            case 54:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 55:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 56:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 57:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 64:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 65:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 66:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 67:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 68:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 69:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 70:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 54 && io->io_subIdx <= 70) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 5)
        {
            switch (io->io_subIdx)
            {
            case 71:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 72:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 73:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 80:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 81:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 82:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 83:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 84:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 85:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 86:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 87:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 71 && io->io_subIdx <= 87) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 6)
        {
            switch (io->io_subIdx)
            {
            case 88:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 89:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 96:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 97:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 98:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;                                         // only control motor1
            case 99:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 100:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 101:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 102:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 103:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 104:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 88 && io->io_subIdx <= 104) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }
    }
}

union RV_TypeConvert
{
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert;

void IoController::set_motor_position(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 8;

    if (ack_status > 3)
        return;

    rv_type_convert.to_float = pos;
    TxMessage->motor[passage - 1].data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    TxMessage->motor[passage - 1].data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    TxMessage->motor[passage - 1].data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    TxMessage->motor[passage - 1].data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    TxMessage->motor[passage - 1].data[4] = (rv_type_convert.buf[0] << 5) | (spd >> 10);
    TxMessage->motor[passage - 1].data[5] = (spd & 0x3FC) >> 2;
    TxMessage->motor[passage - 1].data[6] = (spd & 0x03) << 6 | (cur >> 6);
    TxMessage->motor[passage - 1].data[7] = (cur & 0x3F) << 2 | ack_status;

    for (auto &io : rx_)
    {
        int64_t value = 0;
        float fvalue = 0.0;
        if (passage == 1)
        {
            switch (io->io_subIdx)
            {
            case 1:
                break;
            case 3:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 4:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 5:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 6:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 7:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 8:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 9:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 16:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 17:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 18:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 19:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 3 && io->io_subIdx <= 19) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 2)
        {
            switch (io->io_subIdx)
            {
            case 20:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 21:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 22:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 23:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 24:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 25:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 32:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 33:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 34:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 35:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 36:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 20 && io->io_subIdx <= 36) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 3)
        {
            switch (io->io_subIdx)
            {
            case 37:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 38:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 39:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 40:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 41:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 48:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 49:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 50:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 51:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 52:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 53:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 37 && io->io_subIdx <= 53) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 4)
        {
            switch (io->io_subIdx)
            {
            case 54:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 55:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 56:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 57:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 64:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 65:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 66:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 67:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 68:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 69:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 70:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 54 && io->io_subIdx <= 70) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 5)
        {
            switch (io->io_subIdx)
            {
            case 71:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 72:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 73:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 80:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 81:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 82:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 83:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 84:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 85:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 86:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 87:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 71 && io->io_subIdx <= 87) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 6)
        {
            switch (io->io_subIdx)
            {
            case 88:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 89:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 96:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 97:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 98:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 99:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 100:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 101:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 102:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 103:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 104:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 88 && io->io_subIdx <= 104) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }
    }
}

void IoController::set_motor_cur_tor(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, int16_t cur_tor,
                                     uint8_t ctrl_status, uint8_t ack_status)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 3;

    if (ack_status > 3)
        return;
    if (ctrl_status > 7)
        return;
    if (ctrl_status) // enter torque control mode or brake mode
    {
        if (cur_tor > 3000)
            cur_tor = 3000;
        else if (cur_tor < -3000)
            cur_tor = -3000;
    }
    else
    {
        if (cur_tor > 2000)
            cur_tor = 2000;
        else if (cur_tor < -2000)
            cur_tor = -2000;
    }

    TxMessage->motor[passage - 1].data[0] = 0x60 | ctrl_status << 2 | ack_status;
    TxMessage->motor[passage - 1].data[1] = cur_tor >> 8;
    TxMessage->motor[passage - 1].data[2] = cur_tor & 0xff;

    for (auto &io : rx_)
    {
        int64_t value = 0;
        float fvalue = 0.0;
        if (passage == 1)
        {
            switch (io->io_subIdx)
            {
            case 1:
                break;
            case 3:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 4:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 5:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 6:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 7:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 8:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 9:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 16:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 17:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 18:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 19:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 3 && io->io_subIdx <= 19) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 2)
        {
            switch (io->io_subIdx)
            {
            case 20:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 21:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 22:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 23:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 24:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 25:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 32:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 33:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 34:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 35:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 36:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 20 && io->io_subIdx <= 36) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 3)
        {
            switch (io->io_subIdx)
            {
            case 37:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 38:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 39:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 40:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 41:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 48:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 49:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 50:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 51:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 52:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 53:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 37 && io->io_subIdx <= 53) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 4)
        {
            switch (io->io_subIdx)
            {
            case 54:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 55:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 56:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 57:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 64:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 65:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 66:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 67:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 68:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 69:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 70:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 54 && io->io_subIdx <= 70) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 5)
        {
            switch (io->io_subIdx)
            {
            case 71:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 72:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 73:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 80:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 81:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 82:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 83:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 84:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 85:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 86:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 87:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 71 && io->io_subIdx <= 87) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 6)
        {
            switch (io->io_subIdx)
            {
            case 88:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 89:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 96:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 97:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 98:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 99:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 100:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 101:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 102:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 103:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 104:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 88 && io->io_subIdx <= 104) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }
    }
}

void IoController::on_cycle(bool enable)
{
    if (!enable)
        return;
    controlIO();
}

io_data_type IoController::tans(const std::string &d)
{
    if (d == "BOOL" || d == "BIT")
        return io_data_type::BOOL;
    else if (d == "SINT")
        return io_data_type::SINT;
    else if (d == "USINT")
        return io_data_type::USINT;
    else if (d == "INT")
        return io_data_type::INT;
    else if (d == "UINT")
        return io_data_type::UINT;
    else if (d == "DINT")
        return io_data_type::DINT;
    else if (d == "UDINT")
        return io_data_type::UDINT;
    else if (d == "REAL")
        return io_data_type::REAL;
    else
        return io_data_type::NONE;
}

/*************************************encons_get_angle************************************************ */

void IoController::encons_get_angle_slave0(io_data *io)
{
#define encos_num 12
    static uint16_t pos_encos[encos_num] = {0};
    if (io->io_subIdx == 7)
    {
        pos_encos[1] = *(uint8_t *)io->io_address;
        pos_encos[1] = pos_encos[1] << 8;
    }

    if (io->io_subIdx == 8)
    {
        pos_encos[1] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[1] = (double)(4*(pos_encos[1]-32767.5)*360)/65535; // jiao su du
        encos_axis.encos_angle_actual_value[0] = pos_encos[1]; // mai chong shu
    }

    if (io->io_subIdx == 24)
    {
        pos_encos[2] = *(uint8_t *)io->io_address;
        pos_encos[2] = pos_encos[2] << 8;
    }

    if (io->io_subIdx == 25)
    {
        pos_encos[2] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[2] = (double)(4*(pos_encos[2]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[1] = pos_encos[2]; // mai chong shu
    }

    if (io->io_subIdx == 41)
    {
        pos_encos[3] = *(uint8_t *)io->io_address;
        pos_encos[3] = pos_encos[3] << 8;
    }

    if (io->io_subIdx == 48)
    {
        pos_encos[3] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[3] = (double)(4*(pos_encos[3]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[2] = pos_encos[3]; // mai chong shu
    }

    if (io->io_subIdx == 64)
    {
        pos_encos[4] = *(uint8_t *)io->io_address;
        pos_encos[4] = pos_encos[4] << 8;
    }

    if (io->io_subIdx == 65)
    {
        pos_encos[4] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[4] = (double)(4*(pos_encos[4]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[3] = pos_encos[4]; // mai chong shu
    }

    if (io->io_subIdx == 81)
    {
        pos_encos[5] = *(uint8_t *)io->io_address;
        pos_encos[5] = pos_encos[5] << 8;
    }

    if (io->io_subIdx == 82)
    {
        pos_encos[5] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[5] = (double)(4*(pos_encos[5]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[4] = pos_encos[5]; // mai chong shu
    }

    if (io->io_subIdx == 98)
    {
        pos_encos[6] = *(uint8_t *)io->io_address;
        pos_encos[6] = pos_encos[6] << 8;
    }

    if (io->io_subIdx == 99)
    {
        pos_encos[6] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[6] = (double)(4*(pos_encos[6]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[5] = pos_encos[6]; // mai chong shu
    }
}

void IoController::encons_get_angle_slave1(io_data *io)
{
#define encos_num 12
    static uint16_t pos_encos[encos_num] = {0};
    if (io->io_subIdx == 7)
    {
        pos_encos[1] = *(uint8_t *)io->io_address;
        pos_encos[1] = pos_encos[1] << 8;
    }

    if (io->io_subIdx == 8)
    {
        pos_encos[1] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[1] = (double)(4*(pos_encos[1]-32767.5)*360)/65535; // jiao su du
        encos_axis.encos_angle_actual_value[6] = pos_encos[1]; // mai chong shu
    }

    if (io->io_subIdx == 24)
    {
        pos_encos[2] = *(uint8_t *)io->io_address;
        pos_encos[2] = pos_encos[2] << 8;
    }

    if (io->io_subIdx == 25)
    {
        pos_encos[2] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[2] = (double)(4*(pos_encos[2]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[7] = pos_encos[2]; // mai chong shu
    }

    if (io->io_subIdx == 41)
    {
        pos_encos[3] = *(uint8_t *)io->io_address;
        pos_encos[3] = pos_encos[3] << 8;
    }

    if (io->io_subIdx == 48)
    {
        pos_encos[3] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[3] = (double)(4*(pos_encos[3]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[8] = pos_encos[3]; // mai chong shu
    }

    if (io->io_subIdx == 64)
    {
        pos_encos[4] = *(uint8_t *)io->io_address;
        pos_encos[4] = pos_encos[4] << 8;
    }

    if (io->io_subIdx == 65)
    {
        pos_encos[4] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[4] = (double)(4*(pos_encos[4]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[9] = pos_encos[4]; // mai chong shu
    }

    if (io->io_subIdx == 81)
    {
        pos_encos[5] = *(uint8_t *)io->io_address;
        pos_encos[5] = pos_encos[5] << 8;
    }

    if (io->io_subIdx == 82)
    {
        pos_encos[5] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[5] = (double)(4*(pos_encos[5]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[10] = pos_encos[5]; // mai chong shu
    }

    if (io->io_subIdx == 98)
    {
        pos_encos[6] = *(uint8_t *)io->io_address;
        pos_encos[6] = pos_encos[6] << 8;
    }

    if (io->io_subIdx == 99)
    {
        pos_encos[6] |= *(uint8_t *)io->io_address;
        // encos_axis.encos_angle_actual_value[6] = (double)(4*(pos_encos[6]-32767.5)*360)/65535;
        encos_axis.encos_angle_actual_value[11] = pos_encos[6]; // mai chong shu
    }
}
/*************************************encons_get_angle************************************************ */

/*************************************encons_get_angular_velocity************************************************ */
void IoController::encons_get_angular_velocity_slave0(io_data *io)
{
#define encos_num 12
    static uint16_t angular_velocity_encos[encos_num] = {0};
    // motor 1
    if (io->io_subIdx == 9)
    {
        angular_velocity_encos[1] = *(uint8_t *)io->io_address;
        angular_velocity_encos[1] = angular_velocity_encos[1] << 8;
    }
    if (io->io_subIdx == 16)
    {
        angular_velocity_encos[1] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[1] = angular_velocity_encos[1] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[1]=double(-18.0 + angular_velocity_encos[1]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[0] = angular_velocity_encos[1];
    }

    // motor 2
    if (io->io_subIdx == 32)
    {
        angular_velocity_encos[2] = *(uint8_t *)io->io_address;
        angular_velocity_encos[2] = angular_velocity_encos[2] << 8;
    }
    if (io->io_subIdx == 33)
    {
        angular_velocity_encos[2] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[2] = angular_velocity_encos[2] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[2]=double(-18.0 + angular_velocity_encos[2]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[1] = angular_velocity_encos[2];
    }

    // motor 3
    if (io->io_subIdx == 49)
    {
        angular_velocity_encos[3] = *(uint8_t *)io->io_address;
        angular_velocity_encos[3] = angular_velocity_encos[3] << 8;
    }
    if (io->io_subIdx == 50)
    {
        angular_velocity_encos[3] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[3] = angular_velocity_encos[3] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[3]=double(-18.0 + angular_velocity_encos[3]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[2] = angular_velocity_encos[3];
    }

    // motor 4
    if (io->io_subIdx == 66)
    {
        angular_velocity_encos[4] = *(uint8_t *)io->io_address;
        angular_velocity_encos[4] = angular_velocity_encos[4] << 8;
    }
    if (io->io_subIdx == 67)
    {
        angular_velocity_encos[4] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[4] = angular_velocity_encos[4] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[4]=double(-18.0 + angular_velocity_encos[4]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[3] = angular_velocity_encos[4];
    }

    // motor  5
    if (io->io_subIdx == 83)
    {
        angular_velocity_encos[5] = *(uint8_t *)io->io_address;
        angular_velocity_encos[5] = angular_velocity_encos[5] << 8;
    }
    if (io->io_subIdx == 84)
    {
        angular_velocity_encos[5] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[5] = angular_velocity_encos[5] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[5]=double(-18.0 + angular_velocity_encos[5]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[4] = angular_velocity_encos[5];
    }

    // motor 6
    if (io->io_subIdx == 100)
    {
        angular_velocity_encos[6] = *(uint8_t *)io->io_address;
        angular_velocity_encos[6] = angular_velocity_encos[6] << 8;
    }
    if (io->io_subIdx == 101)
    {
        angular_velocity_encos[6] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[6] = angular_velocity_encos[6] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[6]=double(-18.0 + angular_velocity_encos[6]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[5] = angular_velocity_encos[6];
    }
}

void IoController::encons_get_angular_velocity_slave1(io_data *io)
{
#define encos_num 12
    static uint16_t angular_velocity_encos[encos_num] = {0};
    // motor 1
    if (io->io_subIdx == 9)
    {
        angular_velocity_encos[1] = *(uint8_t *)io->io_address;
        angular_velocity_encos[1] = angular_velocity_encos[1] << 8;
    }
    if (io->io_subIdx == 16)
    {
        angular_velocity_encos[1] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[1] = angular_velocity_encos[1] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[1]=double(-18.0 + angular_velocity_encos[1]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[6] = angular_velocity_encos[1];
    }

    // motor 2
    if (io->io_subIdx == 32)
    {
        angular_velocity_encos[2] = *(uint8_t *)io->io_address;
        angular_velocity_encos[2] = angular_velocity_encos[2] << 8;
    }
    if (io->io_subIdx == 33)
    {
        angular_velocity_encos[2] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[2] = angular_velocity_encos[2] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[2]=double(-18.0 + angular_velocity_encos[2]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[7] = angular_velocity_encos[2];
    }

    // motor 3
    if (io->io_subIdx == 49)
    {
        angular_velocity_encos[3] = *(uint8_t *)io->io_address;
        angular_velocity_encos[3] = angular_velocity_encos[3] << 8;
    }
    if (io->io_subIdx == 50)
    {
        angular_velocity_encos[3] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[3] = angular_velocity_encos[3] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[3]=double(-18.0 + angular_velocity_encos[3]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[8] = angular_velocity_encos[3];
    }

    // motor 4
    if (io->io_subIdx == 66)
    {
        angular_velocity_encos[4] = *(uint8_t *)io->io_address;
        angular_velocity_encos[4] = angular_velocity_encos[4] << 8;
    }
    if (io->io_subIdx == 67)
    {
        angular_velocity_encos[4] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[4] = angular_velocity_encos[4] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[4]=double(-18.0 + angular_velocity_encos[4]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[9] = angular_velocity_encos[4];
    }

    // motor  5
    if (io->io_subIdx == 83)
    {
        angular_velocity_encos[5] = *(uint8_t *)io->io_address;
        angular_velocity_encos[5] = angular_velocity_encos[5] << 8;
    }
    if (io->io_subIdx == 84)
    {
        angular_velocity_encos[5] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[5] = angular_velocity_encos[5] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[5]=double(-18.0 + angular_velocity_encos[5]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[10] = angular_velocity_encos[5];
    }

    // motor 6
    if (io->io_subIdx == 100)
    {
        angular_velocity_encos[6] = *(uint8_t *)io->io_address;
        angular_velocity_encos[6] = angular_velocity_encos[6] << 8;
    }
    if (io->io_subIdx == 101)
    {
        angular_velocity_encos[6] |= *(uint8_t *)io->io_address;
        angular_velocity_encos[6] = angular_velocity_encos[6] >> 4;
        // encos_axis.encos_angular_velocity_actual_value[6]=double(-18.0 + angular_velocity_encos[6]*36.0 / 4095);
        encos_axis.encos_angular_velocity_actual_value[11] = angular_velocity_encos[6];
    }
}

/*************************************encons_get_angular_velocity************************************************ */

/*************************************encons_get_actual_current************************************************ */
void IoController::encons_get_actual_current_slave0(io_data *io)
{
#define encos_num 12
    // 实际电流：第5字节的低四位 第六字节的八位
    static uint16_t actual_current_encos[encos_num] = {0};
    // motor 1
    if (io->io_subIdx == 16) // DATA4
    {                        // 0000 0000 1111 1111
        actual_current_encos[1] = *(uint8_t *)io->io_address;
        actual_current_encos[1] = actual_current_encos[1] << 8;
        // PPPP XXXX 0000 0000
    }
    if (io->io_subIdx == 17) // DATA5
    {
        actual_current_encos[1] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[1] = actual_current_encos[1] & 0X0FFF;

        encos_axis.encos_current_actual_value[0] = actual_current_encos[1];
        //printf("actual_current_encos[1]:%d\n", actual_current_encos[1]);
    }

    // motor 2
    if (io->io_subIdx == 33)
    {
        actual_current_encos[2] = *(uint8_t *)io->io_address;
        actual_current_encos[2] = actual_current_encos[2] << 8;
    }
    if (io->io_subIdx == 34)
    {
        actual_current_encos[2] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[2] = actual_current_encos[2] & 0X0FFF;

        encos_axis.encos_current_actual_value[1] = actual_current_encos[2];
        //printf("actual_current_encos[2]:%d\n", actual_current_encos[2]);
    }

    // motor 3
    if (io->io_subIdx == 50)
    {
        actual_current_encos[3] = *(uint8_t *)io->io_address;
        actual_current_encos[3] = actual_current_encos[3] << 8;
    }
    if (io->io_subIdx == 51)
    {
        actual_current_encos[3] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[3] = actual_current_encos[3] & 0X0FFF;
        encos_axis.encos_current_actual_value[2] = actual_current_encos[3];
        //printf("actual_current_encos[3]:%d\n", actual_current_encos[3]);
    }

    // motor 4
    if (io->io_subIdx == 67)
    {
        actual_current_encos[4] = *(uint8_t *)io->io_address;
        actual_current_encos[4] = actual_current_encos[4] << 8;
    }
    if (io->io_subIdx == 68)
    {
        actual_current_encos[4] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[4] = actual_current_encos[4] & 0X0FFF;
        encos_axis.encos_current_actual_value[3] = actual_current_encos[4];
        //printf("actual_current_encos[4]:%d\n", actual_current_encos[4]);
    }

    // motor  5
    if (io->io_subIdx == 84)
    {
        actual_current_encos[5] = *(uint8_t *)io->io_address;
        actual_current_encos[5] = actual_current_encos[5] << 8;
    }
    if (io->io_subIdx == 85)
    {
        actual_current_encos[5] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[5] = actual_current_encos[5] & 0X0FFF;
        encos_axis.encos_current_actual_value[4] = actual_current_encos[5];
        //printf("actual_current_encos[5]:%d\n", actual_current_encos[5]);
    }

    // motor 6
    if (io->io_subIdx == 101)
    {
        actual_current_encos[6] = *(uint8_t *)io->io_address;
        actual_current_encos[6] = actual_current_encos[6] << 8;
    }
    if (io->io_subIdx == 102)
    {
        actual_current_encos[6] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[6] = actual_current_encos[6] & 0X0FFF;
        encos_axis.encos_current_actual_value[5] = actual_current_encos[6];
        //printf("actual_current_encos[6]:%d\n", actual_current_encos[6]);
    }
}
void IoController::encons_get_actual_current_slave1(io_data *io)
{
#define encos_num 12
    // 实际电流：第5字节的低四位 第六字节的八位
    static uint16_t actual_current_encos[encos_num] = {0};
    // motor 1
    if (io->io_subIdx == 16) // DATA4
    {                        // 0000 0000 1111 1111
        actual_current_encos[1] = *(uint8_t *)io->io_address;
        actual_current_encos[1] = actual_current_encos[1] << 8;
        // PPPP XXXX 0000 0000
    }
    if (io->io_subIdx == 17) // DATA5
    {
        actual_current_encos[1] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[1] = actual_current_encos[1] & 0X0FFF;
        encos_axis.encos_current_actual_value[6] = actual_current_encos[1];
        //printf("actual_current_encos[7]:%d\n", actual_current_encos[1]);
    }

    // motor 2
    if (io->io_subIdx == 33)
    {
        actual_current_encos[2] = *(uint8_t *)io->io_address;
        actual_current_encos[2] = actual_current_encos[2] << 8;
    }
    if (io->io_subIdx == 34)
    {
        actual_current_encos[2] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[2] = actual_current_encos[2] & 0X0FFF;
        encos_axis.encos_current_actual_value[7] = actual_current_encos[2];
        //printf("actual_current_encos[8]:%d\n", actual_current_encos[2]);
    }

    // motor 3
    if (io->io_subIdx == 50)
    {
        actual_current_encos[3] = *(uint8_t *)io->io_address;
        actual_current_encos[3] = actual_current_encos[3] << 8;
    }
    if (io->io_subIdx == 51)
    {
        actual_current_encos[3] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[3] = actual_current_encos[3] & 0X0FFF;
        encos_axis.encos_current_actual_value[8] = actual_current_encos[3];
        //printf("actual_current_encos[9]:%d\n", actual_current_encos[3]);
    }

    // motor 4
    if (io->io_subIdx == 67)
    {
        actual_current_encos[4] = *(uint8_t *)io->io_address;
        actual_current_encos[4] = actual_current_encos[4] << 8;
    }
    if (io->io_subIdx == 68)
    {
        actual_current_encos[4] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[4] = actual_current_encos[4] & 0X0FFF;
        encos_axis.encos_current_actual_value[9] = actual_current_encos[4];
        //printf("actual_current_encos[10]:%d\n", actual_current_encos[4]);
    }

    // motor  5
    if (io->io_subIdx == 84)
    {
        actual_current_encos[5] = *(uint8_t *)io->io_address;
        actual_current_encos[5] = actual_current_encos[5] << 8;
    }
    if (io->io_subIdx == 85)
    {
        actual_current_encos[5] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[5] = actual_current_encos[5] & 0X0FFF;
        encos_axis.encos_current_actual_value[10] = actual_current_encos[5];
        //printf("actual_current_encos[11]:%d\n", actual_current_encos[5]);
    }

    // motor 6
    if (io->io_subIdx == 101)
    {
        actual_current_encos[6] = *(uint8_t *)io->io_address;
        actual_current_encos[6] = actual_current_encos[6] << 8;
    }
    if (io->io_subIdx == 102)
    {
        actual_current_encos[6] |= *(uint8_t *)io->io_address;
        // PPPP XXXX XXXX XXXX
        actual_current_encos[6] = actual_current_encos[6] & 0X0FFF;
        encos_axis.encos_current_actual_value[11] = actual_current_encos[6];
        //printf("actual_current_encos[12]:%d\n", actual_current_encos[6]);
    }
}
/*************************************encons_get_actual_current************************************************ */

void IoController::MotorIDReading(EtherCAT_Msg *TxMessage, int passage)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = 0xFF;
    TxMessage->motor[0].data[1] = 0xFF;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x82;

    for (auto &io : rx_)
    {
        int64_t value = 0;
        float fvalue = 0.0;
        if (passage == 1)
        {
            switch (io->io_subIdx)
            {
            case 1:
                break;
            case 3:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 4:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 5:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 6:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 7:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 8:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 9:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 16:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 17:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 18:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 19:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 3 && io->io_subIdx <= 19) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 2)
        {
            switch (io->io_subIdx)
            {
            case 20:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 21:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 22:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 23:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 24:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 25:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 32:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 33:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 34:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 35:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 36:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 20 && io->io_subIdx <= 36) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 3)
        {
            switch (io->io_subIdx)
            {
            case 37:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 38:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 39:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 40:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 41:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 48:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 49:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 50:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 51:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 52:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 53:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 37 && io->io_subIdx <= 53) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }
    }
}

void IoController::MotorIDSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new, int passage)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x04;
    TxMessage->motor[0].data[4] = motor_id_new >> 8;
    TxMessage->motor[0].data[5] = motor_id_new & 0xff;
    for (auto &io : rx_)
    {
        int64_t value = 0;
        float fvalue = 0.0;
        if (passage == 1)
        {
            switch (io->io_subIdx)
            {
            case 1:
                break;
            case 3:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 4:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 5:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 6:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 7:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 8:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 9:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 16:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 17:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 18:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 19:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 3 && io->io_subIdx <= 19) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 2)
        {
            switch (io->io_subIdx)
            {
            case 20:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 21:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 22:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 23:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 24:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 25:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 32:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 33:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 34:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 35:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 36:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 20 && io->io_subIdx <= 36) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 3)
        {
            switch (io->io_subIdx)
            {
            case 37:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 38:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 39:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 40:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 41:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 48:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 49:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 50:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 51:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 52:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 53:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 37 && io->io_subIdx <= 53) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }
    }
}

// union RV_TypeConvert
// {
//     float to_float;
//     int to_int;
//     unsigned int to_uint;
//     uint8_t buf[4];
// } rv_type_convert;

void IoController::set_motor_speed(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 7;

    rv_type_convert.to_float = spd;
    TxMessage->motor[passage - 1].data[0] = 0x40 | ack_status;
    TxMessage->motor[passage - 1].data[1] = rv_type_convert.buf[3];
    TxMessage->motor[passage - 1].data[2] = rv_type_convert.buf[2];
    TxMessage->motor[passage - 1].data[3] = rv_type_convert.buf[1];
    TxMessage->motor[passage - 1].data[4] = rv_type_convert.buf[0];
    TxMessage->motor[passage - 1].data[5] = cur >> 8;
    TxMessage->motor[passage - 1].data[6] = cur & 0xff;

    // only control motor1
    for (auto &io : rx_)
    {
        int64_t value = 0;
        float fvalue = 0.0;
        if (passage == 1)
        {
            switch (io->io_subIdx)
            {
            case 1:
                break;
            case 3:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 4:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 5:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 6:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 7:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 8:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 9:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 16:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 17:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 18:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 19:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 3 && io->io_subIdx <= 19) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 2)
        {
            switch (io->io_subIdx)
            {
            case 20:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 21:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 22:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 23:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 24:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 25:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 32:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 33:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 34:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 35:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 36:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 20 && io->io_subIdx <= 36) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 3)
        {
            switch (io->io_subIdx)
            {
            case 37:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 38:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 39:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 40:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 41:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 48:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 49:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 50:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 51:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 52:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 53:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 37 && io->io_subIdx <= 53) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 4)
        {
            switch (io->io_subIdx)
            {
            case 54:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 55:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 56:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 57:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 64:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 65:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 66:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 67:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 68:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 69:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 70:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 54 && io->io_subIdx <= 70) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 5)
        {
            switch (io->io_subIdx)
            {
            case 71:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 72:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 73:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 80:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 81:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;
            case 82:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 83:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 84:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 85:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 86:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 87:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 71 && io->io_subIdx <= 87) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        if (passage == 6)
        {
            switch (io->io_subIdx)
            {
            case 88:
                value = TxMessage->motor[passage - 1].id; // 7010:3
                break;
            case 89:
                value = TxMessage->motor[passage - 1].rtr; // 7010:4
                break;
            case 96:
                value = TxMessage->motor[passage - 1].dlc; // 7010:5
                break;
            case 97:
                value = TxMessage->motor[passage - 1].data[0]; // 7010:6
                break;
            case 98:
                value = TxMessage->motor[passage - 1].data[1]; // 7010:7
                break;                                         // only control motor1
            case 99:
                value = TxMessage->motor[passage - 1].data[2]; // 7010:8
                break;
            case 100:
                value = TxMessage->motor[passage - 1].data[3]; // 7010:9
                break;
            case 101:
                value = TxMessage->motor[passage - 1].data[4]; // 7010:16
                break;
            case 102:
                value = TxMessage->motor[passage - 1].data[5]; // 7010:17
                break;
            case 103:
                value = TxMessage->motor[passage - 1].data[6]; // 7010:18
                break;
            case 104:
                value = TxMessage->motor[passage - 1].data[7]; // 7010:19
                break;
            default:
                break;
            }
            fvalue = value;
            if (io->io_subIdx >= 88 && io->io_subIdx <= 104) // only process motor1
            {
                setIO(io.get(), value, fvalue); // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }
    }
}

void IoController::encons_get_torque(io_data *io)
{
    static uint16_t current_12bit; // 明确命名为“12位电流数据”，避免歧义
    if (io->io_subIdx == 84)
    {
        current_12bit = *(uint8_t *)io->io_address;
        current_12bit &= 0x000F;            // 取84低4位（电流高位）
        current_12bit = current_12bit << 8; // 左移8位，为低位留空间
    }
    if (io->io_subIdx == 85)
    {
        current_12bit |= *(uint8_t *)io->io_address; // 组合85完整8位（电流低位）
        // 按文档换算实际电流（以-30A~30A为例，需与电机型号匹配）
        double actual_current = (current_12bit - 2047.5) * 60.0 / 4095.0;
        NECRO_RT(printf("12位电流数据 : %d\n", current_12bit));
        NECRO_RT(printf("实际电流 : %.2f A\n", actual_current)); // 结果符合文档规范
    }
}

void IoController::printIO(io_data *io)
{
    if (io->data_type == io_data_type::REAL)
    {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %f\n", io->slave_pos,
                        io->io_idx, io->io_subIdx, io->io_bit_pos, *(float *)io->io_address));
    }
    else if (io->data_type == io_data_type::DINT)
    {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %d\n", io->slave_pos,
                        io->io_idx, io->io_subIdx, io->io_bit_pos, *(int32_t *)io->io_address));
    }
    else if (io->data_type == io_data_type::INT)
    {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %d\n", io->slave_pos,
                        io->io_idx, io->io_subIdx, io->io_bit_pos, *(int16_t *)io->io_address));
    }
    else if (io->data_type == io_data_type::SINT)
    {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %d\n", io->slave_pos,
                        io->io_idx, io->io_subIdx, io->io_bit_pos, *(int8_t *)io->io_address));
    }
    else if (io->data_type == io_data_type::UDINT)
    {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) 0x%08x\n", io->slave_pos,
                        io->io_idx, io->io_subIdx, io->io_bit_pos, *(uint32_t *)io->io_address));
    }
    else if (io->data_type == io_data_type::UINT)
    {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) 0x%04x\n", io->slave_pos,
                        io->io_idx, io->io_subIdx, io->io_bit_pos, *(uint16_t *)io->io_address));
    }
    else if (io->data_type == io_data_type::USINT)
    {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) 0x%02x\n", io->slave_pos,
                        io->io_idx, io->io_subIdx, io->io_bit_pos, *(uint8_t *)io->io_address));
    }
    else if (io->data_type == io_data_type::BOOL)
    {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %d\n", io->slave_pos,
                        io->io_idx, io->io_subIdx, io->io_bit_pos, io_read_bit(io->io_address, io->io_bit_pos)));
    }
}

void IoController::setIO(io_data *io, int64_t value, float fvalue)
{
    if (io->data_type == io_data_type::REAL)
    {
        *(float *)io->io_address = fvalue;
    }
    else if (io->data_type == io_data_type::DINT)
    {
        *(int32_t *)io->io_address = (int32_t)value;
    }
    else if (io->data_type == io_data_type::INT)
    {
        *(int16_t *)io->io_address = (int16_t)value;
    }
    else if (io->data_type == io_data_type::SINT)
    {
        *(int8_t *)io->io_address = (int8_t)value;
    }
    else if (io->data_type == io_data_type::UDINT)
    {
        *(uint32_t *)io->io_address = (uint32_t)value;
    }
    else if (io->data_type == io_data_type::UINT)
    {
        *(uint16_t *)io->io_address = (uint16_t)value;
    }
    else if (io->data_type == io_data_type::USINT)
    {
        *(uint8_t *)io->io_address = (uint8_t)value;
    }
    else if (io->data_type == io_data_type::BOOL)
    {
        io_write_bit(io->io_address, io->io_bit_pos, (bool)value);
    }
}

void IO_example_ro::controlIO()
{
    static uint32_t count_ro = 0;
    count_ro++;
    if (!(count_ro % freq))
    {
        for (auto &io : tx_)
        {
            printIO(io.get());
        }
        NECRO_RT(printf("#################################### \n"));
    }
    count_ro %= 10000;
}

void IO_example_rw::controlIO()
{
    static uint32_t count_rw = 0;
    static int64_t value = 0;
    static float fvalue = 0.0;
    count_rw++;
    if (!(count_rw % freq))
    {
        for (auto &io : tx_)
        {
            printIO(io.get());
        }
        for (auto &io : rx_)
        {
            setIO(io.get(), value, fvalue);
        }
        value = ~value;
        NECRO_RT(printf("#################################### \n"));
    }
    count_rw %= 10000;
}
