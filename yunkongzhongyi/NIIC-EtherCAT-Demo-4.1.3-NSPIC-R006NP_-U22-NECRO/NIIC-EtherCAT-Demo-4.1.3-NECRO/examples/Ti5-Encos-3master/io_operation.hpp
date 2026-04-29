#pragma once
#include <vector>
#include <memory>
#include "io.hpp"
#include "ecat/detail/config.hpp"

enum io_data_type : char
{
    NONE,
    BOOL,
    SINT,
    USINT,
    INT,
    UINT,
    DINT,
    UDINT,
    REAL,
};

typedef struct
{
    io_data *data_ = nullptr;

    template <typename T>
    T get()
    {
        if (data_->data_type == io_data_type::BOOL)
            return io_read_bit(data_->io_address, data_->io_bit_pos);
        else
            return *(T *)data_->io_address;
    }

    template <typename T>
    void set(T a)
    {
        if (data_->data_type == io_data_type::BOOL)
            io_write_bit(data_->io_address, data_->io_bit_pos, (bool)a);
        else
            *(T *)data_->io_address = a;
    }
} DataIO;

// encos
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define KD_MAX_10020 0.0f
#define KD_MIN_10020 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -140.0f
#define I_MAX 140.0f

#define TMAX_10020 150.0f
#define TMIN_10020 -150.0f
#define IMAX_8116 150.0f
#define IMIN_8116 -150.0f
#define TMAX_6408 60.0f
#define TMIN_6408 -60.0f
#define IMAX_4315 70.0f
#define IMIN_4315 -70.0f

#define param_get_pos 0x01
#define param_get_spd 0x02
#define param_get_cur 0x03
#define param_get_pwr 0x04
#define param_get_acc 0x05
#define param_get_lkgKP 0x06
#define param_get_spdKI 0x07
#define param_get_fdbKP 0x08
#define param_get_fdbKD 0x09

#define comm_ack 0x00
#define comm_auto 0x01

struct Motor_Msg
{
    uint32_t id;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
};

typedef struct
{
    uint8_t motor_num;
    uint8_t can_ide;
    Motor_Msg motor[6];

} EtherCAT_Msg;
// encos

typedef struct
{
    uint16_t angle_actual_int;
    uint16_t angle_desired_int;
    int16_t speed_actual_int;
    int16_t speed_desired_int;
    int16_t current_actual_int;
    int16_t current_desired_int;
    float speed_actual_rad;
    float speed_desired_rad;
    float angle_actual_rad;
    float angle_desired_rad;
    uint16_t motor_id;
    uint8_t temperature;
    uint8_t error;
    float angle_actual_float;
    float speed_actual_float;
    float current_actual_float;
    float angle_desired_float;
    float speed_desired_float;
    float current_desired_float;
    float power;
    uint16_t acceleration;
    uint16_t linkage_KP;
    uint16_t speed_KI;
    uint16_t feedback_KP;
    uint16_t feedback_KD;
} OD_Motor_Msg;

class IoController
{
public:
    bool control_enable = false;
    uint16_t slave_pos;
    uint32_t freq = 0;
    int64_t count_flag = 0;
    std::vector<std::unique_ptr<io_data>> rx_;
    std::vector<std::unique_ptr<io_data>> tx_;

public:
    IoController() {}
    ~IoController() {}

    // encos
    EtherCAT_Msg Rx_Message;
    EtherCAT_Msg Tx_Message;

    int float_to_uint(float x, float x_min, float x_max, int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }
    void on_cycle_encos(int slave_pos);
    void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor, int slave_pos);
    // encos
    void set_motor_position(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur, uint8_t ack_status);
    void set_motor_speed(EtherCAT_Msg *tx_msg, uint8_t passage, uint16_t motor_id, float spd, uint16_t cur, uint8_t ack_status);
    void set_motor_cur_tor(EtherCAT_Msg *tx_msg, uint8_t passage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status);
    // double encons_get_positon(io_data* io , int passage);

    void encons_get_torque(io_data *io);
    void encons_get_angle_slave0(io_data *io);
    void encons_get_angular_velocity_slave0(io_data *io);

    void encons_get_angle_slave1(io_data *io);
    void encons_get_angular_velocity_slave1(io_data *io);

    void encons_get_actual_current_slave0(io_data *io);
    void encons_get_actual_current_slave1(io_data *io);

    void MotorIDReading(EtherCAT_Msg *tx_msg, int passage);
    void MotorIDSetting(EtherCAT_Msg *tx_msg, uint16_t motor_id, uint16_t motor_id_new, int passage);
    void MotorZeroSetting(EtherCAT_Msg *tx_msg, uint16_t motor_id, int passage);

    void on_cycle(bool enable);
    virtual void controlIO() = 0;
    virtual void bind_pdo() = 0;

    io_data_type tans(const std::string &d);
    void printIO(io_data *io);
    void setIO(io_data *io, int64_t value, float fvalue);
};

class IO_example_ro : public IoController
{
public:
    void controlIO();
    void bind_pdo() {}
};

class IO_example_rw : public IoController
{
public:
    void controlIO();
    void bind_pdo() {}
};
