#pragma once
#include <vector>
#include <memory>
#include "io.hpp"
#include "ecat/detail/config.hpp"

enum io_data_type : char {
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

typedef struct {
    io_data* data_ = nullptr;

    template<typename T>
    T get() {
        if (data_->data_type == io_data_type::BOOL)
            return io_read_bit(data_->io_address, data_->io_bit_pos);
        else
            return *(T*)data_->io_address;
    }

    template<typename T>
    void set(T a) {
        if (data_->data_type == io_data_type::BOOL)
            io_write_bit(data_->io_address, data_->io_bit_pos, (bool)a);
        else
            *(T*)data_->io_address = a;
    }
} DataIO;

// encos
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -300.0f
#define T_MAX 300.0f
#define I_MIN -140.0f
#define I_MAX 140.0f


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



class IoController
{
public:
    bool control_enable = false;
    uint16_t slave_pos;
    uint32_t freq = 0;
    int64_t count_flag=0;
    std::vector<std::unique_ptr<io_data>> rx_;
    std::vector<std::unique_ptr<io_data>> tx_;
public:
    IoController() {}
    ~IoController() {}


    // encos
    EtherCAT_Msg Rx_Message;
    EtherCAT_Msg Tx_Message;
    
    int float_to_uint(float x, float x_min, float x_max, int bits) {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }
    void on_cycle_encos(int slave_pos);
    void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage,uint8_t passage,uint16_t motor_id, float kp, float kd, float pos, float spd, float tor, int slave_pos);
    // encos



    
    void on_cycle(bool enable);
    virtual void controlIO() = 0;
    virtual void bind_pdo() = 0;

    io_data_type tans(const std::string& d);
    void printIO(io_data* io);
    void setIO(io_data* io, int64_t value, float fvalue);
    
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