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


class IoController
{
public:
    bool control_enable = false;
    uint16_t slave_pos;
    uint32_t freq = 0;
    std::vector<std::unique_ptr<io_data>> rx_;
    std::vector<std::unique_ptr<io_data>> tx_;
    uint16_t joint_id=0;
public:
    IoController() {}
    ~IoController() {}

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
