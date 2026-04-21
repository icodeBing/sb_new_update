#pragma once
#include <iostream>

struct io_data
{
    std::uint16_t io_idx;
    std::uint8_t io_subIdx;
    std::uint8_t io_bitlen;
    std::uint16_t slave_pos;
    char data_type;

    // process data address
    volatile void* io_address;
    std::uint8_t io_bit_pos;
};

template <typename T>
void io_write_bit(T* address, std::uint8_t pos, bool val)
{
    if (val)
        *(std::uint8_t*)address |= (1 << pos);
    else
        *(std::uint8_t*)address &= ~(1 << pos);
}

template <typename T>
bool io_read_bit(T* address, std::uint8_t pos)
{
    return (*(uint8_t*)address >> pos) & 0x01;
}
