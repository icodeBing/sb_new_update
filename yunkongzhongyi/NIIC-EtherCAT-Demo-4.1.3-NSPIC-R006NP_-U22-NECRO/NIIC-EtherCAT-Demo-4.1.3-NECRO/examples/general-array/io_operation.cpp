#include "io_operation.hpp"

void IoController::on_cycle(bool enable)
{
    if (!enable)
        return;

    controlIO();
}

io_data_type IoController::tans(const std::string& d)
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
    else if (d == "ARRAY [0..1] OF BYTE")
        return io_data_type::ARRAY_OF_BYTE;
    else
        return io_data_type::NONE;
}

void IoController::printIO(io_data* io)
{
    if (io->data_type == io_data_type::REAL) {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %f\n", io->slave_pos,
            io->io_idx, io->io_subIdx, io->io_bit_pos, *(float*)io->io_address));
    }
    else if (io->data_type == io_data_type::DINT) {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %d\n", io->slave_pos,
            io->io_idx, io->io_subIdx, io->io_bit_pos, *(int32_t*)io->io_address));
    }
    else if (io->data_type == io_data_type::INT) {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %d\n", io->slave_pos,
            io->io_idx, io->io_subIdx, io->io_bit_pos, *(int16_t*)io->io_address));
    }
    else if (io->data_type == io_data_type::SINT) {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %d\n", io->slave_pos,
            io->io_idx, io->io_subIdx, io->io_bit_pos, *(int8_t*)io->io_address));
    }
    else if (io->data_type == io_data_type::UDINT) {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) 0x%08x\n", io->slave_pos,
            io->io_idx, io->io_subIdx, io->io_bit_pos, *(uint32_t*)io->io_address));
    }
    else if (io->data_type == io_data_type::UINT) {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) 0x%04x\n", io->slave_pos,
            io->io_idx, io->io_subIdx, io->io_bit_pos, *(uint16_t*)io->io_address));
    }
    else if (io->data_type == io_data_type::USINT) {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) 0x%02x\n", io->slave_pos,
            io->io_idx, io->io_subIdx, io->io_bit_pos, *(uint8_t*)io->io_address));
    }
    else if (io->data_type == io_data_type::BOOL) {
        NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %d\n", io->slave_pos,
            io->io_idx, io->io_subIdx, io->io_bit_pos, io_read_bit(io->io_address, io->io_bit_pos)));
    }
    else if (io->data_type == io_data_type::ARRAY_OF_BYTE) {
        for(size_t pos = 0; pos < io->io_bitlen/8; pos++) {
            volatile uint8_t* byte_address = static_cast<volatile uint8_t*>(io->io_address);
            uint8_t value = byte_address[pos];
            NECRO_RT(printf("[IO] %d (0x%04x:%02d|%02d) %d\n", io->slave_pos,
            io->io_idx, io->io_subIdx, pos, static_cast<int>(value)));
        }
    }
}


void IoController::setIO(io_data* io, int64_t value, float fvalue, int pos)
{
    if (io->data_type == io_data_type::REAL) {
        *(float*)io->io_address = fvalue;
    }
    else if (io->data_type == io_data_type::DINT) {
        *(int32_t*)io->io_address = (int32_t)value;
    }
    else if (io->data_type == io_data_type::INT) {
        *(int16_t*)io->io_address = (int16_t)value;
    }
    else if (io->data_type == io_data_type::SINT) {
        *(int8_t*)io->io_address = (int8_t)value;
    }
    else if (io->data_type == io_data_type::UDINT) {
        *(uint32_t*)io->io_address = (uint32_t)value;
    }
    else if (io->data_type == io_data_type::UINT) {
        *(uint16_t*)io->io_address = (uint16_t)value;
    }
    else if (io->data_type == io_data_type::USINT) {
        *(uint8_t*)io->io_address = (uint8_t)value;
    }
    else if (io->data_type == io_data_type::BOOL) {
        io_write_bit(io->io_address, io->io_bit_pos, (bool)value);
    }
    else if (io->data_type == io_data_type::ARRAY_OF_BYTE) {
        volatile uint8_t* byte_address = static_cast<volatile uint8_t*>(io->io_address);
        byte_address[pos] = (uint8_t)value;
    }
}


void IO_example_ro::controlIO()
{
    static uint32_t count_ro = 0;
    count_ro++;
    if (!(count_ro % freq)) {
        for (auto& io : tx_) {
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
    if (!(count_rw % freq)) {
        for (auto& io : tx_) {
            printIO(io.get());
        }
        for (auto& io : rx_) {
            printIO(io.get());
            if (io->io_idx==0x7042)  // GL20
            {
                setIO(io.get(), value, fvalue, 0);
                setIO(io.get(), value, fvalue, 1);
                if (value==0) value=1;
                else if (value>0&&value<=128) value=value*2;
                else value=0;
            }
        }
        // value = ~value;
        NECRO_RT(printf("#################################### \n"));
    }
    count_rw %= 10000;
}

