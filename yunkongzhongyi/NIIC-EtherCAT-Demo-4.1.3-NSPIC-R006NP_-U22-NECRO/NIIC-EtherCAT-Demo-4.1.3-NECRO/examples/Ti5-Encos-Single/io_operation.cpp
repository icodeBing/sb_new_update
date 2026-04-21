#include "io_operation.hpp"


// encos
void IoController::send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage,uint8_t passage,uint16_t motor_id, float kp, float kd, float pos, float spd, float tor, int slave_pos)
{
    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;

    TxMessage->can_ide = 1;
    TxMessage->motor[passage-1].rtr = 0;
    TxMessage->motor[passage-1].id = motor_id;
    TxMessage->motor[passage-1].dlc = 8;

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

    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    tor_int = float_to_uint(tor, T_MIN, T_MAX, 12);

    TxMessage->motor[passage-1].data[0] = 0x00 | (kp_int >> 7);                             // kp5
    TxMessage->motor[passage-1].data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    TxMessage->motor[passage-1].data[2] = kd_int & 0xFF;
    TxMessage->motor[passage-1].data[3] = pos_int >> 8;
    TxMessage->motor[passage-1].data[4] = pos_int & 0xFF;
    TxMessage->motor[passage-1].data[5] = spd_int >> 4;
    TxMessage->motor[passage-1].data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    TxMessage->motor[passage-1].data[7] = tor_int & 0xff;


    for (auto& io : rx_) {
        int64_t value=0;
        float fvalue=0.0;
        if (passage == 1)
        {
            switch(io->io_subIdx)
            {
                case 1: 
                    break;
                case 3:
                    value=TxMessage->motor[passage-1].id;  // 7010:3
                    break;
                case 4:
                    value=TxMessage->motor[passage-1].rtr;  // 7010:4
                    break;
                case 5:
                    value=TxMessage->motor[passage-1].dlc;  // 7010:5
                    break;
                case 6:
                    value=TxMessage->motor[passage-1].data[0];  // 7010:6
                    break;
                case 7:
                    value=TxMessage->motor[passage-1].data[1];  // 7010:7
                    break;
                case 8:
                    value=TxMessage->motor[passage-1].data[2];  // 7010:8
                    break;
                case 9:
                    value=TxMessage->motor[passage-1].data[3];  // 7010:9
                    break;
                case 16:
                    value=TxMessage->motor[passage-1].data[4];  // 7010:16
                    break;
                case 17:
                    value=TxMessage->motor[passage-1].data[5];  // 7010:17
                    break;
                case 18:
                    value=TxMessage->motor[passage-1].data[6];  // 7010:18
                    break;
                case 19:
                    value=TxMessage->motor[passage-1].data[7];  // 7010:19
                    break;
                default:
                    break;

            }
            fvalue=value;
            if(io->io_subIdx>=3&&io->io_subIdx<=19) // only process motor1
            {
                setIO(io.get(), value, fvalue);  // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        else if (passage == 2)
        {
            switch(io->io_subIdx)
            {
                case 20:
                    value=TxMessage->motor[passage-1].id;  // 7010:20
                    break;
                case 21:
                    value=TxMessage->motor[passage-1].rtr;  // 7010:21
                    break;
                case 22:
                    value=TxMessage->motor[passage-1].dlc;  // 7010:22
                    break;
                case 23:
                    value=TxMessage->motor[passage-1].data[0];  // 7010:23
                    break;
                case 24:
                    value=TxMessage->motor[passage-1].data[1];  // 7010:24
                    break;
                case 25:
                    value=TxMessage->motor[passage-1].data[2];  // 7010:25
                    break;
                case 32:
                    value=TxMessage->motor[passage-1].data[3];  // 7010:32
                    break;
                case 33:
                    value=TxMessage->motor[passage-1].data[4];  // 7010:33
                    break;
                case 34:
                    value=TxMessage->motor[passage-1].data[5];  // 7010:34
                    break;
                case 35:
                    value=TxMessage->motor[passage-1].data[6];  // 7010:35
                    break;
                case 36:
                    value=TxMessage->motor[passage-1].data[7];  // 7010:36
                    break;
                default:
                    break;

            }
            fvalue=value;
            if(io->io_subIdx>=20&&io->io_subIdx<=36) // only process motor2
            {
                setIO(io.get(), value, fvalue);  // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        else if (passage == 3)
        {
            switch(io->io_subIdx)
            {
                case 37:
                    value=TxMessage->motor[passage-1].id;  // 7010:37
                    break;
                case 38:
                    value=TxMessage->motor[passage-1].rtr;  // 7010:38
                    break;
                case 39:
                    value=TxMessage->motor[passage-1].dlc;  // 7010:39
                    break;
                case 40:
                    value=TxMessage->motor[passage-1].data[0];  // 7010:40
                    break;
                case 41:
                    value=TxMessage->motor[passage-1].data[1];  // 7010:41
                    break;
                case 48:
                    value=TxMessage->motor[passage-1].data[2];  // 7010:48
                    break;
                case 49:
                    value=TxMessage->motor[passage-1].data[3];  // 7010:49
                    break;
                case 50:
                    value=TxMessage->motor[passage-1].data[4];  // 7010:50
                    break;
                case 51:
                    value=TxMessage->motor[passage-1].data[5];  // 7010:51
                    break;
                case 52:
                    value=TxMessage->motor[passage-1].data[6];  // 7010:52
                    break;
                case 53:
                    value=TxMessage->motor[passage-1].data[7];  // 7010:53
                    break;
                default:
                    break;
            }
            fvalue=value;
            if(io->io_subIdx>=37&&io->io_subIdx<=53) // only process motor3
            {
                setIO(io.get(), value, fvalue);  // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        else if (passage == 4)
        {
            switch(io->io_subIdx)
            {
                case 54:
                    value=TxMessage->motor[passage-1].id;  // 7010:54
                    break;
                case 55:
                    value=TxMessage->motor[passage-1].rtr;  // 7010:55
                    break;
                case 56:
                    value=TxMessage->motor[passage-1].dlc;  // 7010:56
                    break;
                case 57:
                    value=TxMessage->motor[passage-1].data[0];  // 7010:57
                    break;
                case 64:
                    value=TxMessage->motor[passage-1].data[1];  // 7010:64
                    break;
                case 65:
                    value=TxMessage->motor[passage-1].data[2];  // 7010:65
                    break;
                case 66:
                    value=TxMessage->motor[passage-1].data[3];  // 7010:66
                    break;
                case 67:
                    value=TxMessage->motor[passage-1].data[4];  // 7010:67
                    break;
                case 68:
                    value=TxMessage->motor[passage-1].data[5];  // 7010:68
                    break;
                case 69:
                    value=TxMessage->motor[passage-1].data[6];  // 7010:69
                    break;
                case 70:
                    value=TxMessage->motor[passage-1].data[7];  // 7010:70
                    break;
                default:
                    break;
            }
            fvalue=value;
            if(io->io_subIdx>=54&&io->io_subIdx<=70) // only process motor4
            {
                setIO(io.get(), value, fvalue);  // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        else if (passage == 5)
        {
            switch(io->io_subIdx)
            {
                case 71:
                    value=TxMessage->motor[passage-1].id;  // 7010:71
                    break;
                case 72:
                    value=TxMessage->motor[passage-1].rtr;  // 7010:72
                    break;
                case 73:
                    value=TxMessage->motor[passage-1].dlc;  // 7010:73
                    break;
                case 80:
                    value=TxMessage->motor[passage-1].data[0];  // 7010:80
                    break;
                case 81:
                    value=TxMessage->motor[passage-1].data[1];  // 7010:81
                    break;
                case 82:
                    value=TxMessage->motor[passage-1].data[2];  // 7010:82
                    break;
                case 83:
                    value=TxMessage->motor[passage-1].data[3];  // 7010:83
                    break;
                case 84:
                    value=TxMessage->motor[passage-1].data[4];  // 7010:84
                    break;
                case 85:
                    value=TxMessage->motor[passage-1].data[5];  // 7010:85
                    break;
                case 86:
                    value=TxMessage->motor[passage-1].data[6];  // 7010:86
                    break;
                case 87:
                    value=TxMessage->motor[passage-1].data[7];  // 7010:87
                    break;
                default:
                    break;
            }
            fvalue=value;
            if(io->io_subIdx>=71&&io->io_subIdx<=87) // only process motor5
            {
                setIO(io.get(), value, fvalue);  // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }

        else if (passage == 6)
        {
            switch(io->io_subIdx)
            {
                case 88:
                    value=TxMessage->motor[passage-1].id;  // 7010:88
                    break;
                case 89:
                    value=TxMessage->motor[passage-1].rtr;  // 7010:89
                    break;
                case 96:
                    value=TxMessage->motor[passage-1].dlc;  // 7010:96
                    break;
                case 97:
                    value=TxMessage->motor[passage-1].data[0];  // 7010:97
                    break;
                case 98:
                    value=TxMessage->motor[passage-1].data[1];  // 7010:98
                    break;
                case 99:
                    value=TxMessage->motor[passage-1].data[2];  // 7010:99
                    break;
                case 100:
                    value=TxMessage->motor[passage-1].data[3];  // 7010:100
                    break;
                case 101:
                    value=TxMessage->motor[passage-1].data[4];  // 7010:101
                    break;
                case 102:
                    value=TxMessage->motor[passage-1].data[5];  // 7010:102
                    break;
                case 103:
                    value=TxMessage->motor[passage-1].data[6];  // 7010:103
                    break;
                case 104:
                    value=TxMessage->motor[passage-1].data[7];  // 7010:104
                    break;
                default:
                    break;
            }
            fvalue=value;
            if(io->io_subIdx>=88&&io->io_subIdx<=104) // only process motor6
            {
                setIO(io.get(), value, fvalue);  // set value to pdo
                // __RT(printf("SetIO::(%04x:%d) value is %d\n", io->io_idx, io->io_subIdx, value));
            }
        }
    }
}    



void IoController::on_cycle_encos(int slave_pos) {
                // send_motor_ctrl_cmd(&Tx_Message, passage, motor_id, kp, kd, pos, spd, tor, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 1,1, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 2,2, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 3,3, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 4,4, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 5,5, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 6,6, 0, 0, 0, 0, 5, slave_pos);



    
    // send_motor_ctrl_cmd(&Tx_Message, 1,1, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 2,2, 0, 0, 0, 0, 5, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 3,3, 0, 0, 0, 0, 5, slave_pos);

    send_motor_ctrl_cmd(&Tx_Message, 4,4, 0, 0, 0, 0, 5, slave_pos);
    send_motor_ctrl_cmd(&Tx_Message, 5,5, 0, 0, 10, 0, 15, slave_pos);
    // send_motor_ctrl_cmd(&Tx_Message, 6,6, 0, 0, 0, 0, 5, slave_pos);





    
    // count_flag++;
    if(count_flag%1000==0){
        for (auto& io : tx_) {
            printIO(io.get());
        }
        NECRO_RT(printf("#################################### \n"));
    }
}

// encos







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
}


void IoController::setIO(io_data* io, int64_t value, float fvalue)
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
            setIO(io.get(), value, fvalue);
        }
        value = ~value;
        NECRO_RT(printf("#################################### \n"));
    }
    count_rw %= 10000;
}
