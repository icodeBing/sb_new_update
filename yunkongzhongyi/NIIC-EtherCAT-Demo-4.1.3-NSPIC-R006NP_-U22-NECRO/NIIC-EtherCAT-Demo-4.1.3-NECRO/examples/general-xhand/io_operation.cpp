#include "io_operation.hpp"
#include <ctime>
#include <cmath>

#define EV_HAND_JOINT_NUM 12
//#define ANGLE_ACTION_NUM 10
#define time_c 10

typedef struct {
	float poses[EV_HAND_JOINT_NUM];
	uint32_t msec;
} hand_angle_pose_t;


hand_angle_pose_t angle_hand_actions[] = {	// Aging Test
		// {.poses = { 90,  80,   70, 10,   0,   0,   0,   0,   0,   0,   0,   0}, .msec = time_c},
        // {.poses = { 60,  20,   20,  6,  30,  30,  30,  30,  30,  30,  30,  30}, .msec = time_c},
		// {.poses = { 20,   5,  -20,  2,  90,  90,  90,  90,  90,  90,  90,  90}, .msec = time_c},
        // {.poses = {  0,   0,  -40,  0, 110, 110, 110, 110, 110, 110, 110, 110}, .msec = time_c},
        // {.poses = { 40,  10,    0,  4,  60,  60,  60,  60,  60,  60,  60,  60}, .msec = time_c},
        // {.poses = { 80,  40,   40,  8,  10,  10,  10,  10,  10,  10,  10,  10}, .msec = time_c},
		// {.poses = { 90,  80,   70, 10,   0,   0,   0,   0,   0,   0,   0,   0}, .msec = time_c},
        // {.poses = { 90,  80,   70, 10,   0,   0,   0,   0,   0,   0,   0,   0}, .msec = time_c},
        // {.poses = { 0,   60,   60,  0,   110,  110, 110,110,110,110,110,110}, .msec = time_c},//9 握拳
        // {.poses = {80, 20, 40,  0, 80, 50, 30, 30, 20, 20, 10, 10}, .msec = time_c},//10 孔雀1
        // {.poses = { 0, 20, 20, 10,  0,  0,110,110,110,110,  0,  0}, .msec = time_c},//12 爱你1

        {.poses = { 0,  90,  0,  0,  0,  0,  0,  0, 0,   0,  0,  0}, .msec = time_c},// 握拳
		{.poses = { 25, 45,  20,  0,  30,  30,  30,  30, 30, 30, 30, 30}, .msec = time_c},// 握拳
        {.poses = { 50,  0,  40,  0,  60,  60,  60,  60, 60, 60, 60, 60}, .msec = time_c},// 握拳
        {.poses = { 70,  20,  50,  0,  90,  90,  90,  90, 90, 90, 90, 90}, .msec = time_c},// 握拳
        {.poses = { 90,  40,  70,  0,  110,  110,  110,  110, 110, 110, 110, 110}, .msec = time_c},// 握拳
        {.poses = { 90,  40,  70,  0,  110,  110,  110,  110, 110, 110, 110, 110}, .msec = time_c*10},// 握拳
        {.poses = { 70,  20,  50,  0,  110,  110,  110,  110, 110, 110, 110, 110}, .msec = time_c},// 握拳
        {.poses = { 50,  0,  40,  0,  60,  60,  60,  60, 60, 60, 60, 60}, .msec = time_c},// 握拳
        {.poses = { 25, 45,  20,  0,  30,  30,  30,  30, 30, 30, 30, 30}, .msec = time_c},// 握拳
        {.poses = { 0,  90,  0,  0,  0,  0,  0,  0, 0,   0,  0,  0}, .msec = time_c},// 握拳
        {.poses = { 0,  90,  0,  0,  0,  0,  0,  0, 0,   0,  0,  0}, .msec = time_c*10},// 
        {.poses = { 25, 45,  20,  3,  0,  0,  0,  0, 30, 30, 30, 30}, .msec = time_c},// V
        {.poses = { 50, 0,  40,  6,  0,  0,  0,  0, 60, 60, 60, 60}, .msec = time_c},// V
        {.poses = { 80, 20,  50,  8,  0,  0,  0,  0, 90, 90, 90, 90}, .msec = time_c},// V
        {.poses = { 105, 40,  50,  10,  0,  0,  0,  0, 110, 110, 110, 110}, .msec = time_c},// V
        {.poses = { 105, 40,  50,  10,  0,  0,  0,  0, 110, 110, 110, 110}, .msec = time_c*10},// V
        {.poses = { 80, 20,  50,  8,  0,  0,  0,  0, 90, 90, 90, 90}, .msec = time_c},// V
        {.poses = { 50, 0,  40,  6,  0,  0,  0,  0, 60, 60, 60, 60}, .msec = time_c},// V
        {.poses = { 25, 45,  20,  3,  0,  0,  0,  0, 30, 30, 30, 30}, .msec = time_c},// V
        {.poses = { 0,  90,  0,  0,  0,  0,  0,  0, 0,   0,  0,  0}, .msec = time_c},// v
        {.poses = { 0,  90,  0,  0,  0,  0,  0,  0, 0,   0,  0,  0}, .msec = time_c*10},//
        {.poses = {20, 10, 20,  0, 30, 20, 0, 0, 0, 0, 0, 0}, .msec = time_c},//对指
        {.poses = {40, 20, 40,  0, 60, 40, 0, 0, 0, 0, 0, 0}, .msec = time_c},//对指
        {.poses = {70, 30, 60,  0, 80, 70, 0, 0, 0, 0, 0, 0}, .msec = time_c},//对指
        {.poses = {70, 30, 60,  0, 80, 70, 0, 0, 0, 0, 0, 0}, .msec = time_c*10},//对指
        {.poses = {40, 20, 40,  0, 60, 40, 0, 0, 0, 0, 0, 0}, .msec = time_c},//对指
        {.poses = {20, 10, 20,  0, 30, 20, 0, 0, 0, 0, 0, 0}, .msec = time_c},//对指
        {.poses = { 0,  90,  0,  0,  0,  0,  0,  0, 0,   0,  0,  0}, .msec = time_c},// v
        {.poses = { 0,  90,  0,  0,  0,  0,  0,  0, 0,   0,  0,  0}, .msec = time_c*10},//

};

float joint_des_position[EV_HAND_JOINT_NUM] = {0,1.57,0,0,0,0,0,0,0,0,0,0};


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
    if(io->io_subIdx == 1)   //jiont id
    {
        *(uint16_t*)io->io_address = value;
    }
    if(io->io_subIdx == 2)  //kp
    {
        *(uint16_t*)io->io_address = 100;
    }
    if(io->io_subIdx == 5)  //position
    {
        *(float*)io->io_address = joint_des_position[value];
    }
    if(io->io_subIdx == 6)
    {
        *(uint16_t*)io->io_address = 300;
    }
    if(io->io_subIdx == 7)
    {
        *(uint16_t*)io->io_address = 3;
    }

    // if (io->data_type == io_data_type::REAL) {
    //     *(float*)io->io_address = fvalue;
    // }
    // else if (io->data_type == io_data_type::DINT) {
    //     *(int32_t*)io->io_address = (int32_t)value;
    // }
    // else if (io->data_type == io_data_type::INT) {
    //     *(int16_t*)io->io_address = (int16_t)value;
    // }
    // else if (io->data_type == io_data_type::SINT) {
    //     *(int8_t*)io->io_address = (int8_t)value;
    // }
    // else if (io->data_type == io_data_type::UDINT) {
    //     *(uint32_t*)io->io_address = (uint32_t)value;
    // }
    // else if (io->data_type == io_data_type::UINT) {
    //     *(uint16_t*)io->io_address = (uint16_t)value;
    // }
    // else if (io->data_type == io_data_type::USINT) {
    //     *(uint8_t*)io->io_address = (uint8_t)value;
    // }
    // else if (io->data_type == io_data_type::BOOL) {
    //     io_write_bit(io->io_address, io->io_bit_pos, (bool)value);
    // }
}


void IO_example_ro::controlIO()
{
    static uint32_t count_ro = 0;
    static int64_t value = 0;
    static float fvalue = 0.0;
    static int joint_index = 0;
    count_ro++;
    //if (!(count_ro % freq)) 
    {
        for (auto& io : tx_) {
            printIO(io.get());
        }
        NECRO_RT(printf("#################################### \n"));
    }
    count_ro %= 100000;

    
    
    static int currentPoseIndex = 0;
    static uint32_t currentTick = 0;
    static uint32_t control_count = 0;
    uint32_t ANGLE_ACTION_NUM = sizeof(angle_hand_actions)/sizeof(angle_hand_actions[0]);
    control_count++;
    if(control_count>20)
    {
        control_count = 0;
        
        uint32_t TOTAL_STEPS = angle_hand_actions[currentPoseIndex].msec;
        if (currentTick >= TOTAL_STEPS) 
        {
            currentPoseIndex = (currentPoseIndex + 1) % ANGLE_ACTION_NUM;
            currentTick = 0;
        }
        for (int joint_index = 0; joint_index < EV_HAND_JOINT_NUM; ++joint_index) 
        {
            if (currentPoseIndex > 0) {
                int prevPoseIndex = (currentPoseIndex - 1 + ANGLE_ACTION_NUM)
                        % ANGLE_ACTION_NUM;
                uint16_t startPos =
                        angle_hand_actions[prevPoseIndex].poses[joint_index];
                uint16_t endPos =
                        angle_hand_actions[currentPoseIndex].poses[joint_index];
                float fraction = (float) currentTick / TOTAL_STEPS;
                joint_des_position[joint_index] = round(startPos + fraction * (endPos - startPos)) / 180.0f * 3.1415f;
            }
        }

        currentTick++;
    }
    value = joint_index;
    for (auto& io : rx_) {
            setIO(io.get(), value, fvalue);
        }
    joint_index++;
    if(joint_index >= EV_HAND_JOINT_NUM)
    {
        joint_index = 0;
    }    

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

