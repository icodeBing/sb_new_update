#include "global/golbalParams.h"



// 集中定义所有全局变量——整个工程仅此处定义，所有源文件通过头文件extern声明访问
std::array<std::string, motor_num> joint_names = 
    {"neck_ti5_joint1", "arm_L1_ti5_joint2", "arm_L2_ti5_joint3",  "arm_L3_ti5_joint4",  "arm_L4_ti5_joint5",
    "waist_ti5_1_joint6" , "waist_ti5_2_joint7","arm_R1_ti5_joint8", "arm_R2_ti5_joint9", "arm_R3_ti5_joint10", "arm_R4_ti5_joint11", 
    "leg_L1_enc_joint12","leg_L2_enc_joint13", "leg_L3_enc_joint14", "leg_L4_enc_joint15", "leg_L5_enc_joint16", "leg_L6_enc_joint17", 
    "leg_R1_enc_joint18","leg_R2_enc_joint19", "leg_R3_enc_joint20", "leg_R4_enc_joint21", "leg_R5_enc_joint22", "leg_R6_enc_joint23"};

std::int8_t Motor_Index_Switch[motor_num] = {
    //master0
    //leg_L
    0,
    1,
    2,
    3,
    4,
    5,
    //leg_H
    6,
    7,
    8,
    9,
    10,
    11,
    //master1  
    14,       // head_yaw
    16,       // left_shoulder_roll
    18,       // left_elbow
    17,       // left_shoulder_yaw
    15,       // left_shoulder_pitch
    //master2
    20,       // right_shoulder_roll
    22,       // right_elbow
    21,       // right_shoulder_yaw
    19,       // right_shoulder_pitch
    12,       // waist_yaw
    13        // waist_pitch
};

std::mutex joint_mutex;
std::shared_ptr<motor_interface::msg::JointMotor> global_Joint_data = std::make_shared<motor_interface::msg::JointMotor>();
std::shared_ptr<motor_interface::msg::JointMotor> temp_global_Joint_data = std::make_shared<motor_interface::msg::JointMotor>();

std::mutex joint_mutex2;
std::shared_ptr<motor_interface::msg::JointMotor> global_Joint_data_pub = std::make_shared<motor_interface::msg::JointMotor>();
std::shared_ptr<motor_interface::msg::JointMotor> temp_global_Joint_data_pub = std::make_shared<motor_interface::msg::JointMotor>();
