#ifndef GIMBAL_H
#define GIMBAL_H

// 子云台相对于母云台的角度
typedef struct
{
    float l_yaw;
    float l_single_yaw;
    float r_yaw;
    float r_single_yaw;
} Rela_To_Base_Yaw;

typedef struct 
{   
    Rela_To_Base_Yaw gimbal_rela;              // 自瞄计算时使用

    float Vision_l_yaw; //自瞄模式计算时的值
    float Vision_r_yaw;
    float Vision_l_pitch;
    float Vision_r_pitch;
    
    // float Vision_l_yaw_offset; //与目标的差
    // float Vision_r_yaw_offset;
    // float Vision_l_pitch_offset;
    // float Vision_r_pitch_offset;

    float Vision_set_l_yaw; // 自瞄模式下给电机发送的值
    float Vision_set_r_yaw;
    float Vision_set_l_pitch;  
    float Vision_set_r_pitch;  

} Vision_Gimbal_Data_s;


/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H