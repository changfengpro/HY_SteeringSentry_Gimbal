#ifndef GIMBAL_H
#define GIMBAL_H


typedef struct 
{   
    float Vision_l_yaw_tar; //目标角度
    float Vision_r_yaw_tar;
    float Vision_l_pitch_tar;
    float Vision_r_pitch_tar;

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