#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"


#define YAW_L_INIT_ANGLE -150.0f // 云台初始角度
#define PITCH_L_INIT_ANGLE -118.0f // 云台初始俯仰角度   -117.0f
#define YAW_R_INIT_ANGLE -30.0f // 云台初始角度
#define PITCH_R_INIT_ANGLE -120.5f // 云台初始俯仰角度   -118.0f
#define PITCH_R_MIN 28 // 右云台经IMU测出下限时的pitch角度 25.3
#define YAW_COEFF_REMOTE 0.036363636f //云台遥控系数
#define PITCH_COEFF_REMOTE 0.134848485f //云台俯仰遥控系数
#define YAW_VISION_OFFSET 12

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_l_motor, *yaw_r_motor, *pitch_l_motor, *pitch_r_motor; // 云台电机实例

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Vision_Gimbal_Data_s vision_gimbal_data; // 自瞄时云台数据(为方便计算，定义了相对角度)

// static BMI088Instance *bmi088; // 云台IMU


void GimbalInit()
{   
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 52, // Me:30
                .Ki = 30,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 100,
                .Output_LPF_RC=0.00005,
                .MaxOut = 2000,
            },
            .speed_PID = {
                .Kp = 40,  // 50
                .Ki = 50, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement |PID_OutputFilter,
                .Output_LPF_RC=0.00649999983,
                .IntegralLimit = 2000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50, //  Me:20
                .Ki = 15, // 3
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2000, // 30
                .MaxOut = 800,
            },
            .speed_PID = {
                .Kp = 40,  // 15    空载k = 10  Me: 46
                .Ki = 300, // 500
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .Output_LPF_RC=0.00649999983,
                .IntegralLimit = 8500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimbal_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };  

    /*
        抬头参数
    */
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_l_motor = DJIMotorInit(&yaw_config);
    yaw_config.can_init_config.can_handle = &hcan2;
    yaw_r_motor = DJIMotorInit(&yaw_config);

    pitch_l_motor = DJIMotorInit(&pitch_config);
    pitch_config.can_init_config.can_handle = &hcan2;
    pitch_r_motor = DJIMotorInit(&pitch_config);


    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}


/**
 * @brief 角度计算
 */
static void VisionAngleCalc()
{   
    vision_gimbal_data.Vision_r_yaw = gimbal_IMU_data->Yaw + yaw_r_motor->measure.total_angle - YAW_R_INIT_ANGLE;
    vision_gimbal_data.Vision_r_pitch = pitch_r_motor->measure.total_angle + PITCH_R_MIN - PITCH_R_INIT_ANGLE;

    vision_gimbal_data.Vision_set_r_yaw = vision_gimbal_data.Vision_r_yaw_tar + YAW_R_INIT_ANGLE - gimbal_IMU_data->Yaw;
    vision_gimbal_data.Vision_set_r_pitch = vision_gimbal_data.Vision_r_pitch_tar + PITCH_R_INIT_ANGLE - PITCH_R_MIN;

}


/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{   

    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    if(gimbal_cmd_recv.gimbal_mode == GIMBAL_VISION)
    {
        vision_gimbal_data.Vision_r_yaw_tar = gimbal_cmd_recv.yaw;
        vision_gimbal_data.Vision_r_pitch_tar = gimbal_cmd_recv.pitch;
    }

    VisionAngleCalc();
    VisionSetAltitude(vision_gimbal_data.Vision_r_yaw * DEGREE_2_RAD, vision_gimbal_data.Vision_r_pitch * DEGREE_2_RAD, 0);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_l_motor);
        DJIMotorStop(pitch_l_motor);
        DJIMotorStop(yaw_r_motor);
        DJIMotorStop(pitch_r_motor);
        
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_l_motor);
        DJIMotorEnable(pitch_l_motor);
        DJIMotorChangeFeed(yaw_l_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_r_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_l_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_r_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorSetRef(yaw_l_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // DJIMotorSetRef(pitch_l_motor, gimbal_cmd_recv.pitch);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_l_motor);
        DJIMotorEnable(pitch_l_motor);
        DJIMotorEnable(yaw_r_motor);
        DJIMotorEnable(pitch_r_motor);
        DJIMotorChangeFeed(yaw_l_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_r_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_l_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_r_motor, ANGLE_LOOP, MOTOR_FEED);
        // DJIMotorSetRef(yaw_l_motor, -gimbal_cmd_recv.yaw + YAW_L_INIT_ANGLE); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // DJIMotorSetRef(pitch_l_motor, pitch_r_angle);
        DJIMotorSetRef(yaw_r_motor, -gimbal_cmd_recv.yaw + YAW_R_INIT_ANGLE);
        DJIMotorSetRef(pitch_r_motor, -gimbal_cmd_recv.pitch + PITCH_R_INIT_ANGLE - 5.0);
        break;
    // 云台自瞄模式，自瞄计算使用相对母云台角度，发送时转换为实际角度
    case GIMBAL_VISION: 
        DJIMotorEnable(yaw_l_motor);
        DJIMotorEnable(pitch_l_motor);
        DJIMotorEnable(yaw_r_motor);
        DJIMotorEnable(pitch_r_motor);
        DJIMotorChangeFeed(yaw_l_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_r_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_l_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_r_motor, ANGLE_LOOP, MOTOR_FEED);


        // LIMIT_MIN_MAX(vision_gimbal_data.Vision_set_r_yaw, -90, 52);
        LIMIT_MIN_MAX(vision_gimbal_data.Vision_set_r_pitch, -180, 20);

        DJIMotorSetRef(yaw_r_motor, vision_gimbal_data.Vision_set_r_yaw);
        DJIMotorSetRef(pitch_r_motor, vision_gimbal_data.Vision_set_r_pitch);

    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_l_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}