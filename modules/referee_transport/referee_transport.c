/*
 * @Description: 
 * @Author: changfeng
 * @brief: 
 * @version: 
 * @Date: 2025-03-13 06:34:23
 * @LastEditors:  
 * @LastEditTime: 2025-03-13 07:12:25
 */
#include "referee_transport.h"

#define START_BYTE 0xAA
#define END_BYTE 0x55
#define REFEREE_DATA_CONTROL_FRAME_SIZE 139U

referee_info_t referee_data;
static uint8_t referee_init_flag;
static referee_info_t referee_recv_data;

static USARTInstance *referee_data_usart_instance;  // 裁判系统数据转发串口实例
static DaemonInstance *referee_data_daemo_instance; // 裁判系统数据转发进程守护实例

/**
 * @brief 裁判系统数据转发解析函数
 * @return 
 */
static void RefereeDataRxCallback()
{
    
}

/**
 * @brief 裁判系统数据转发离线回调函数 
 * @return
 */
static void RefereeDataLostCallback()
{
    memset(&radar_ctrl, 0, sizeof(radar_ctrl)); //清空cmd_vel数据
    USARTServiceInit(cmd_vel_usart_instance);   //尝试重新启动

    LOGWARNING("[Cmd_Vel] radar control lost");
}

/**
 * @brief 初始化裁判系统数据转发串口通信 
 * @return
 */
referee_info_t *RefereeDataTransportInit(UART_HandleTypeDef *referee_data_usart_handle)
{
    USART_Init_Config_s config;
    config.module_callback = RefereeDataRxCallback;
    config.usart_handle = referee_data_usart_handle;
    config.recv_buff_size = REFEREE_DATA_CONTROL_FRAME_SIZE;
    referee_data_usart_instance = USARTRegister(&config);
}


