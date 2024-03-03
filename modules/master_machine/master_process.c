/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
// #include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "crc.h"
#include "bsp_dwt.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static USARTInstance *vision_usart_instance;
// void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
// {
//     send_data.enemy_color = enemy_color;
//     send_data.work_mode = work_mode;
//     send_data.bullet_speed = bullet_speed;
// }

// void VisionSetAltitude(float yaw, float pitch, float roll)
// {
//     send_data.yaw = yaw;
//     send_data.pitch = pitch;
//     send_data.roll = roll;
// }

//硬件crc32校验
CRC_STATE check_data4_crc32(uint8_t *pbuffer,uint8_t length_4multi)
{
    uint32_t *p32;
    uint32_t crc_cal = 0;
    
    p32 = (uint32_t*)&pbuffer[0];
    crc_cal = HAL_CRC_Calculate(&hcrc,p32,(uint32_t)(length_4multi/4-1));

    p32 = (uint32_t*)&pbuffer[length_4multi-4];
    if (*p32 == crc_cal)
    {
      return CRC_RIGHT;
    }
    else
    {
      return CRC_WRONG;
    }
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_data(
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint8_t *tx_data          // 待发送的float数据
                            
                            )    // 待发送的数据帧长度
{
    static uint32_t crc;
    tx_buf[0] = tx_data[0];
    tx_buf[1] = tx_data[1];
    tx_buf[2] = tx_data[2];
    tx_buf[3] = tx_data[4];
    tx_buf[4] = tx_data[3];
    tx_buf[5] = tx_data[6];
    tx_buf[6] = tx_data[5];
    tx_buf[7] = tx_data[8];
    tx_buf[8] = tx_data[7];
    tx_buf[9] = tx_data[10];
    tx_buf[10] = tx_data[9];
    tx_buf[11] = tx_data[11];
    crc=HAL_CRC_Calculate(&hcrc, (uint32_t *)&tx_buf[0], CV_SEND_LENGTH/4-1);
    tx_buf[12] = (uint8_t)(crc&0xff);
    tx_buf[13] = (uint8_t)((crc>>8)&0xff);
    tx_buf[14] = (uint8_t)((crc>>16)&0xff);
    tx_buf[15] = (uint8_t)((crc>>24)&0xff);
}
/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
uint16_t get_protocol_info(uint8_t *rx_buf, Vision_Recv_s *rx_data)         // 接收的float数据存储地址
{
    switch (rx_buf[0])
    {
    case 'A':
        if(check_data4_crc32(rx_buf,ACTION_DATA_LENGTH) == CRC_WRONG)
        {
            return DATA_STATE_WRONG;
        }
        else
        {
            rx_data->ACTION_DATA.sof = rx_buf[0];
            rx_data->ACTION_DATA.fire_times = rx_buf[1];
            rx_data->ACTION_DATA.relative_pitch = (int16_t)((rx_buf[2]>>8)|(rx_buf[3]<<8));
            rx_data->ACTION_DATA.relative_yaw = (int16_t)((rx_buf[4]>>8)|(rx_buf[5]<<8));
            rx_data->ACTION_DATA.reach_minute = (uint8_t)rx_buf[6];
            rx_data->ACTION_DATA.reach_second = (uint8_t)rx_buf[7];
            rx_data->ACTION_DATA.reach_second_frac = (uint16_t)((rx_buf[8]>>8)|(rx_buf[9]<<8));
            rx_data->ACTION_DATA.setting_voltage_or_rpm = (uint16_t)((rx_buf[10]>>8)|(rx_buf[11]<<8));
            rx_data->ACTION_DATA.crc_check = (uint32_t)((rx_buf[12]>>24)|(rx_buf[13]<<8)|(rx_buf[14]<<16)|(rx_buf[15]<<24));
            return DATA_STATE_ACTION;
        }
        break;
    case 'S':
        if (check_data4_crc32(rx_buf,SYN_DATA_LENGTH) == CRC_WRONG)
        {
            return DATA_STATE_WRONG;
        }
        else
        {
            rx_data->SYN_DATA.sof = rx_buf[0];
            rx_data->SYN_DATA.time_minute = rx_buf[1];
            rx_data->SYN_DATA.time_second = rx_buf[2];
            rx_data->SYN_DATA.time_second_frac = (uint16_t)((rx_buf[3]>>8)|(rx_buf[4]<<8));
            rx_data->SYN_DATA.null_7byte[0] = rx_buf[5];
            rx_data->SYN_DATA.null_7byte[1] = rx_buf[6];
            rx_data->SYN_DATA.null_7byte[2] = rx_buf[7];
            rx_data->SYN_DATA.null_7byte[3] = rx_buf[8];
            rx_data->SYN_DATA.null_7byte[4] = rx_buf[9];
            rx_data->SYN_DATA.null_7byte[5] = rx_buf[10];
            rx_data->SYN_DATA.null_7byte[6] = rx_buf[11];
            rx_data->SYN_DATA.crc_check = (uint32_t)((rx_buf[12]>>24)|(rx_buf[13]<<8)|(rx_buf[14]<<16)|(rx_buf[15]<<24));
            rx_data->SYN_DATA.dwttime = DWT_GetTimeline_ms();
            return DATA_STATE_SYN;
        }
        break;
    default:
        return DATA_STATE_WRONG;
        break;
    }
    return 0;
}



/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"



/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    // uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    get_protocol_info(vision_usart_instance->recv_buff, &recv_data);
    // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend(Vision_Send_s *tx_data)
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    // static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    // static uint16_t tx_len;
    // TODO: code to set flag_register
    // flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    
    float dt = DWT_GetTimeline_ms() - recv_data.SYN_DATA.dwttime;
    recv_data.SYN_DATA.dwttime = DWT_GetTimeline_ms();
    if (recv_data.SYN_DATA.time_second_frac + dt >= 1000)
    {
        recv_data.SYN_DATA.time_second_frac = (recv_data.SYN_DATA.time_second_frac + (uint16_t)dt)%1000;
        recv_data.SYN_DATA.time_second = (recv_data.SYN_DATA.time_second + (recv_data.SYN_DATA.time_second_frac+(uint16_t)dt)/1000);
    } else {
        recv_data.SYN_DATA.time_second_frac = recv_data.SYN_DATA.time_second_frac + (uint16_t)dt;
    }
    if (recv_data.SYN_DATA.time_second >= 60)
    {
        recv_data.SYN_DATA.time_second = recv_data.SYN_DATA.time_second % 60;
        recv_data.SYN_DATA.time_minute = recv_data.SYN_DATA.time_minute + recv_data.SYN_DATA.time_second/60;
    }
    if (recv_data.SYN_DATA.time_minute >= 60)
    {
        recv_data.SYN_DATA.time_minute = recv_data.SYN_DATA.time_minute % 60;
    }
    tx_data->time_minute = recv_data.SYN_DATA.time_minute;
    tx_data->time_second = recv_data.SYN_DATA.time_second;
    tx_data->time_second_frac = recv_data.SYN_DATA.time_second_frac;
    
    get_protocol_send_data(send_buff, (uint8_t *)tx_data);
    USARTSend(vision_usart_instance, send_buff, sizeof(Vision_Send_s), USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    uint16_t flag_register;
    get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP
