#include "vision_send.h"
#include "robot_def.h"

//module
#include "message_center.h"
#include "general_def.h"
#include "master_process.h"
#include "seasky_protocol.h"

//bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "bsp_usb.h"

static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息
static Publisher_t *vision_cmd_pub;           // 视觉控制消息发布者
static Vision_Ctrl_Cmd_s vision_cmd_data;      // 传递视觉控制的控制信息（从视觉控制命令的内存中将需要的控制命令提取出来）
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回（保存解析后的视觉控制命令内存所在的地址）
// static Vision_Send_s vision_send_data;  // 视觉发送数据
// static POS_DATA vision_send_data;//视觉发送数据

uint8_t vision_tx_buf[sizeof(POS_DATA)]={0};//视觉发送缓存区

void VisionStartInit(){
    vision_recv_data = VisionInit(&huart1); // 视觉虚拟串口初始化
   
    vision_cmd_pub = PubRegister("vision_cmd", sizeof(Vision_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    memset(&vision_cmd_data, 0, sizeof(Vision_Ctrl_Cmd_s));

};


void VisionSend()
{
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(&vision_send_data, vision_tx_buf);
    USBTransmit(vision_tx_buf, sizeof(POS_DATA));
}

void VisionSendTask(){
    SubGetMessage(gimbal_feed_sub, (void *)&gimbal_fetch_data);
    float dt = DWT_GetTimeline_ms() - dwttime;
    vision_send_data.sof = 'P';
    if (vision_send_data.time_second_frac+dt>=1000)
    {
        vision_send_data.time_second = vision_send_data.time_second + ((int8_t)((float)vision_send_data.time_second_frac+dt)/1000);
        vision_send_data.time_second_frac = ((int16_t)((float)vision_send_data.time_second_frac+dt))%1000;
    } else {
        vision_send_data.time_second_frac = (int16_t)((float)vision_send_data.time_second_frac + dt);
    }
    if (vision_send_data.time_second>=60)
    {
        vision_send_data.time_minute = vision_send_data.time_minute + vision_send_data.time_second/60;
        vision_send_data.time_second = vision_send_data.time_second%60;
    }
    if (vision_send_data.time_minute>=60)
    {
        vision_send_data.time_minute = vision_send_data.time_minute%60;
    }
    vision_send_data.present_pitch = (int16_t)gimbal_fetch_data.gimbal_imu_data.Pitch;
    vision_send_data.present_yaw = (int16_t)gimbal_fetch_data.gimbal_imu_data.Yaw;
    vision_send_data.present_debug_value = 0;
    vision_send_data.null_byte = 1;
    VisionSend();
    dwttime = DWT_GetTimeline_ms();
    vision_cmd_data.pitch = (float)vision_recv_data->ACTION_DATA.relative_pitch/10000;
    vision_cmd_data.yaw = (float)vision_recv_data->ACTION_DATA.relative_yaw/10000;
    vision_cmd_data.shoot_frequency = (float)vision_recv_data->ACTION_DATA.fire_times;
    vision_cmd_data.reach_minute = vision_recv_data->ACTION_DATA.reach_minute;
    vision_cmd_data.reach_second = vision_recv_data->ACTION_DATA.reach_second;
    vision_cmd_data.reach_second_frac = vision_recv_data->ACTION_DATA.reach_second_frac;
    PubPushMessage(vision_cmd_pub, (void *)&vision_cmd_data);
};