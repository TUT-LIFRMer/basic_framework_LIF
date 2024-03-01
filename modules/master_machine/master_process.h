#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 36u

extern float dwttime;

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	struct 
	{
    	char   sof                ;
    	int8_t fire_times         ;
    	int16_t relative_pitch    ;
    	int16_t relative_yaw      ;
    	uint8_t reach_minute      ;
   		uint8_t reach_second      ;
    	uint16_t reach_second_frac;
    	int16_t setting_voltage_or_rpm;
    	uint32_t crc_check        ;

	}ACTION_DATA;
	struct
	{
    	char   sof                  ;
    	uint8_t time_minute         ;
   		uint8_t time_second         ;
    	uint16_t time_second_frac   ;
    	char null_7byte[7]          ;
    	uint32_t crc_check          ;
    
	}SYN_DATA;
} Vision_Recv_s;

typedef struct
{

    char sof;
    uint8_t time_minute;
    uint8_t time_second;
    uint16_t time_second_frac;
    int16_t present_pitch;
    int16_t present_yaw;
    int16_t present_debug_value;
    char null_byte;
    uint32_t crc_value;

}POS_DATA;
extern POS_DATA vision_send_data;//视觉数据发送缓存区
typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;
	Bullet_Speed_e bullet_speed;

	float yaw;
	float pitch;
	float roll;
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

// /**
//  * @brief 发送视觉数据
//  *
//  */
// void VisionSend();

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif // !MASTER_PROCESS_H