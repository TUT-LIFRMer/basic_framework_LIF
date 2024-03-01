#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "crc.h"
#include "master_process.h"

#define PROTOCOL_CMD_ID 0XA5
#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

#define ACTION_DATA_LENGTH 16
#define SYN_DATA_LENGTH 16
#define CV_MAX_TOPIC_LENGTH 16

typedef enum
{
    CRC_RIGHT=1,
    CRC_WRONG=0
}CRC_STATE;


typedef enum
{
    DATA_STATE_ACTION,
    DATA_STATE_POS,
    DATA_STATE_SYN,
    DATA_STATE_WRONG

}DATA_STATE;

typedef struct
{
	struct
	{
		uint8_t sof;
		uint16_t data_length;
		uint8_t crc_check; // 帧头CRC校验
	} header;			   // 数据帧头
	uint16_t cmd_id;	   // 数据ID
	uint16_t frame_tail;   // 帧尾CRC校验
} protocol_rm_struct;




/*更新发送数据帧，并计算发送数据帧长度*/
extern void get_protocol_send_data(
                            POS_DATA *tx_data,         // 待发送的数据帧
                            uint8_t *tx_buf);    // 待发送的数据帧长度

/*接收数据处理*/
uint16_t get_protocol_info(uint8_t *rx_buf,			 // 接收到的原始数据
						   uint8_t *rx_data);			 // 接收的float数据存储地址

#endif
