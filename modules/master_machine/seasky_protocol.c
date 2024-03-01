/**
 * @file seasky_protocol.c
 * @author Liu Wei
 * @author modified by Neozng
 * @brief RoBoMatster串口通信协议
 * @version 0.1
 * @date 2022-11-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "seasky_protocol.h"
#include "master_process.h"
#include "crc8.h"
#include "crc16.h"
#include "memory.h"


//LIF视觉串口校验
CRC_STATE check_data4_crc32(uint8_t *pbuffer,uint8_t length_4multi)
{
    uint32_t *p32;
    uint32_t crc_cal = 0;
    
    p32 = (uint32_t*)&pbuffer[0];
    crc_cal = HAL_CRC_Calculate(&hcrc,p32,(uint32_t)(length_4multi/4-1));

    p32 = (uint32_t*)&pbuffer[12];
    if (*p32 == crc_cal)
    {
      return CRC_RIGHT;
    }
    else
    {
      return CRC_WRONG;
    }
}


/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*获取CRC16校验码*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}

/*检验CRC16数据段*/
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0];
        if (CRC8_Check_Sum(&rx_buf[0], 4))
        {
            pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];
            pro->header.crc_check = rx_buf[3];
            pro->cmd_id = (rx_buf[5] << 8) | rx_buf[4];
            return 1;
        }
    }
    return 0;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/

void get_protocol_send_data(
                            POS_DATA *tx_data,         // 待发送的数据帧
                            uint8_t *tx_buf)    // 待发送的数据帧长度
{
    uint16_t *pu16;
    int16_t *pi16;
    uint32_t *p32;
    

    tx_buf[0] = tx_data->sof;
    tx_buf[1] = tx_data->time_minute;
    tx_buf[2] = tx_data->time_second;


    pu16 = (uint16_t*)&tx_buf[3];
    *pu16 = tx_data->time_second_frac;

    pi16 = (int16_t*)&tx_buf[5];
    *pi16 = tx_data->present_pitch;

    pi16 = (int16_t*)&tx_buf[7];
    *pi16 = tx_data->present_yaw;

    pi16 = (int16_t*)&tx_buf[9];
    *pi16 = tx_data->present_debug_value;
   
    tx_buf[11] = tx_data->null_byte;



    p32 = (uint32_t*)&tx_buf[0];
    tx_data->crc_value=HAL_CRC_Calculate(&hcrc,p32,3);
    p32 = (uint32_t*)&tx_buf[12];
    *p32 = tx_data->crc_value;
}
/*
    此函数用于处理接收数据，
    返回数据内容的id
*/

void receive_action_to_data(uint8_t *prx_data ,Vision_Recv_s *pout)
{
  pout->ACTION_DATA.sof                 = prx_data[0];
  pout->ACTION_DATA.fire_times          = prx_data[1];
  pout->ACTION_DATA.relative_pitch      = *((int16_t*) &prx_data[2]);
  pout->ACTION_DATA.relative_yaw        = *((int16_t*) &prx_data[4]);
  pout->ACTION_DATA.reach_minute        = prx_data[6];
  pout->ACTION_DATA.reach_second        = prx_data[7];
  pout->ACTION_DATA.reach_second_frac   = *((uint16_t*) &prx_data[8]);
  pout->ACTION_DATA.setting_voltage_or_rpm = *((int16_t*)&prx_data[10]);
  pout->ACTION_DATA.crc_check           = *((uint32_t*) &prx_data[12]);
}

void receive_syn_to_data(uint8_t *prx_data,Vision_Recv_s *pout)
{
  pout->SYN_DATA.sof = prx_data[0];
  pout->SYN_DATA.time_minute = *((uint8_t*)&prx_data[1]);
  pout->SYN_DATA.time_second = *((uint8_t*)&prx_data[2]);
  pout->SYN_DATA.time_second_frac = *((uint16_t*)&prx_data[3]);
  for (uint8_t i = 0; i < 7; i++)
  {
    pout->SYN_DATA.null_7byte[i] = prx_data[5+i];
  }  
  pout->SYN_DATA.crc_check = *((uint32_t*)&prx_data[12]);
}

uint16_t get_protocol_info(uint8_t *rx_buf,          // 接收到的原始数据
                           uint8_t *rx_data)         // 接收的float数据存储地址
{
    switch (rx_buf[0])
    {
    case 'A':
        if(check_data4_crc32(rx_buf,ACTION_DATA_LENGTH) != CRC_RIGHT)
        {
            return DATA_STATE_WRONG;
        }
        else
        {
            receive_action_to_data(rx_buf,(Vision_Recv_s *)rx_data);
            return DATA_STATE_ACTION;
        }
        break;
    case 'S':
        if (check_data4_crc32(rx_buf,SYN_DATA_LENGTH) != CRC_RIGHT)
        {
            return DATA_STATE_WRONG;
        }
        else
        {
            receive_syn_to_data(rx_buf,(Vision_Recv_s *)rx_data);
            return DATA_STATE_SYN;
        }
        break;
    default:
        return DATA_STATE_WRONG;
        break;
    }
    return 0;
}
