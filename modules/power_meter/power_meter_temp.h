#ifndef __POWER_METER_H
#define __POWER_METER_H

#include "general_def.h"
#include <stdint.h>

// INA226默认地址(A0/A1接地)
#define INA226_ADDR         0x40
// 寄存器定义
#define INA226_REG_CONFIG   0x00
#define INA226_REG_SHUNT_V  0x01
#define INA226_REG_BUS_V    0x02
#define INA226_REG_POWER    0x03
#define INA226_REG_CURRENT  0x04
#define INA226_REG_CALIB    0x05

// 配置参数(模式:连续测量总线电压和电流)
#define INA226_CONFIG_MODE  (0x07 << 9)
// 总线电压量程:36V
#define INA226_CONFIG_VRANGE (0x01 << 4)
// 转换时间:1.1ms
#define INA226_CONFIG_CT    (0x04 << 3)

typedef struct {
    float bus_voltage;   // 总线电压(V)
    float current;       // 电流(A)
    float power;         // 功率(W)
    uint32_t timestamp;
} PowerData_t;

void PowerMeter_Init(void);
void PowerMeter_Update(void);
const PowerData_t* PowerMeter_GetData(void);

#endif
