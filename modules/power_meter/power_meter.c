#include "power_meter.h"
#include "bsp_iic.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include <stdint.h>

static PowerData_t power_data;
static float current_lsb = 0.001f;  // 1mA/bit (10A量程)
static float shunt_resistor = 0.002f; // 2mΩ分流电阻

static IICInstance *ina226_iic;  // 新增IIC实例指针

static uint16_t ReadRegister(uint8_t reg) {
    uint8_t buf[2];
    // 使用正确的IIC驱动函数
    IICAccessMem(ina226_iic, reg, buf, 2, IIC_READ_MEM, 1); // 1表示8位内存地址
    return (buf[0] << 8) | buf[1];
}

static void WriteRegister(uint8_t reg, uint16_t value) {
    uint8_t buf[2] = {value >> 8, value & 0xFF};
    // 使用正确的IIC驱动函数
    IICAccessMem(ina226_iic, reg, buf, 2, IIC_WRITE_MEM, 1); // 1表示8位内存地址
}

void PowerMeter_Init(void) {
    // 初始化IIC实例
    static IIC_Init_Config_s iic_conf = {
        .handle = &hi2c2,  // 假设使用I2C2，根据实际硬件连接修改
        .dev_address = INA226_ADDR << 1,  // 7位地址左移1位
        .work_mode = IIC_BLOCK_MODE,
        .callback = NULL,
        .id = "INA226"
    };
    ina226_iic = IICRegister(&iic_conf);

    // 配置测量参数
    uint16_t config = INA226_CONFIG_MODE | INA226_CONFIG_VRANGE | INA226_CONFIG_CT;
    WriteRegister(INA226_REG_CONFIG, config);
    
    // 计算并设置校准值
    uint16_t cal = (uint16_t)(0.00512 / (current_lsb * shunt_resistor));
    WriteRegister(INA226_REG_CALIB, cal);
    
    // 初始值清零
    power_data = (PowerData_t){0};
}

void PowerMeter_Update(void) {
    power_data.bus_voltage = ReadRegister(INA226_REG_BUS_V) * 0.00125f; // 1.25mV/LSB
    int16_t current_raw = (int16_t)ReadRegister(INA226_REG_CURRENT);
    power_data.current = current_raw * current_lsb;
    power_data.power = ReadRegister(INA226_REG_POWER) * 25 * current_lsb;
    power_data.timestamp = DWT_GetTimeline_s();
}

const PowerData_t* PowerMeter_GetData(void) {
    return &power_data;
}
