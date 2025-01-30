#include "power_meter.h"
#include "bsp_iic.h"
#include "bsp_dwt.h"

static PowerData_t power_data;
static float current_lsb = 0.001f;  // 1mA/bit (10A量程)
static float shunt_resistor = 0.002f; // 2mΩ分流电阻

static uint16_t ReadRegister(uint8_t reg) {
    uint8_t buf[2];
    BSP_IIC_ReadBytes(INA226_ADDR, reg, buf, 2);
    return (buf[0] << 8) | buf[1];
}

static void WriteRegister(uint8_t reg, uint16_t value) {
    uint8_t buf[2] = {value >> 8, value & 0xFF};
    BSP_IIC_WriteBytes(INA226_ADDR, reg, buf, 2);
}

void PowerMeter_Init(void) {
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
    power_data.timestamp = BSP_DWT_GetTimeline();
}

const PowerData_t* PowerMeter_GetData(void) {
    return &power_data;
}
