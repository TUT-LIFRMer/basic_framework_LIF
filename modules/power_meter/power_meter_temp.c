#include "power_meter.h"
#include "general_def.h"
#include "bsp_i2c.h"
#include "message_center.h"
#include "math.h"

// INA226寄存器地址
#define INA226_REG_CONFIG     0x00
#define INA226_REG_SHUNT_V    0x01
#define INA226_REG_BUS_V      0x02
#define INA226_REG_POWER      0x03
#define INA226_REG_CURRENT    0x04
#define INA226_REG_CALIB      0x05

// 模块静态存储
static power_meter_config_t meter_config;
static const float LSB_CURRENT[] = {0.0001f, 0.0005f}; // 对应2mΩ和10mΩ的电流LSB

// 私有函数声明
static void power_meter_cmd_callback(uint8_t i2c_ch, uint8_t* data, uint16_t size);
static uint8_t write_register(uint8_t reg, uint16_t value);
static void start_async_read(uint8_t reg);
static float convert_to_voltage(uint16_t raw);
static float convert_to_current(uint16_t raw);
static float convert_to_power(uint16_t raw);

// 异步操作状态
typedef enum {
    PM_STATE_IDLE,
    PM_STATE_READING_SHUNT,
    PM_STATE_READING_BUS,
    PM_STATE_READING_POWER
} pm_state_t;

static pm_state_t current_state = PM_STATE_IDLE;
static power_data_t current_data;

// 初始化函数
uint8_t power_meter_init(power_meter_config_t* config) {
    if (config == NULL) {
        return 1; // 参数错误
    }
    
    // 保存配置
    meter_config = *config;
    if (meter_config.i2c_addr == 0) {
        meter_config.i2c_addr = 0x40; // 设置默认地址
    }

    // 配置校准寄存器
    const float shunt_val = (meter_config.shunt == SHUNT_2MOHM) ? 0.002f : 0.01f;
    const float current_lsb = LSB_CURRENT[meter_config.shunt];
    const uint16_t cal = (uint16_t)(0.00512f / (current_lsb * shunt_val));
    
    // 写入配置寄存器 (平均模式128次，总线电压测量时间1.1ms，分流电压测量时间1.1ms)
    uint16_t config_val = (0x05 << 9) | (0x05 << 6) | (0x07 << 3) | 0x07;
    if (write_register(INA226_REG_CONFIG, config_val) != 0) {
        return 2; // 配置失败
    }
    
    // 写入校准寄存器
    if (write_register(INA226_REG_CALIB, cal) != 0) {
        return 3; // 校准失败
    }

    return 0;
}

// 获取测量数据
uint8_t power_meter_get_data(power_data_t* data) {
    if (data == NULL) return 1;
    
    uint16_t raw_shunt_v, raw_bus_v, raw_power;
    
    if (read_register(INA226_REG_SHUNT_V, &raw_shunt_v) ||
        read_register(INA226_REG_BUS_V, &raw_bus_v) ||
        read_register(INA226_REG_POWER, &raw_power)) {
        data->error_code = 4; // 读取错误
        return 1;
    }
    
    data->voltage_v = convert_to_voltage(raw_bus_v);
    data->current_a = convert_to_current(raw_shunt_v);
    data->power_w = convert_to_power(raw_power);
    data->error_code = 0;
    
    return 0;
}

// 私有函数实现
static uint8_t write_register(uint8_t reg, uint16_t value) {
    uint8_t tx_data[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    return BSP_I2C_Write(meter_config.i2c_channel, 
                        meter_config.i2c_addr,
                        reg, tx_data, 2);
}

static uint8_t read_register(uint8_t reg, uint16_t* value) {
    uint8_t rx_data[2];
    if (BSP_I2C_Read(meter_config.i2c_channel,
                    meter_config.i2c_addr,
                    reg, rx_data, 2) != BSP_OK) {
        return 1;
    }
    *value = (rx_data[0] << 8) | rx_data[1];
    return 0;
}

static float convert_to_voltage(uint16_t raw) {
    return raw * 0.00125f; // 1.25mV/LSB
}

static float convert_to_current(uint16_t raw) {
    return raw * LSB_CURRENT[meter_config.shunt];
}

static float convert_to_power(uint16_t raw) {
    return raw * LSB_CURRENT[meter_config.shunt] * 25.0f; // Power LSB = 25 * Current LSB
}
