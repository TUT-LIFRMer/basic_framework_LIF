#ifndef __POWER_METER_H
#define __POWER_METER_H

#include "bsp_iic.h"
#include "INA226.h"

typedef struct{
    float voltage;
    float current;
    float power;
}power_meter_data_t;

power_meter_data_t *get_power_meter_data(void);

#endif

