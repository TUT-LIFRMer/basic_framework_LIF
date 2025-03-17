# NO_Power_Meter功率详解

## 在`chassis.c`中——（因为2025届RM裁判系统不能读取底盘功率，所以有部分代码会有删减）

1、电源信息映射：
`chassis_power`：直接从`referee_data->PowerHeatData.chassis_power`赋值。
`chassis_power_limit`：从`referee_data->GameRobotState.chassis_power_limit`获取，并强制转换为
float类型。

2、能量信息映射：
`buffer_energy`：从`referee_data->PowerHeatData.buffer_energy`获取，并强制转换为float类型。

3、热量信息映射：
`shoot_heat`：从`referee_data->PowerHeatData.shooter_17mm_1_barrel_heat`获取，并强制转换为float类型。

---

## 在`robot_cmd.c`中

MouseKeySet（）函数的功能涵盖了：

底盘速度调整： 根据底盘功率限制 `(chassis_power_limit) `设置底盘速度缓冲值 `(chassis_speed_buff)`。

通过一系列 `if-else` 判断，将不同的功率限制映射为对应的速度值（范围从 14000 到 38000）。

默认速度值为 15000。

---

## 在`robot_def.h`中

`Chassis_Power_Data_s` 是一个简单的结构体，用于表示底盘的功率控制信息。目前仅包含一个成员变量 `chassis_power_mx`表示底盘的最大功率限制。

`Chassis_Upload_Data_s` 是一个用于底盘反馈数据的结构体，包含了底盘运行状态的关键信息。这些信息可以被其他模块（如 cmd 或其他应用）订阅和使用。

---

## 在`referee_protocol.h`中

`ID_power_heat_data = 0x0202` 是一个命令码（CmdID），用于标识裁判系统协议中的“实时功率热量数据”。该数据包含了机器人底盘的电压、电流、功率、缓冲能量以及射击系统的热量等关键信息;在 chassis.c 文件中，这部分数据被解析并映射到底盘反馈结构体 Chassis_Upload_Data_s 中。

`LEN_power_heat_data = 16` 表示命令码 0x0202 的数据段长度为 16 字节，用于传输实时功率和热量相关的数据。

`chassis_power_limit` 表示底盘功率的限制值，通常用于防止过载。
`power_management_gimbal_output`、`power_management_chassis_output` 和 `power_management_shooter_output` 是三位布尔标志，分别表示云台、底盘和射击器的功率管理状态。

`ext_power_heat_data_t` 是一个结构体类型，用于表示实时功率和热量数据。

---

## 在`referee_task.c`中

`UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");`

`UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);`

这两行代码的作用是绘制并刷新一个静态字符显示区域，用于在UI上显示“Power:”文本。

`RobotModeTest`是一个测试用函数，通过周期性切换机器人模式，模拟不同状态下的行为。它涵盖了底盘、云台、射击、摩擦和盖板等多个子系统的状态设置，并动态调整功率数据。

`if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)`用于检测功率数据是否发生变化，并在发生变化时更新UI界面中的功率显示。

`if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)`检测底盘功率数据是否发生变化，并在发生变化时设置标志位 Power_flag，以便后续更新UI界面中的功率显示。

`case ID_power_heat_data: // 0x0202memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);`处理裁判系统发送的 ID_power_heat_data（命令码 0x0202）数据包，并将接收到的数据解析后存储到 referee_info.PowerHeatData 结构体中。

## 在`rm_referee.h`中

完成了包括

1、数据定义：在 referee_info_t 和 Referee_Interactive_info_t 结构体中定义了功率相关的字段。

2、数据接收与解析：通过 JudgeReadData 函数解析裁判系统发送的功率数据包。

3、动态更新：通过标志位检测功率值的变化，并更新基准值。

4、数据显示：在UI界面中动态显示功率数值和能量条。

---

## 在`super_cap.c`中

`SuperCapRxCallback`实现了从接收缓冲区的第 4 和第 5 字节提取功率值，并将其组合为一个 16 位无符号整数。

---

## 在`super_cap.h`中

`SuperCap_Msg_s`是一个简单的结构体，用于存储超级电容模块的电压、电流和功率信息。它的设计紧凑且高效，适用于嵌入式系统中的 CAN 数据传输和解析场景。

---

