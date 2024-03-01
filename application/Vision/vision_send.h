#ifndef VISIONSEND_H
#define VISIONSEND_H
#include "master_process.h"
#include "seasky_protocol.h"
extern void VisionStartInit();

extern void VisionSendTask();

/**
 * @brief 发送视觉数据
 *
 */
extern void VisionSend();




#endif // SHOOT_H