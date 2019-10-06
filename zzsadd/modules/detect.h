#ifndef __DETECT_H
#define __DETECT_H
#include "drv_can.h"

#define ONLINE 1
#define OFFLINE 0

extern uint8_t Devicestate[13];
extern uint8_t Offline[13];
void GetDeviceState(void);
void DeviceDetect(uint8_t *state,uint8_t *result,uint8_t time_out);
int JudgeDeviceState(int fps,int i);



#endif
