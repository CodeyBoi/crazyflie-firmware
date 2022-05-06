#ifndef __MONITOR_H__
#define __MONITOR_H__

#include "stabilizer_types.h"



typedef struct monitor_s {
    state_t state;
    sensorData_t sensorData;
} monitor_t;

monitor_t getState();

void setState(state_t, sensorData_t); 

#endif