#include "monitor.h"
#include "stdatomic.h"

static monitor_t monitor;

char mutex = 1;
static const char available = 1;

/* Emergencystop-variabel i monitor också!? */


monitor_t getState(){
    //mutex
    while(!atomic_compare_exchange_weak(&mutex, &available, 0)){
        // mutex upptagen, testa igen:)
    }

    monitor_t temp;
    memcpy(monitor, temp, sizeof(monitor_t));

    //unmutex
    atomic_exchange(&mutex, 1);

    return temp;
}

void setState(const state_t state, const sensorData_t sensorData){
    //mutex
    while(!atomic_compare_exchange_weak(&mutex, &available, 0)){
        // mutex upptagen, testa igen:)
    }
    
    memcpy(state, monitor.state, sizeof(state_t));
    memcpy(sensorData, monitor.sensorData, sizeof(sensorData_t));

    //unmutex
    atomic_exchange(&mutex, 1);
}
