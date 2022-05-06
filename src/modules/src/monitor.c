#include "monitor.h"
#include "stdatomic.h"
#include <string.h>

static monitor_t monitor;
static monitor_t temp;

char mutex = 1;
static const char available = 1;

/* Emergencystop-variabel i monitor också!? */


monitor_t getState(){
    //mutex
    while(!atomic_compare_exchange_weak(&mutex, &available, 0)){
        // mutex upptagen, testa igen:)
    }

    memcpy(&monitor, &temp, sizeof(monitor_t));

    //unmutex
    atomic_exchange(&mutex, 1);

    return temp;
}

//vi vill nog ha pointers här
void setState(state_t state, sensorData_t sensorData){
    //mutex
    while(!atomic_compare_exchange_weak(&mutex, &available, 0)){
        // mutex upptagen, testa igen:)
    }
    
    memcpy(&state, &monitor.state, sizeof(state_t));
    memcpy(&sensorData, &monitor.sensorData, sizeof(sensorData_t));

    //unmutex
    atomic_exchange(&mutex, 1);
}
