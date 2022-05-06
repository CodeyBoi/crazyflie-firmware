#include "monitor.h"
#include "stdatomic.h"
#include "debug.h"
#include <string.h>

static monitor_t monitor;
static monitor_t temp;

char mutex = 1;
static const char available = 1;

/* Emergencystop-variabel i monitor också!? */

static int counter = 0;

monitor_t getState(){
    //mutex
    while(!atomic_compare_exchange_weak(&mutex, &available, 0)){
        // mutex upptagen, testa igen:)
    }

    memcpy(&temp, &monitor, sizeof(monitor_t));

    //unmutex
    atomic_exchange(&mutex, 1);

    return temp;
}

//vi vill nog ha pointers här
void setState(state_t state, sensorData_t sensorData){
    //mutex
    while(!atomic_compare_exchange_weak(&mutex, &available, 0)){
        // mutex upptagen, testa igen:)
        DEBUG_PRINT("atomic comp exch failed\n");
    }

    if(++counter >= 1000){
        DEBUG_PRINT("Before memcpy. state.z: %d\n", (int) (10000*state.position.z));
    }

    memcpy(&monitor.state, &state, sizeof(state_t));
    memcpy(&monitor.sensorData, &sensorData, sizeof(sensorData_t));

    if(counter >= 1000){
        DEBUG_PRINT("After memcpy. monitor.state.z: %d\n", (int) (10000*monitor.state.position.z));
        counter = 0;
    }

    //unmutex
    atomic_exchange(&mutex, 1);
}
