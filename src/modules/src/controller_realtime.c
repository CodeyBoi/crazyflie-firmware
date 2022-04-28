#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

//Test
#include "commander.h"
#include "system.h"
#include <string.h>

// Allocate memory for a queue with length 1
static xQueueHandle inputQueue;
STATIC_MEM_QUEUE_ALLOC(inputQueue, 1, sizeof(int));

static void realtimeTask(void*);
STATIC_MEM_TASK_ALLOC(realtimeTask, REALTIME_TASK_STACKSIZE);

static bool isInit = false;

void realtimeTaskInit() {
  inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);
  // TODO
  STATIC_MEM_TASK_CREATE(realtimeTask, realtimeTask, REALTIME_TASK_NAME, NULL, REALTIME_TASK_PRI);
  isInit = true;
}

bool realtimeTaskTest() {
  return isInit;
}

static setpoint_t setpoint;

static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw)
{
  memset(setpoint, 0, sizeof(setpoint_t));

  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeAbs;
  setpoint->attitudeRate.yaw = yaw;


  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->velocity.x = x;
  setpoint->velocity.y = y;

  setpoint->velocity_body = true;
}

static void realtimeTask(void *parameters) {
  DEBUG_PRINT("Realtime task main function is running!\n");
  //uint32_t lastWakeTime = xTaskGetTickCount();   
  systemWaitStart();
  vTaskDelay(1000);
  for (;;) {
    vTaskDelay(500);
    setHoverSetpoint(&setpoint, 0, 0, 4, 0);
    commanderSetSetpoint(&setpoint, 3);

    /*
    int input;
    if (pdTRUE == xQueueReceive(inputQueue, &input, portMAX_DELAY)) {
      // Do stuff with input
    }
    */

  }
}

void realtimeTaskEnqueueInput(int value) {
  xQueueOverwrite(inputQueue, &value);
}