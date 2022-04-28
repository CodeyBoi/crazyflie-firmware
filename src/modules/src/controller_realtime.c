#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

#include "ledseq.h"

//Test - ta bort denna sida
#include "stabilizer_types.h"

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

/* LED blinking stuff */


static void realtimeTask(void *parameters) {
  DEBUG_PRINT("Realtime task main function is running!\n");
  uint32_t lastWakeTime = xTaskGetTickCount();   
  for (;;) {
    int input;
    bool result = ledseqRun(&seq_test);
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
    DEBUG_PRINT("Result from ledseq %i\n", result);
    if (pdTRUE == xQueueReceive(inputQueue, &input, portMAX_DELAY)) {
      // Do stuff with input
    }
  }
}

void realtimeTaskEnqueueInput(int value) {
  xQueueOverwrite(inputQueue, &value);
}