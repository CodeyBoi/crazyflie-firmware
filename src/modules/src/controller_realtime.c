#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

#include "ledseq.h"

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
  DEBUG_PRINT("Realtime task main function is running!");

  ledseqRun(&seq_test);

  for (;;) {
    int input;
    if (pdTRUE == xQueueReceive(inputQueue, &input, portMAX_DELAY)) {
      // Do stuff with input
    }
  }
}

void realtimeTaskEnqueueInput(int value) {
  xQueueOverwrite(inputQueue, &value);
}