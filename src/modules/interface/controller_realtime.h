#pragma once

#include <stdbool.h>

void realtimeTaskInit();
bool realtimeTaskTest();

void realtimeTaskEnqueueInput(int value);