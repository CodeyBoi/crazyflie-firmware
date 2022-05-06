/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#define DEBUG_MODULE "STAB"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"

#include "observer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "crtp_localization_service.h"
#include "controller.h"
#include "power_distribution.h"
#include "collision_avoidance.h"
#include "health.h"
#include "supervisor.h"

#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"
#include "rateSupervisor.h"

#include "monitor.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

//static uint32_t inToOutLatency;

// State variables for the stabilizer
//static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;

// For scratch storage - never logged or passed to other subsystems.
//static setpoint_t tempSetpoint;

static StateEstimatorType estimatorType;

static STATS_CNT_RATE_DEFINE(observerRate, 500);
static rateSupervisor_t rateSupervisorContext;
//static bool rateWarningDisplayed = false;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressed;

/*
static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed;
*/

STATIC_MEM_TASK_ALLOC(observerTask, OBSERVER_TASK_STACKSIZE);

static void observerTask(void* param);

/*
static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}
*/

static void compressState()
{
  stateCompressed.x = state.position.x * 1000.0f;
  stateCompressed.y = state.position.y * 1000.0f;
  stateCompressed.z = state.position.z * 1000.0f;

  stateCompressed.vx = state.velocity.x * 1000.0f;
  stateCompressed.vy = state.velocity.y * 1000.0f;
  stateCompressed.vz = state.velocity.z * 1000.0f;

  stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
  stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
  stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;

  float const q[4] = {
    state.attitudeQuaternion.x,
    state.attitudeQuaternion.y,
    state.attitudeQuaternion.z,
    state.attitudeQuaternion.w};
  stateCompressed.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
}

/*
static void compressSetpoint()
{
  setpointCompressed.x = setpoint.position.x * 1000.0f;
  setpointCompressed.y = setpoint.position.y * 1000.0f;
  setpointCompressed.z = setpoint.position.z * 1000.0f;

  setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
  setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
  setpointCompressed.vz = setpoint.velocity.z * 1000.0f;

  setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
  setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
  setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
}
*/

void observerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  //controllerInit(ControllerTypeAny); //TODO Flytta denna till nån annan stans (system.c?)
  powerDistributionInit();
  collisionAvoidanceInit();
  estimatorType = getStateEstimator();
  //controllerType = getControllerType(); //TODO samma som ovan

  STATIC_MEM_TASK_CREATE(observerTask, observerTask, OBSERVER_TASK_NAME, NULL, OBSERVER_TASK_PRI);

  isInit = true;
}

bool observerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  //pass &= controllerTest();
  //pass &= powerDistributionTest();
  //pass &= collisionAvoidanceTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void observerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_OBSERVER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  DEBUG_PRINT("Wait for sensor calibration...\n");

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_OBSERVER_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

  DEBUG_PRINT("Ready to observe.\n");

  while(1) {
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();

    // update sensorData struct (for logging variables)
    sensorsAcquire(&sensorData, tick);

    // allow to update estimator dynamically
    if (getStateEstimator() != estimatorType) {
        stateEstimatorSwitchTo(estimatorType);
        estimatorType = getStateEstimator();
    }

    stateEstimator(&state, tick);
    compressState();
  
    checkEmergencyStopTimeout();

    //
    // The supervisor module keeps track of Crazyflie state such as if
    // we are ok to fly, or if the Crazyflie is in flight.
    //
    supervisorUpdate(&sensorData);

    //här kan vi få race kanske om vi läser sensorvärden så långt innan vi skriver?
    setState(state, sensorData);

    // Ändra detta så det funkar med ny struktur?
    /*
    calcSensorToOutputLatency(&sensorData);
    tick++;
    STATS_CNT_RATE_EVENT(&stabilizerRate);

    if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount())) {
        if (!rateWarningDisplayed) {
            DEBUG_PRINT("WARNING: stabilizer loop rate is off (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
            rateWarningDisplayed = true;
        }
    }
    */
  }
}

void observerSetEmergencyStop()
{
  //TODO monitor?
  emergencyStop = true;
}

void observerResetEmergencyStop()
{
  //TODO monitor?
  emergencyStop = false;
}

void observerSetEmergencyStopTimeout(int timeout)
{
  //TODO monitor?
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}