/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * observer.h: not a non-observer orchestrator
 */
#ifndef OBSERVER_H_
#define OBSERVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "estimator.h"

//#define EMERGENCY_STOP_TIMEOUT_DISABLED (-1)

/**
 * Initialize the observer subsystem and launch the observer loop task.
 * The observer loop task will wait on systemWaitStart() before running.
 */
void observerInit(StateEstimatorType estimator);

/**
 * Test the observer subsystem. Calls test for all the observer related
 * sensors.
 * @return True if all test has passed. False otherwise.
 */
bool observerTest(void);

/**
 * Enable emergency stop, will shut-off energy to the motors.
 */
void observerSetEmergencyStop();

/**
 * Disable emergency stop, will enable energy to the motors.
 */
void observerResetEmergencyStop();

/**
 * Restart the countdown until emergercy stop will be enabled.
 *
 * @param timeout Timeout in observer loop tick. The observer loop rate is
 *                RATE_MAIN_LOOP.
 */
void observerSetEmergencyStopTimeout(int timeout);


#endif /* OBSERVER_H_ */
