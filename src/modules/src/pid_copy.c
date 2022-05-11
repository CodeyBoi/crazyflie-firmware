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
 *
 * pid.c - implementation of the PID regulator
 */

#ifdef IMPROVED_BARO_Z_HOLD
#define PID_FILTER_ALL
#endif

#include "pid.h"
#include "num.h"
#include <math.h>
#include <float.h>

void pidInit(PidObject* pid, const float ref, const float kp,
             const float ki, const float kd, 
              const float N, const float beta, const float h,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  pid->error         = 0;
  pid->prevError     = 0;
  pid->y             = 0;
  pid->yOld          = 0;
  pid->integ         = 0;
  pid->deriv         = 0;
  pid->ref           = ref;
  pid->kp            = kp;
  pid->ki            = ki;
  pid->kd            = kd;
  pid->u             = 0;
  pid->v             = 0;
  pid->K             = kp
  pid->Ti            = kp/k
  pid->Td            = kd/k
  //Tr for recomputation (see slide 58 lecture 6)
  pid->Tr            = 0;
  pid->N             = N;
  pid->Beta          = beta 
  pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;
  pid->h             = h;
  pid->ad            = Td/(Td + N*h)
  pid->bd            = kp*ad*N
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  }
}

float calculateOutput(float yref, float newY)
{
  pid->y = newY;
  pid->error = yref - pid->y;
  pid->deriv = pid->ad*pid->deriv - pid->bd*(pid->y - pid->yOld);
  pid->v = pid->K*(pid->Beta*yref - pid->y) + pid->integ + pid->deriv;
}

float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output = 0.0f;

    //---------------------------------------------------------------------------------------------------------------------//
    //-------------------------------------------------My code-------------------------------------------------------------//

    pid->y = measured;
    if (updateError)
    {
        pid->error = pid->ref - pid->y;
    }
    //Calculating the proportional part of the output, I think
    pid->outP = pid->kp * pid->error;
    output += pid->outP;
    //Using backward difference to compute the D part
    pid->deriv = pid->ad*pid->deriv - pid->bd*(pid->y - pid->yOld);
    //Doing some weird check if the D-part is activated?
    #ifdef PID_FILTER_ALL
      pid->deriv = deriv;
    #else
      if (pid->enableDFilter){
        pid->deriv = lpf2pApply(&pid->dFilter, deriv);
      } else {
        pid->deriv = deriv;
      }
    #endif
    if (isnan(pid->deriv)) {
      pid->deriv = 0;
    }

    pid->outD = pid->deriv;
    output += pid->outD;

    pid->v = pid->K*(pid->Beta*yref - pid->y) + pid->integ + pid->deriv;

    //Check slide 73 lecture 6 for this expression. Should we add the term (h/Tr)*(u-v)?
    pid->integ = pid->integ + (pid->K*pid->h/pid->Ti)*pid->error 

    if(pid->iLimit != 0)
    {
    	pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
    }

    pid->outI = pid->integ;
    output += pid->outI;
    
    #ifdef PID_FILTER_ALL
      //filter complete output instead of only D component to compensate for increased noise from increased barometer influence
      if (pid->enableDFilter)
      {
        output = lpf2pApply(&pid->dFilter, output);
      }
      else {
        output = output;
      }
      if (isnan(output)) {
        output = 0;
      }
     #endif
      
    

    // Constrain the total PID output (unless the outputLimit is zero)
    if(pid->outputLimit != 0)
    {
      output = constrain(output, -pid->outputLimit, pid->outputLimit);
    }

    //I don't think this line below is necessary anymore as I never use it
    pid->prevError = pid->error;

    return output;

    
//-------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//

}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}


void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetref(PidObject* pid, const float ref)
{
  pid->ref = ref;
}

float pidGetref(PidObject* pid)
{
  return pid->ref;
}

bool pidIsActive(PidObject* pid)
{
  bool isActive = true;

  if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
  {
    isActive = false;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}
void pidSeth(PidObject* pid, const float h) {
    pid->h = h;
}

void filterReset(PidObject* pid, const float samplingRate, const float cutoffFreq, bool enableDFilter) {
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  }
}
