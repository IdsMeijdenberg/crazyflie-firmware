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
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "ext_position.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"
#include "SimpleMultiRotorModel.h"
#include "YawController.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

static bool isInit;

// State variables for the stabilizer
static setpoint_t setpoint;
static setpoint_t setpoint_PID;
static sensorData_t sensorData;
static state_t state;
static control_t control;
static SMRM_state SMRM_roll;
static SMRM_state SMRM_pitch;
static SMRM_sampled s_roll;
static SMRM_sampled s_pitch;
static SMRM_control cont_roll;
static SMRM_control cont_pitch;
static positionMeasurement_t ext_pos;

float dt_ExtPosition = 0;

static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit();
  stateControllerInit();
  powerDistributionInit();
  SimpleMultiRotorInit();
  YawControllerInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();

  return pass;
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    if (getExtPosition(&state, &ext_pos, dt_ExtPosition)) 		// Receive position from GoT
    {
    	SimpleMultiRotorPosition(&SMRM_roll, &SMRM_pitch, &ext_pos); 	// Update x-position of roll and pitch by the measurement of position from GoT
    	SimpleMultiRotorSample(&SMRM_roll, &s_roll); 					// Update sampled signals x(n), x_hat(n), and v_hat(n) when there is a new measurement
    	SimpleMultiRotorSample(&SMRM_pitch, &s_pitch);

        YawAltitudeAndAttitudeController(&setpoint_PID, &ext_pos, dt_ExtPosition);
    }

#ifdef ESTIMATOR_TYPE_kalman
    stateEstimatorUpdate(&state, &sensorData, &control);
#else
    sensorsAcquire(&sensorData, tick);
    stateEstimator(&state, &sensorData, tick);
#endif

    SimpleMultiRotorUpdate(&SMRM_roll, &s_roll, sensorData.gyro.x, tick); 					// Update linear-model by first order euler approximation
    SimpleMultiRotorUpdate(&SMRM_pitch, &s_pitch, sensorData.gyro.y, tick);

    commanderGetSetpoint(&setpoint, &state);
    setpoint.thrust = setpoint_PID.thrust;

//    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    SimpleMultiRotorControl(&cont_roll,&SMRM_roll,&s_roll,tick); 		// Obtain control values: torque for pitch and roll
    SimpleMultiRotorControl(&cont_pitch,&SMRM_pitch,&s_pitch,tick);

    stateController(&control, &sensorData, &state, &setpoint, tick);
    powerDistribution(&control);

    tick++;
  }
}

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

LOG_GROUP_START(actuator)
LOG_ADD(LOG_FLOAT, aT, &control.thrust)
LOG_ADD(LOG_INT16, aR, &control.roll)
LOG_ADD(LOG_INT16, aP, &control.pitch)
LOG_ADD(LOG_INT16, aY, &control.yaw)
LOG_GROUP_STOP(actuator)

LOG_GROUP_START(SMRM_roll)
LOG_ADD(LOG_FLOAT, x_hat, &SMRM_roll.position.x_hat)
LOG_ADD(LOG_FLOAT, v_hat, &SMRM_roll.velocity.v_hat)
LOG_ADD(LOG_FLOAT, th_hat, &SMRM_roll.angle.theta_hat)
LOG_ADD(LOG_FLOAT, om_hat, &SMRM_roll.angular_velocity.omega_d)
LOG_GROUP_STOP(SMRM_roll)

LOG_GROUP_START(ext_pos)
  LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
  LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
  LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)
