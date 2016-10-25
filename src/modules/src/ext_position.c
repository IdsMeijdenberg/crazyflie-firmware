/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 */

#include "FreeRTOS.h"
#include "task.h"

#include "ext_position.h"
#include "crtp.h"
#include "log.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#endif

/**
 * Position data cache
 */
typedef struct
{
  struct CrtpExtPosition targetVal[2];
  bool activeSide;
  uint32_t timestamp; // FreeRTOS ticks
} ExtPositionCache;

// Struct for logging position information
static ExtPositionCache crtpExtPosCache;
static void extPositionCrtpCB(CRTPPacket* pk);

static bool isInit = false;

void extPositionInit()
{
  if (isInit) {
    return;
  }

  crtpRegisterPortCB(CRTP_PORT_POSITION, extPositionCrtpCB);
  isInit = true;
}

static void extPositionCrtpCB(CRTPPacket* pk)
{
  crtpExtPosCache.targetVal[!crtpExtPosCache.activeSide] = *((struct CrtpExtPosition*)pk->data);
  crtpExtPosCache.activeSide = !crtpExtPosCache.activeSide;
  crtpExtPosCache.timestamp = xTaskGetTickCount();
}


bool getExtPosition(state_t *state, positionMeasurement_t *ext_pos, float dt_ExtPosition)
{
  // Only use position information if it's valid and recent
  if ((xTaskGetTickCount() - crtpExtPosCache.timestamp) < M2T(5)) {
    // Get the updated position from the mocap
    ext_pos->x = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].x;
    ext_pos->y = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].y;
    ext_pos->z = crtpExtPosCache.targetVal[crtpExtPosCache.activeSide].z;
    ext_pos->stdDev = 0.01;
    dt_ExtPosition = (xTaskGetTickCount() - crtpExtPosCache.timestamp)/1000.0;
#ifdef ESTIMATOR_TYPE_kalman
    stateEstimatorEnqueuePosition(&ext_pos);
#endif
    return true;
  }
  return false;
}



