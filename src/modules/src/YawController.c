/*
 * YawController.c
 *
 *  Created on: Oct 25, 2016
 *      Author: J.W.A. van den Meijdenberg
 */

#include "YawController.h"
#include "pid.h"

#define PID_AlTITUDE_KP 15000.0
#define PID_ALTITUDE_KI 4000.0
#define PID_AlTITUDE_KD 8000.0
#define PID_ALTITUDE_INTEGRATION_LIMIT 20000.0
#define PID_ALTITUDE_OFFSET 36000.0
#define PID_ALTITUDE_DT 0.10

static bool isInit = false;
PidObject pidAltitude;
int16_t altitudeOutput;

void YawControllerInit(void)
{
	if (isInit)
	{
		return;
	}

	pidInit(&pidAltitude, 0, PID_AlTITUDE_KP, PID_ALTITUDE_KI, PID_AlTITUDE_KD, PID_ALTITUDE_DT); // Desired set to zero since reference is relative error between setpoint and real pos
	pidSetIntegralLimit(&pidAltitude, PID_ALTITUDE_INTEGRATION_LIMIT);
	pidSetOffset(&pidAltitude, PID_ALTITUDE_OFFSET);

	isInit = true;
	return;

}

void YawAltitudeController(setpoint_t *setpoint, positionMeasurement_t *ext_pos, float dt)
{
	pidSetDt(&pidAltitude, dt);
	setpoint->thrust = pidUpdate(&pidAltitude, -ext_pos->z, true);
}
