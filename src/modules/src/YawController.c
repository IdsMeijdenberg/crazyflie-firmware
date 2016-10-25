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
#define PID_AlTITUDE_KD 12000.0
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

	pidInit(&pidAltitude, 0, PID_AlTITUDE_KP, PID_ALTITUDE_KI, PID_AlTITUDE_KD, PID_ALTITUDE_DT);
	pidSetIntegralLimit(&pidAltitude, PID_ALTITUDE_INTEGRATION_LIMIT);


	isInit = true;
	return;

}

void YawAltitudeAndAttitudeController(setpoint_t *setpoint, positionMeasurement_t *ext_pos, float dt)
{
	pidSetDt(&pidAltitude, dt);
	setpoint->thrust = pidUpdate(&pidAltitude, ext_pos->z, true);
}
