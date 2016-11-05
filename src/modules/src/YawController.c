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

#define DT_DYNAMIC_THRESHOLD (0.05f)
#define UPDATE_FREQUENCY_ALTITUDE 200

#define L_X (2.114)
#define L_V (2.236)
#define GRAVITY (9.91f)
#define MASS_CRAZYFLIE (0.039f)
#define THRUST_TO_G GRAVITY*MASS_CRAZYFLIE/42598.4

static bool isInit = false;
static uint32_t tick_last_dynamic;
PidObject pidAltitude;
int16_t altitudeOutput;
/*
 * Private functions
 */

static void DynamicEquationAltitude(altitude_state_t *state, positionMeasurement_t *ext_pos, setpoint_t *setpoint, float dt);

/*
 * Public functions
 */

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


void YawAltitudeRunDynamics(altitude_state_t *altitude_state, positionMeasurement_t *ext_pos, setpoint_t *setpoint, uint32_t tick)
{
	if (RATE_DO_EXECUTE(UPDATE_FREQUENCY_ALTITUDE, tick))
		{
		float dt_dynamic = (float)(tick - tick_last_dynamic)/1000.0;
				if (dt_dynamic > DT_DYNAMIC_THRESHOLD){
					dt_dynamic = DT_DYNAMIC_THRESHOLD;
				}
		DynamicEquationAltitude(altitude_state, ext_pos, setpoint, dt_dynamic);
		tick_last_dynamic = tick;
		}
}

static void DynamicEquationAltitude(altitude_state_t *state, positionMeasurement_t *ext_pos, setpoint_t *setpoint, float dt)
{

	state->input = setpoint->thrust*THRUST_TO_G - MASS_CRAZYFLIE*GRAVITY;

	float Y[2] = {0};
	float dY[2]= {0};

	Y[0] = state->x_hat;
	Y[1] = state->v_hat;

	dY[0] = Y[1] + L_X*(ext_pos->z - Y[0]);
	dY[1] = -state->input/MASS_CRAZYFLIE + L_V*(ext_pos->z - Y[0]);

	Y[0] = Y[0] + dY[0]*dt;									// First order euler approximation
	Y[1] = Y[1] + dY[1]*dt;

	state->x_hat = Y[0];
	state->v_hat = Y[1];
}
