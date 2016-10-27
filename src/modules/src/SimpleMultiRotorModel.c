/*
 * Constants used
 */
#include "SimpleMultiRotorModel.h"

#define K_X  (1.0708f)
#define K_V  (0.4674f)
#define K_P  (0.2319f)
#define K_D  (0.0251f)
#define L_X  (2.9303f)
#define L_V  (4.0459f)
#define L_TH (0.3367f)
#define L_OM (0.1365f)
#define GRAVITY (9.91f)
#define J_ROLLPITCH (2.3951e-5f)

#define PI	3.14159265358979f
#define DEG_TO_RAD (PI/180.0f)
#define UPDATE_FREQUENCY 1000
#define CONTROL_RATE 500

static int32_t lastUpdate;
static bool isInit = false;

static void DynamicEquationSMRM(SMRM_state *SMRM_st, const SMRM_sampled *SMRM_samp, const float sensorData, float dt);
static void SimpleMultiRotorPosition(SMRM_state *SMRM_roll, SMRM_state *SMRM_pitch, const positionMeasurement_t *pos);
static void SimpleMultiRotorSample(const SMRM_state *SMRM_st, SMRM_sampled *SMRM_samp);
static void SimpleMultiRotorUpdate(SMRM_state *SMRM_st,		const SMRM_sampled *SMRM_samp,
															const float sensorData,
															const uint32_t tick);
static void SimpleMultiRotorControl(SMRM_control *SMRM_cont, 	const SMRM_state*SMRM_st,
														const SMRM_sampled *SMRM_samp,
														const uint32_t tick);

void SimpleMultiRotorInit(void)
{
	if(isInit)
		return;

	isInit = true;
	return;
}

void SimpleMultiRotorNewPosition(SMRM_state *SMRM_roll, SMRM_state *SMRM_pitch,
														positionMeasurement_t *ext_pos,
														SMRM_sampled *s_roll,
														SMRM_sampled *s_pitch)
{
	SimpleMultiRotorPosition(SMRM_roll, SMRM_pitch, ext_pos); 	// Update x-position of roll and pitch by the measurement of position from GoT
	SimpleMultiRotorSample(SMRM_roll, s_roll); 					// Update sampled signals x(n), x_hat(n), and v_hat(n) when there is a new measurement
	SimpleMultiRotorSample(SMRM_pitch, s_pitch);
}

void SimpleMultiRotorRunDynamics(SMRM_state *SMRM_roll, SMRM_state *SMRM_pitch,
														SMRM_sampled *s_roll,
														SMRM_sampled *s_pitch,
														const float GyroX,
														const float GyroY,
														const uint32_t tick)
{
	    SimpleMultiRotorUpdate(SMRM_roll, s_roll, GyroX, tick);
	    SimpleMultiRotorUpdate(SMRM_pitch, s_pitch, GyroY, tick);
}

void SimpleMultiRotorTorque(SMRM_control *cont_roll, 	SMRM_control *cont_pitch,
														SMRM_state *SMRM_roll,
														SMRM_state *SMRM_pitch,
														SMRM_sampled *s_roll,
														SMRM_sampled *s_pitch,
														const uint32_t tick)
{
	    SimpleMultiRotorControl(cont_roll,SMRM_roll,s_roll,tick); 		// Obtain control values: torque for pitch and roll
	    SimpleMultiRotorControl(cont_pitch,SMRM_pitch,s_pitch,tick);
}



static void SimpleMultiRotorPosition(SMRM_state *SMRM_roll, SMRM_state *SMRM_pitch, const positionMeasurement_t *pos)
{
	SMRM_roll->position.x 	= pos->y;
	SMRM_pitch->position.x 	=-pos->x;
}

static void SimpleMultiRotorSample(const SMRM_state *SMRM_st, SMRM_sampled *SMRM_samp)
{
	SMRM_samp->s_x  		= SMRM_st->position.x;
	SMRM_samp->s_x_hat  	= SMRM_st->position.x_hat;
	SMRM_samp->s_v_hat  	= SMRM_st->velocity.v_hat;
}

static void SimpleMultiRotorUpdate(SMRM_state *SMRM_st,		const SMRM_sampled *SMRM_samp,
															const float sensorData,
															const uint32_t tick)
{
	if ((tick - lastUpdate) >= 1000/UPDATE_FREQUENCY)
		{
		float gyroscope_meas = sensorData * DEG_TO_RAD; 		// Gyro measurement is in deg/sec, we need rad/sec
		float dt = (float)(tick-lastUpdate)/1000.0;

		DynamicEquationSMRM(SMRM_st, SMRM_samp, gyroscope_meas,dt);
		}

	lastUpdate = tick;

}

static void DynamicEquationSMRM(SMRM_state *SMRM_st, const SMRM_sampled *SMRM_samp, const float sensorData, float dt)
{
	float Y[8]  = {0};
	float dY[8] = {0};

	Y[0] = SMRM_st->position.x;
	Y[1] = SMRM_st->velocity.v;
	Y[2] = SMRM_st->angle.theta;
	Y[3] = SMRM_st->angular_velocity.omega;
	Y[4] = SMRM_st->position.x_hat;
	Y[5] = SMRM_st->velocity.v_hat;
	Y[6] = SMRM_st->angle.theta_hat;
	Y[7] = SMRM_st->angular_velocity.omega_d;

	dY[0] = Y[1];
	dY[1] = GRAVITY*Y[2];
	dY[2] = Y[3];
	dY[3] = (K_P*(-K_X*SMRM_samp->s_x_hat - K_V*SMRM_samp->s_v_hat - Y[6]) - K_D*(sensorData + Y[7]))/(J_ROLLPITCH);
	dY[4] = Y[5] + L_X*(SMRM_samp->s_x - SMRM_samp->s_x_hat);
	dY[5] = GRAVITY*Y[6] + L_V*(SMRM_samp->s_x - SMRM_samp->s_x_hat);
	dY[6] = sensorData + Y[7] + L_TH*(SMRM_samp->s_x - SMRM_samp->s_x_hat);
	dY[7] = L_OM*(SMRM_samp->s_x - SMRM_samp->s_x_hat);

	Y[0] = Y[0] + dY[0]*dt;
	Y[1] = Y[1] + dY[1]*dt;
	Y[2] = Y[2] + dY[2]*dt;
	Y[3] = Y[3] + dY[3]*dt;
	Y[4] = Y[4] + dY[4]*dt;
	Y[5] = Y[5] + dY[5]*dt;
	Y[6] = Y[6] + dY[6]*dt;
	Y[7] = Y[7] + dY[7]*dt;

	SMRM_st->position.x  				= Y[0];
	SMRM_st->velocity.v  				= Y[1];
	SMRM_st->angle.theta  				= Y[2];
	SMRM_st->angular_velocity.omega  	= Y[3];
	SMRM_st->position.x_hat  			= Y[4];
	SMRM_st->velocity.v_hat  			= Y[5];
	SMRM_st->angle.theta_hat  			= Y[6];
	SMRM_st->angular_velocity.omega_d   = Y[7];

}

void SimpleMultiRotorControl(SMRM_control *SMRM_cont, 	const SMRM_state*SMRM_st,
														const SMRM_sampled *SMRM_samp,
														const uint32_t tick)
{
	if (RATE_DO_EXECUTE(CONTROL_RATE, tick))
	SMRM_cont->torque = K_P*(-K_X*SMRM_samp->s_x_hat - K_V*SMRM_samp->s_v_hat - SMRM_st->angle.theta_hat) - K_D*(SMRM_st->angular_velocity.omega + SMRM_st->angular_velocity.omega_d);
}

