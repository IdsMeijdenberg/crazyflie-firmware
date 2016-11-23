/*
 * Constants used
 */
#include "SimpleMultiRotorModel.h"

// MACROS

// Optimal configuration
#define K_X  (0.9935f)
#define K_V  (0.4609f)
#define K_P  (0.0158f)
#define K_D  (0.0018f)
#define L_X  (3.2048f)
#define L_V  (4.7815f)
#define L_TH (0.3944f)
#define L_OM (0.0852f)

// non-optimal configuration (= unstable!)
//#define K_X  (0.6502f)
//#define K_V  (0.3698f)
//#define K_P  (0.0024f)
//#define K_D  (3.7470E-4f)
//#define L_X  (9.3607f)
//#define L_V  (35.7403f)
//#define L_TH (7.2739f)
//#define L_OM (1.9646f)

#define GRAVITY (9.91f)
#define J_ROLLPITCH (2.3951e-5f)
#define PI	3.14159265358979f
#define DEG_TO_RAD (PI/180.0f)
#define UPDATE_FREQUENCY 200
#define CONTROL_RATE 200
#define Ct (3.1582E-10f)
#define OMEGAE (13736.0f)
#define sqrt_2 1.41421356f
#define d_arm 39.73E-3f


//State variables and normal variables for SimpleMultiRotorModel
static KALMAN_gain_t kalman_gain;
static bool isInit = false;

//Private functions
static void DynamicEquationSMRM(SMRM_state_t *SMRM_st, 	const SMRM_sampled_t *SMRM_samp,
														const float gyroMeasurement,
														float dt);

static void SimpleMultiRotorPosition(SMRM_state_t *SMRM_roll, SMRM_state_t *SMRM_pitch, const positionMeasurement_t *pos);
static void SimpleMultiRotorSample(const SMRM_state_t *SMRM_st, SMRM_sampled_t *SMRM_samp);
void SimpleMultiRotorUpdate(SMRM_state_t *SMRM_st,	const SMRM_sampled_t *SMRM_samp,
		const float sensorData,
		const float dt);
static void SimpleMultiRotorControl(SMRM_control_t *SMRM_cont, 	const SMRM_state_t*SMRM_st,
		const SMRM_sampled_t *SMRM_samp,
		const float gyroMeasurement);

static inline int16_t saturateSignedInt16_SMRM(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > 32767)
    return 32767;
  else if (in < -32767)
    return -32767;
  else
    return (int16_t)in;
}
//Public functions
void SimpleMultiRotorInit(void)
{
	if(isInit)
		return;

	isInit = true;
	return;
}

void SimpleMultiRotorNewPosition(SMRM_state_t *SMRM_roll, SMRM_state_t *SMRM_pitch,
		positionMeasurement_t *ext_pos,
		SMRM_sampled_t *s_roll,
		SMRM_sampled_t *s_pitch)
{
	SimpleMultiRotorPosition(SMRM_roll, SMRM_pitch, ext_pos); 	// Update linear position of roll and pitch by the measurement of position from GoT
	SimpleMultiRotorSample(SMRM_roll, s_roll); 					// Update sampled signals x(n), x_hat(n), and v_hat(n) when there is a new measurement
	SimpleMultiRotorSample(SMRM_pitch, s_pitch);
}

void SimpleMultiRotorRunDynamics(SMRM_state_t *SMRM_roll, 	SMRM_state_t *SMRM_pitch,
															SMRM_sampled_t *s_roll,
															SMRM_sampled_t *s_pitch,
															const float GyroX,
															const float GyroY,
															const uint32_t tick){
	if (RATE_DO_EXECUTE(UPDATE_FREQUENCY, tick))
	{

		SimpleMultiRotorUpdate(SMRM_roll, s_roll, GyroX, 1.0/UPDATE_FREQUENCY);
		SimpleMultiRotorUpdate(SMRM_pitch, s_pitch, GyroY, 1.0/UPDATE_FREQUENCY);

	}
}

void SimpleMultiRotorLocalisation_on(){

	kalman_gain.Lx = L_X;
	kalman_gain.Lv = L_V;
	kalman_gain.Lth = L_TH;
	kalman_gain.Lom = L_OM;
}

void SimpleMultiRotorLocalisation_off(){

	kalman_gain.Lx = 0;
	kalman_gain.Lv = 0;
	kalman_gain.Lth = 0;
	kalman_gain.Lom = 0;
}

void SimpleMultiRotorScaleInput(control_t *control_SMRM, SMRM_control_t *cont_roll,SMRM_control_t *cont_pitch)
{
	control_SMRM->roll = saturateSignedInt16_SMRM((1.0/(2.0*OMEGAE)*sqrt_2/(4.0*d_arm*Ct)*cont_roll->torque)/0.8);  // From RPM to PWM
	control_SMRM->pitch= saturateSignedInt16_SMRM((1.0/(2.0*OMEGAE)*sqrt_2/(4.0*d_arm*Ct)*cont_pitch->torque)/0.8);
}


static void SimpleMultiRotorPosition(SMRM_state_t *SMRM_roll, SMRM_state_t *SMRM_pitch, const positionMeasurement_t *pos)
{
	SMRM_roll->position.x 	= pos->y;
	SMRM_pitch->position.x 	=-pos->x;
}

static void SimpleMultiRotorSample(const SMRM_state_t *SMRM_st, SMRM_sampled_t *SMRM_samp)
{
	SMRM_samp->s_x  		= SMRM_st->position.x;
	SMRM_samp->s_x_hat  	= SMRM_st->position.x_hat;
	SMRM_samp->s_v_hat  	= SMRM_st->velocity.v_hat;
}

void SimpleMultiRotorUpdate(SMRM_state_t *SMRM_st,	const SMRM_sampled_t *SMRM_samp,
													const float sensorData,
													const float dt)
{
		float gyroscope_meas = sensorData * DEG_TO_RAD; 		// Gyro measurement is in deg/sec, we need rad/sec
		DynamicEquationSMRM(SMRM_st, SMRM_samp, gyroscope_meas, dt);
}

static void DynamicEquationSMRM(SMRM_state_t *SMRM_st, 	const SMRM_sampled_t *SMRM_samp,
														const float gyroMeasurement,
														float dt)
{
	float Y[4]  = {0};
	float dY[4] = {0};

	Y[0] = SMRM_st->position.x_hat;
	Y[1] = SMRM_st->velocity.v_hat;
	Y[2] = SMRM_st->angle.theta_hat;
	Y[3] = SMRM_st->angular_velocity.omega_d;

	dY[0] = Y[1] + kalman_gain.Lx*(SMRM_samp->s_x - SMRM_samp->s_x_hat);
	dY[1] = GRAVITY*Y[2] + kalman_gain.Lv*(SMRM_samp->s_x - SMRM_samp->s_x_hat);
	dY[2] = gyroMeasurement + Y[3] + kalman_gain.Lth*(SMRM_samp->s_x - SMRM_samp->s_x_hat);
	dY[3] = kalman_gain.Lom*(SMRM_samp->s_x - SMRM_samp->s_x_hat);

	Y[0] = Y[0] + dY[0]*dt; 			// First order Euler approximation
	Y[1] = Y[1] + dY[1]*dt;
	Y[2] = Y[2] + dY[2]*dt;
	Y[3] = Y[3] + dY[3]*dt;

	SMRM_st->position.x_hat  			= Y[0];
	SMRM_st->velocity.v_hat  			= Y[1];
	SMRM_st->angle.theta_hat  			= Y[2];
	SMRM_st->angular_velocity.omega_d   = Y[3];

}

void SimpleMultiRotorTorque(SMRM_control_t *cont_roll, 	SMRM_control_t *cont_pitch,
		SMRM_state_t *SMRM_roll,
		SMRM_state_t *SMRM_pitch,
		SMRM_sampled_t *s_roll,
		SMRM_sampled_t *s_pitch,
		float gyroRoll,
		float gyroPitch,
		const uint32_t tick)
{
	if (RATE_DO_EXECUTE(CONTROL_RATE, tick)){
		float gyroRollRad = gyroRoll * DEG_TO_RAD; 		// Gyro measurement is in deg/sec, we need rad/sec
		float gyroPitchRad = gyroPitch * DEG_TO_RAD;
		SimpleMultiRotorControl(cont_roll,SMRM_roll,s_roll,gyroRollRad); 		// Obtain control values: torque for pitch and roll
		SimpleMultiRotorControl(cont_pitch,SMRM_pitch,s_pitch,gyroPitchRad);
	}
}

static void SimpleMultiRotorControl(SMRM_control_t *SMRM_cont,	const SMRM_state_t*SMRM_st,
																const SMRM_sampled_t *SMRM_samp,
																const float gyroMeasurement)
{
		SMRM_cont->torque = K_P*(-K_X*SMRM_samp->s_x_hat - K_V*SMRM_samp->s_v_hat - SMRM_st->angle.theta_hat) - K_D*(gyroMeasurement + SMRM_st->angular_velocity.omega_d);
}

