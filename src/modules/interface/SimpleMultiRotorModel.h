#include "stabilizer_types.h"
#include <stdint.h>

/* Initialize SimpleMultiRotor model
 *
 */
void SimpleMultiRotorInit(void);

/* Inject new updated position of CF
 *
 */
void SimpleMultiRotorNewPosition(SMRM_state_t *SMRM_roll, SMRM_state_t *SMRM_pitch,
														positionMeasurement_t *ext_pos,
														SMRM_sampled_t *samp_roll,
														SMRM_sampled_t *samp_pitch);
/* When there is measurement data available,
 * use kalman gains to correct estimates
 */
void SimpleMultiRotorLocalisation_on();

/*When there is no measurement data available,
 * set kalman gains equal to zero
 */
void SimpleMultiRotorLocalisation_off();

/*
 * Run the dynamics of the roll and pitch estimators
 */
void SimpleMultiRotorRunDynamics(SMRM_state_t *SMRM_roll, SMRM_state_t *SMRM_pitch,
														SMRM_sampled_t *s_roll,
														SMRM_sampled_t *s_pitch,
														const float GyroX,
														const float GyroY,
														const uint32_t tick);

/*Compute the rotor torque for roll and pitch of the SMRM model
 *
 */
void SimpleMultiRotorTorque(SMRM_control_t *cont_roll, 	SMRM_control_t *cont_pitch,
														SMRM_state_t *SMRM_roll,
														SMRM_state_t *SMRM_pitch,
														SMRM_sampled_t *s_roll,
														SMRM_sampled_t *s_pitch,
														float gyroRoll,
														float gyroPitch,
														const uint32_t tick);


/*Scale the input torque of roll and pitch in such a way it is feasible for
 * sending it to the powerDistribution block
 */
void SimpleMultiRotorScaleInput(control_t *control_SMRM, SMRM_control_t *cont_roll,SMRM_control_t *cont_pitch);
