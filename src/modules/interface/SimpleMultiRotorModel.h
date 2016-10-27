#include "stabilizer_types.h"
#include <stdint.h>

void SimpleMultiRotorInit(void);

void SimpleMultiRotorNewPosition(SMRM_state *SMRM_roll, SMRM_state *SMRM_pitch,
														positionMeasurement_t *ext_pos,
														SMRM_sampled *samp_roll,
														SMRM_sampled *samp_pitch);

void SimpleMultiRotorRunDynamics(SMRM_state *SMRM_roll, SMRM_state *SMRM_pitch,
														SMRM_sampled *s_roll,
														SMRM_sampled *s_pitch,
														const float gyroX,
														const float gyroY,
														const uint32_t tick);

void SimpleMultiRotorTorque(SMRM_control *cont_roll, 	SMRM_control *cont_pitch,
														SMRM_state *SMRM_roll,
														SMRM_state *SMRM_pitch,
														SMRM_sampled *s_roll,
														SMRM_sampled *s_pitch,
														const uint32_t tick);





