#include "stabilizer_types.h"
#include <stdint.h>

void SimpleMultiRotorInit(void);
void SimpleMultiRotorPosition(SMRM_state *SMRM_roll, SMRM_state *SMRM_pitch, const positionMeasurement_t *pos);
void SimpleMultiRotorUpdate(SMRM_state *SMRM_st,		const SMRM_sampled *SMRM_samp,
														const float sensorData,
														const uint32_t tick);
void SimpleMultiRotorSample(const SMRM_state *SMRM_st, SMRM_sampled *SMRM_samp);
void SimpleMultiRotorControl(SMRM_control *SMRM_cont, 	const SMRM_state*SMRM_st,
														const SMRM_sampled *SMRM_samp,
														const uint32_t tick);

