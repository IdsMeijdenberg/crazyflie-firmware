#include "stabilizer_types.h"
#include <stdint.h>

void YawControllerInit(void);
void YawAltitudeController(setpoint_t *setpoint, positionMeasurement_t *ext_pos, float dt);
void YawAltitudeRunDynamics(altitude_state_t *altitude_state, positionMeasurement_t *ext_pos, setpoint_t *setpoint, uint32_t tick);
