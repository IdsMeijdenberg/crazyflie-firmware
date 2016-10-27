#include "stabilizer_types.h"
#include <stdint.h>

void YawControllerInit(void);
void YawAltitudeController(setpoint_t *setpoint, positionMeasurement_t *ext_pos, float dt);
