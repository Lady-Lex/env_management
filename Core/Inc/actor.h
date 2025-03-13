#ifndef ACTOR_H
#define ACTOR_H

#include <stdint.h>

// 控制动作
void process_control_actions(uint8_t control_type);
void log_sensor_data(void);

#endif // ACTOR_H
