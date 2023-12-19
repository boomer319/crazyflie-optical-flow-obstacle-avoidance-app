#include <stdint.h>
#include <float.h>

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

// Declare yaw_command as external variable to pass to collision avoidance
extern int obstacle_trigger;
extern int obstacle_trigger_new_pos;

typedef struct {
    float x;
    float y;
} Vector;

extern Vector calculatedVector;

#endif /* OBSTACLE_AVOIDANCE_H */