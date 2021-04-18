#ifndef OWN_SENSOR_H
#define OWN_SENSOR_H

#include "robot_server/pose.h"

typedef struct
{
    double reading;
    Pose pose;
} Sensor;

#endif // OWN_SENSOR_H
