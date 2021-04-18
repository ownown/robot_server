#ifndef OWN_MOTORS_H
#define OWN_MOTORS_H

enum class MotorSide
{
    Left,
    Right
};

/**
 * @brief struct to store the motor speeds in in DPS
 * 
 */
typedef struct
{
    double left;
    double right;
} Motors;

#endif // OWN_MOTORS_H
