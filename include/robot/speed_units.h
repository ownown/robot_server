/**
 * @file speed_units.h
 * @author Oliver Newman (oliver@olivernewman.co.uk)
 * @brief enum for defining the units to work with speed in
 * 
 * The BrickPi works in degrees per second. We may also want to work in cm/s,
 * mm/s or rads/s.
 *
 * @version 0.1
 * @date 2021-04-17
 * 
 * (c) Copyright 2021 Oliver Newman
 * 
 */
#ifndef OWN_SPEED_UNITS_H
#define OWN_SPEED_UNITS_H

/**
 * @brief enum for defining the units to work with speed in
 * 
 * The BrickPi works in degrees per second. We may also want to work in cm/s,
 * mm/s or rads/s.
 * 
 */
enum class SpeedUnit
{
    CM,
    MM,
    DEG,
    RAD
};

#endif // OWN_SPEED_UNITS_H
