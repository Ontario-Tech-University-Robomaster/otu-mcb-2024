#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <math.h>

#include <cstdint>

#include "tap/communication/serial/remote.hpp"
#include "tap/motor/dji_motor.hpp"

#include "global.hpp"

#define SCALE 0.3

#define MAX_INT16 32676
#define MAX_SPEED ((int)(MAX_INT16 * SCALE))
#define MIN_SPEED -((int)(MAX_INT16 * SCALE))

using namespace tap::communication::serial;

float deg_to_rad(float deg) { return (deg * 3.14159262) / 180; }

int16_t map(int16_t in, int16_t min1, int16_t max1, int16_t min2, int16_t max2)
{
    return (max2 - min2) / (max1 - min1) * (in - min1) + max1;
}

void set_motor_speeds(const Remote &remote)
{
    /*
     * 1  4 +
     * 2  3 Y
     * X  +
     */

    // float chassis_angle =
    // deg_to_rad(tap::motor::DjiMotor::encoderToDegrees(pan_motor.getEncoderUnwrapped()));

    int16_t turret_pan = remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) * MAX_SPEED;
    int16_t turret_tilt = remote.getChannel(Remote::Channel::RIGHT_VERTICAL) * MAX_SPEED;
    int16_t turret_x = remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) * MAX_SPEED;
    int16_t turret_y = -remote.getChannel(Remote::Channel::LEFT_VERTICAL) * MAX_SPEED;

    // int16_t chassis_x = turret_x * std::cos(chassis_angle) - turret_y * std::sin(chassis_angle);
    // int16_t chassis_y = turret_x * std::sin(chassis_angle) + turret_y * std::cos(chassis_angle);
    int16_t chassis_x = turret_x;
    int16_t chassis_y = turret_y;

    // TODO: add rotation factor to make the robot rotate
    int fl = (chassis_y + chassis_x);
    int bl = (chassis_y - chassis_x);
    int fr = -(chassis_y + chassis_x);
    int br = -(chassis_y - chassis_x);

    // TODO: subtract the rotation factor to make the  turret counter rotate
    // pan_motor.setDesiredOutput(turret_pan);

    // tilt_motor.setDesiredOutput(turret_tilt);
    fl_motor.setDesiredOutput(fl);
    bl_motor.setDesiredOutput(bl);
    fr_motor.setDesiredOutput(fr);
    br_motor.setDesiredOutput(br);

    if (remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP)
    {
        left_turret_motor.setDesiredOutput(MAX_INT16 * 0.9);
        right_turret_motor.setDesiredOutput(MAX_INT16 * 0.9);
        agitator_motor.setDesiredOutput(MAX_SPEED);
    }
    else if (remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::MID)
    {
        left_turret_motor.setDesiredOutput(0);
        right_turret_motor.setDesiredOutput(0);
        agitator_motor.setDesiredOutput(0);
    }
}

#endif