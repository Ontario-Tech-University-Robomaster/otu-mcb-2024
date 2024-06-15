#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "drivers_singleton.hpp"
#include "tap/motor/servo.hpp"

static constexpr tap::motor::MotorId BACK_RIGHT_MOTOR_ID = tap::motor::MOTOR1;
static constexpr tap::motor::MotorId FRONT_RIGHT_MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::motor::MotorId FRONT_LEFT_MOTOR_ID = tap::motor::MOTOR3;
static constexpr tap::motor::MotorId BACK_LEFT_MOTOR_ID = tap::motor::MOTOR4;
// static constexpr tap::motor::MotorId PAN_MOTOR_ID = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId TILT_MOTOR_ID = tap::motor::MOTOR8;

static constexpr tap::motor::MotorId left_turret_MOTOR_ID = tap::motor::MOTOR5;
static constexpr tap::motor::MotorId right_turret_MOTOR_ID = tap::motor::MOTOR6;
static constexpr tap::motor::MotorId agitator_MOTOR_ID = tap::motor::MOTOR7;

// static constexpr tap::gpio::Pwm::Pin right_turret_pwm_pin = tap::gpio::Pwm::Pin::X;
// static constexpr tap::gpio::Pwm::Pin agitator_pwm_pin = tap::gpio::Pwm::Pin::Z;

static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr tap::can::CanBus CAN_BUS2 = tap::can::CanBus::CAN_BUS2;

tap::motor::DjiMotor fl_motor(src::DoNotUse_getDrivers(), FRONT_LEFT_MOTOR_ID, CAN_BUS, false, "front left");
tap::motor::DjiMotor bl_motor(src::DoNotUse_getDrivers(), BACK_LEFT_MOTOR_ID, CAN_BUS, false, "back left");
tap::motor::DjiMotor br_motor(src::DoNotUse_getDrivers(), BACK_RIGHT_MOTOR_ID, CAN_BUS, false, "back right");
tap::motor::DjiMotor fr_motor(src::DoNotUse_getDrivers(), FRONT_RIGHT_MOTOR_ID, CAN_BUS, false, "front right");
// tap::motor::DjiMotor pan_motor(src::DoNotUse_getDrivers(), PAN_MOTOR_ID, CAN_BUS, false, "pan");
// tap::motor::DjiMotor tilt_motor(src::DoNotUse_getDrivers(), TILT_MOTOR_ID, CAN_BUS, false, "tilt");

tap::motor::DjiMotor left_turret_motor(src::DoNotUse_getDrivers(), left_turret_MOTOR_ID, CAN_BUS, false, "left shooter");
tap::motor::DjiMotor right_turret_motor(src::DoNotUse_getDrivers(), right_turret_MOTOR_ID, CAN_BUS, false, "right shooter");
tap::motor::DjiMotor agitator_motor(src::DoNotUse_getDrivers(), agitator_MOTOR_ID, CAN_BUS, false, "agitator");

// tap::motor::Servo right_turret_motor(src::DoNotUse_getDrivers(), right_turret_pwm_pin, 1, 0, 5);

tap::arch::PeriodicMilliTimer sendMotorTimeout(2);

#endif