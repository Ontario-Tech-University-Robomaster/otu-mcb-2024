/*
 * Copyright (c) 2020-2021 OTU-Robomaster
 *
 * This file is part of otu-mcb.
 *
 * otu-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * otu-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with otu-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers.hpp"
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

// Include for global variables
#include "global.hpp"

// board
#include "tap/board/board.hpp"

// include for PID 
#include "tap/algorithms/smooth_pid.hpp"

// include for motor control
#include "motor_control.hpp"


// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);


#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            // take the input from controller, map it to the motors
            drivers->remote.read();

            set_motor_speeds(drivers->remote);
        }

        drivers->remote.read();
        drivers->canRxHandler.pollCanData();
        modm::delay_us(10);
        
    }
    return 0;
}

static void initializeIo(src::Drivers *drivers)
{
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    drivers->refSerial.initialize();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();

    fl_motor.initialize();
    bl_motor.initialize();
    br_motor.initialize();
    fr_motor.initialize();
    // pan_motor.initialize();
    // tilt_motor.initialize();
    agitator_motor.initialize();
    left_turret_motor.initialize();
    right_turret_motor.initialize();

    fl_motor.setDesiredOutput(0);
    bl_motor.setDesiredOutput(0);
    fr_motor.setDesiredOutput(0);
    br_motor.setDesiredOutput(0);
    // pan_motor.setDesiredOutput(0);
    // tilt_motor.setDesiredOutput(0);
    agitator_motor.setDesiredOutput(0);
    left_turret_motor.setDesiredOutput(0);
    right_turret_motor.setDesiredOutput(0);

}

static void updateIo(src::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->mpu6500.read();
    drivers->remote.read();
}
