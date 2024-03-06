#pragma once

#include <iostream>
#include <frc/Filesystem.h>
#include <wpi/json.h>
#include <fstream>
#include <units/velocity.h>
#include <units/acceleration.h>



namespace CowLib
{

static wpi::json ParsePathFile(std::string pathname);

static void UpdatePathplannerVelocity(wpi::json *data, units::feet_per_second_t velocity);
static void UpdatePathplannerAcceleration(wpi::json *data, units::feet_per_second_squared_t acceleration);



}