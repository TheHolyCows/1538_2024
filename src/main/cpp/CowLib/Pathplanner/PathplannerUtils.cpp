#include "PathplannerUtils.h"



namespace CowLib
{

static wpi::json ParsePathFile(std::string pathName)
{
    const std::string filePath = frc::filesystem::GetDeployDirectory()
			+ "/pathplanner/paths/" + pathName + ".path";
    std::ifstream pathFile(filePath);
    wpi::json data = wpi::json::parse(pathFile);
    pathFile.close();

    return data;
}

static void UpdatePathplannerVelocity(wpi::json *data, units::feet_per_second_t velocity)
{
    (*data)["globalConstraints"]["maxVelocity"] = units::meters_per_second_t(velocity).value();
}

static void UpdatePathplannerAcceleration(wpi::json *data, units::feet_per_second_squared_t acceleration)
{
    (*data)["globalConstraints"]["maxAcceleration"] = units::meters_per_second_squared_t(acceleration).value();
}

};