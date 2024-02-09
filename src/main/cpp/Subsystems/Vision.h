#pragma once

#include <tgmath.h>
#include <units/length.h>
#include <frc/geometry/Translation2d.h>


class Vision
{

    frc::Translation2d BlueSpeaker = { 0_in, 218.42_in };
    frc::Translation2d RedSpeaker = { 651.23_in, 218.42_in };

    static double ComputeAngle(frc::Translation2d currentPosition, frc::Translation2d targetPosition);


};