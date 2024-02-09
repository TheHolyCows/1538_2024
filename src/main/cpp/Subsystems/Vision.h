#pragma once

#include <math.h>
#include <tgmath.h>
#include "../CowConstants.h"
#include <units/length.h>
#include <frc/geometry/Pose2d.h>


const frc::Translation2d BLUE_SPEAKER = { 0_in, 218.42_in };
const frc::Translation2d RED_SPEAKER = { 651.23_in, 218.42_in };

class Vision
{
public:
    

    // used to comute yaw when shooting
    static double ComputeRobotRotation(frc::Pose2d currentPosition, frc::Translation2d targetPosition);

    // used to compute pitch of wrist while shooting
    static double ComputeWristAngle(frc::Pose2d currentPosition, frc::Translation2d targetPosition);

};