#include "Vision.h"



double Vision::ComputeRobotRotation(frc::Pose2d curPos, frc::Translation2d targetPos)
{
    double x = units::inch_t(curPos.X()).value() - units::inch_t(targetPos.X()).value();
    double y = units::inch_t(curPos.Y()).value() - units::inch_t(targetPos.Y()).value();
    
    return atan2(y,x);
}

double Vision::ComputeWristAngle(frc::Pose2d curPos, frc::Translation2d targetPos)
{
    double x = units::inch_t(curPos.X()).value() - units::inch_t(targetPos.X()).value();
    double y = units::inch_t(curPos.Y()).value() - units::inch_t(targetPos.Y()).value();

    x = pow(x,2);
    y = pow(y,2);

    double dist = sqrt(x+y);

    // linear function to compute optimal wrist angle
    // assuming 70 deg at 0 dist and 30 deg at 421 in (opposing alliance line)
    // math would be 70 - 0.095d = angle
    // compute wrist multiplier
    double wristMultiplier = (CONSTANT("WRIST_AT_MAX_SHOOT") - CONSTANT("WRIST_AT_MIN_SHOOT")) / CONSTANT("MAX_SHOOT_DIST");
    return CONSTANT("WRIST_AT_MIN_SHOOT") - wristMultiplier * dist;
}