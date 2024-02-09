#include "Vision.h"



double Vision::ComputeAngle(frc::Translation2d curPos, frc::Translation2d targetPos)
{
    double x = units::inch_t(curPos.X()).value() - units::inch_t(targetPos.X()).value();
    double y = units::inch_t(curPos.Y()).value() - units::inch_t(targetPos.Y()).value();
    
    return atan2(y,x);
}