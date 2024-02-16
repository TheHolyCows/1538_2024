#pragma once

#include "../CowLib/Swerve/CowSwerveModulePosition.h"
#include "../CowLib/Swerve/CowSwerveModuleState.h"

#include <ctre/phoenix6/StatusSignal.hpp>

class SwerveModuleInterface
{
protected:
    const int m_Id;
    const double m_EncoderOffset;

    /**
     * @brief Helper function for optimize
     * @param scopeReference Current angle
     * @param newAngle Target angle
     * @return The closed angle within scope
     */
    static double PlaceInAppropriate0To360Scope(double scopeReference, double newAngle);

    /**
     * @brief Modified WPILib optimize function.
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include
     * placing in appropriate scope for CTRE onboard control.
     * Port ofhttps://github.com/frc1678/C2022/blob/main/src/main/java/com/lib/util/CTREModuleState.java
     * @param desiredState The desired module state
     * @param currentAngle The current module angle
     * @return frc::SwerveModuleState
     */
    static CowLib::CowSwerveModuleState Optimize(CowLib::CowSwerveModuleState desiredState, double currentAngle);

public:
    SwerveModuleInterface(const int id, const double encoderOffset);

    virtual ~SwerveModuleInterface() = default;

    virtual std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals() = 0;

    inline int GetID() const { return m_Id; }

    virtual CowLib::CowSwerveModulePosition GetPosition() = 0;

    virtual void SetTargetState(CowLib::CowSwerveModuleState state, bool force = false) = 0;

    virtual void SetBrakeMode(bool brakeMode);

    virtual void ResetConstants() = 0;
    virtual void ResetEncoders()  = 0;

    virtual void Handle() = 0;
};