#include "SwerveModule.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/kinematics/SwerveModuleState.h>

SwerveModule::SwerveModule(const int id,
                           const int driveMotor,
                           const int rotationMotor,
                           const int encoderId,
                           const double encoderOffset)
    : SwerveModuleInterface(id, encoderOffset)
{
    m_DriveMotor    = std::make_unique<CowMotor::TalonFX>(driveMotor, "cowdrive");
    m_RotationMotor = std::make_unique<CowMotor::TalonFX>(rotationMotor, "cowdrive");
    m_Encoder       = std::make_unique<CowLib::CowCANCoder>(encoderId, "cowdrive");

    m_Encoder->ConfigAbsoluteOffset(-encoderOffset / 360.0);

    m_RotationMotor->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    m_RotationMotor->FuseCANCoder(encoderId, CONSTANT("SWERVE_ROTATION_GEAR_RATIO"));
    m_RotationMotor->ConfigContinuousWrap(true);

    m_DriveControlRequest.EnableFOC = true;
    m_DriveControlRequest.DutyCycle = 0;

    m_RotationControlRequest.EnableFOC = true;
    m_RotationControlRequest.Position = 0;
    m_RotationControlRequest.FeedForward = 0;

    m_PreviousAngle = 0;

    m_BrakeMode = true;

    ResetConstants();
    ResetEncoders();

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "Module %d abs encoder angle: %f  motor angle %f\n",
    //                           id,
    //                           m_Encoder->GetAbsolutePosition(),
    //                           m_RotationMotor->GetPosition());
    // frc::SmartDashboard::PutNumber("swerve/module " + std::to_string(m_Id) + "/absolute encoder angle",
    // m_Encoder->GetAbsolutePosition());
}

std::vector<ctre::phoenix6::BaseStatusSignal*> SwerveModule::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> driveMotorSignals = m_DriveMotor->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> rotationMotorSignals = m_RotationMotor->GetSynchronizedSignals();

    signals.insert(signals.end(), driveMotorSignals.begin(), driveMotorSignals.end());
    signals.insert(signals.end(), rotationMotorSignals.begin(), rotationMotorSignals.end());

    return signals;
}

CowLib::CowSwerveModulePosition SwerveModule::GetPosition()
{
    double drive_rotation_offset = -(m_RotationMotor->GetPosition() - m_InitialRotation) * (50 / 14);

    return CowLib::CowSwerveModulePosition{
        .distance = ((m_DriveMotor->GetPosition() + drive_rotation_offset) / CONSTANT("SWERVE_DRIVE_GEAR_RATIO")) * CONSTANT("WHEEL_CIRCUMFERENCE"),
        .angle = m_RotationMotor->GetPosition() * 360.0
    };
}

void SwerveModule::SetTargetState(CowLib::CowSwerveModuleState state, bool force)
{
    m_TargetState = state;

    // Do not rotate if velocity is below threshold, unless e-braking
    if (!force && fabs(m_TargetState.velocity) <= CONSTANT("SWERVE_MIN_ROT_SPEED"))
    {
        m_TargetState.angle = m_PrevTargetState.angle;
    }

    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/feedforward",
    //                                m_RotationControlRequest.FeedForward);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/omega", state.omega);
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "module ");

    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/target velocity", optimized.velocity);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/target angle", optimized.angle);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/angle error", m_Angle - optimized.angle);
}

void SwerveModule::SetBrakeMode(bool brakeMode)
{
    if (brakeMode == m_BrakeMode)
    {
        return;
    }

    m_BrakeMode = brakeMode;

    if (m_BrakeMode)
    {
        m_DriveMotor->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    }
    else
    {
        m_DriveMotor->ConfigNeutralMode(CowMotor::NeutralMode::COAST);
    }
}

void SwerveModule::ResetConstants()
{
    m_DriveMotor->ConfigPID(CONSTANT("SWERVE_DRIVE_P"), CONSTANT("SWERVE_DRIVE_I"), CONSTANT("SWERVE_DRIVE_D"));
    m_RotationMotor->ConfigPID(CONSTANT("SWERVE_ANGLE_P"), CONSTANT("SWERVE_ANGLE_I"), CONSTANT("SWERVE_ANGLE_D"));
}

void SwerveModule::ResetEncoders()
{
    ctre::phoenix6::BaseStatusSignal::WaitForAll(0_ms, m_RotationMotor->GetSynchronizedSignals());
    m_InitialRotation = m_RotationMotor->GetPosition();

    int errCode;
    do
    {
        errCode = std::get<ctre::phoenix::StatusCode>(m_DriveMotor->SetEncoderPosition(0));
        if (errCode != 0)
        {
            CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR, "err code %d", errCode);
        }
    } while (errCode != 0);
}

void SwerveModule::Handle()
{
    CowLib::CowSwerveModuleState optimized = Optimize(m_TargetState, m_PrevTargetState.angle);

    m_DriveControlRequest.DutyCycle = optimized.velocity / CONSTANT("SWERVE_MAX_SPEED");
    m_RotationControlRequest.Position = optimized.angle / 360.0;

    m_DriveMotor->Set(m_DriveControlRequest);
    m_RotationMotor->Set(m_RotationControlRequest);

    m_PrevTargetState = optimized;
}