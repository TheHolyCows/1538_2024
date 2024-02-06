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

    // Don't rotate for low speeds - unless we are e-braking
    // may want to lower this value when in teleop
    // Dustin: commented this out for merge
//     double targetAngle;

//     if (!force && fabs(optimized.velocity) <= CONSTANT("SWERVE_MIN_ROT_SPEED"))
//     {
//         targetAngle = m_PreviousAngle;
//     }
//     else
//     {
//         targetAngle = optimized.angle;
//     }

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "omtimized vel %f\n", optimized.velocity);
    // printf("optimized vel: %f\n", optimized.velocity);

    // m_PreviousAngle = targetAngle;

    // m_RotationControlRequest.Position = targetAngle / 360.0;
    // m_RotationControlRequest.FeedForward
    // = state.omega * 12 * 0.3 / (360 * (6380 / CONSTANT("SWERVE_ROTATION_GEAR_RATIO")) / 60);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/feedforward",
    //                                m_RotationControlRequest.FeedForward);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/omega", state.omega);
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "module ");

    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/target velocity", optimized.velocity);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/target angle", optimized.angle);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/angle error", m_Angle - optimized.angle);
}

void SwerveModule::ResetConstants()
{
    m_RotationMotor->ConfigPID(CONSTANT("SWERVE_ANGLE_P"), CONSTANT("SWERVE_ANGLE_I") * CONSTANT("SWERVE_ROTATION_GEAR_RATIO"), CONSTANT("SWERVE_ANGLE_D") * CONSTANT("SWERVE_ROTATION_GEAR_RATIO"));

    // Percent output so not used
    // m_DriveMotor->SetPID(CONSTANT("SWERVE_DRIVE_P"),
    //                      CONSTANT("SWERVE_DRIVE_I"),
    //                      CONSTANT("SWERVE_DRIVE_D"),
    //                      CONSTANT("SWERVE_DRIVE_F"));
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

    // printf("Module %d abs enc angle: %f\n", m_Id, m_Encoder->GetAbsolutePosition());

    // Update current positions once per loop
    m_Velocity = CowLib::Conversions::FalconToFPS(m_DriveMotor->GetVelocity(),
                                                  CONSTANT("WHEEL_CIRCUMFERENCE"),
                                                  CONSTANT("SWERVE_DRIVE_GEAR_RATIO"));

    double drive_rotation_offset = -(m_RotationMotor->GetPosition() - m_InitialRotation) * (50 / 14);

    // I think this is right...
    m_Position = ((m_DriveMotor->GetPosition() + drive_rotation_offset) / CONSTANT("SWERVE_DRIVE_GEAR_RATIO"))
                  * CONSTANT("WHEEL_CIRCUMFERENCE");

    m_Angle = m_RotationMotor->GetPosition();
    m_AngularVelocity = m_RotationMotor->GetVelocity();

    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/position", m_Position);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/angle", m_Angle);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/velocity", m_Velocity);
    // frc::SmartDashboard::PutNumber("swerve/module" + std::to_string(m_Id) + "/angular velocity", m_AngularVelocity);

    m_PrevTargetState = optimized;
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
