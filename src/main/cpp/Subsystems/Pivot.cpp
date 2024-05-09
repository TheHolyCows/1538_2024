#include "Pivot.h"

Pivot::Pivot(const int motorLeftID, const int motorRightID, const int encoderId, double encoderOffset)
{
    m_MotorLeft = std::make_unique<CowMotor::TalonFX>(motorLeftID, "cowdrive");
    m_MotorLeft->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);

    m_MotorRight = std::make_unique<CowMotor::TalonFX>(motorRightID, "cowdrive");
    m_MotorRight->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);

    m_Encoder = std::make_unique<CowLib::CowCANCoder>(encoderId, "cowdrive");
    m_Encoder->ConfigAbsoluteOffset(encoderOffset);

    ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    ResetConstants();
}

std::vector<ctre::phoenix6::BaseStatusSignal*> Pivot::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> motorLeftSignals = m_MotorLeft->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> motorRightSignals = m_MotorRight->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> encoderSignals = m_Encoder->GetSynchronizedSignals();

    signals.insert(signals.end(), motorLeftSignals.begin(), motorLeftSignals.end());
    signals.insert(signals.end(), motorRightSignals.begin(), motorRightSignals.end());
    signals.insert(signals.end(), encoderSignals.begin(), encoderSignals.end());

    return signals;
}

void Pivot::ConfigNeutralMode(CowMotor::NeutralMode neutralMode)
{
    m_MotorLeft->ConfigNeutralMode(neutralMode);
    m_MotorRight->ConfigNeutralMode(neutralMode);
}

double Pivot::GetLeftMotorPosition()
{
    return m_MotorLeft->GetPosition() / CONSTANT("PIVOT_GEAR_RATIO");
}

double Pivot::GetRightMotorPosition()
{
    return m_MotorRight->GetPosition() / CONSTANT("PIVOT_GEAR_RATIO");
}

double Pivot::GetAbsoluteEncoderPosition()
{
    return m_Encoder->GetAbsolutePosition();
}

double Pivot::GetAbsoluteEncoderVelocity()
{
    return m_Encoder->GetVelocity();
}

double Pivot::GetAngle()
{
    return GetAbsoluteEncoderPosition() * 360.0;
}

double Pivot::GetAngularVelocity()
{
    return GetAbsoluteEncoderVelocity() * 360.0;
}

double Pivot::GetTargetAngle()
{
    return m_TargetPosition * 360.0;
}

void Pivot::SetTargetAngle(double angle)
{
    m_State = State::POSITION;
    m_TargetPosition = std::clamp(angle, CONSTANT("PIVOT_MIN_ANGLE"), CONSTANT("PIVOT_MAX_ANGLE")) / 360.0;
}

bool Pivot::IsOnTarget()
{
    return std::abs(GetTargetAngle() - GetAngle()) < 2.0;

    // percent difference doesnt work well at higher values
    // double delta = std::abs(setpoint - angle);
    // double avg = (setpoint + angle) / 2.0;

    // double pctDiff = (delta / avg);

    // if (pctDiff < 0.03)
    // {
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }
}

void Pivot::ResetConstants()
{
    m_MotorLeft->ConfigPID(CONSTANT("PIVOT_P"),
                           CONSTANT("PIVOT_I"),
                           CONSTANT("PIVOT_D"),
                           CONSTANT("PIVOT_S"),
                           CONSTANT("PIVOT_V"),
                           CONSTANT("PIVOT_A"));

    m_MotorRight->ConfigPID(CONSTANT("PIVOT_P"),
                            CONSTANT("PIVOT_I"),
                            CONSTANT("PIVOT_D"),
                            CONSTANT("PIVOT_S"),
                            CONSTANT("PIVOT_V"),
                            CONSTANT("PIVOT_A"));
}

void Pivot::Handle(double elevatorPos)
{
    // Find the upper-edge of the backlash
    if (!m_LeftMotorParameters.offset.has_value() || !m_RightMotorParameters.offset.has_value())
    {
        m_LeftMotorParameters.offset = GetLeftMotorPosition() - GetAbsoluteEncoderPosition();
        m_RightMotorParameters.offset = GetRightMotorPosition() - GetAbsoluteEncoderPosition();
    }

    // TODO (dustinlieu): Do we really need to wait for it to stop moving?
    if (fabs(GetAbsoluteEncoderVelocity()) == 0.0 &&
        fabs(m_MotorLeft->GetVelocity()) == 0.0 &&
        fabs(m_MotorRight->GetVelocity()) == 0.0 &&
        m_MotorLeft->GetCurrent() > 20 &&
        m_MotorRight->GetCurrent() > 20)
    {
        m_LeftMotorParameters.offset = std::max(
            m_LeftMotorParameters.offset.value(),
            GetLeftMotorPosition() - GetAbsoluteEncoderPosition());

        m_RightMotorParameters.offset = std::max(
            m_RightMotorParameters.offset.value(),
            GetRightMotorPosition() - GetAbsoluteEncoderPosition());
    }

    if (m_State == State::IDLE)
    {
        CowMotor::Control::TorqueCurrent motorRequest = {};
        m_MotorLeft->Set(motorRequest);
        m_MotorRight->Set(motorRequest);
    }
    else if (m_State == State::POSITION)
    {
        CowMotor::Control::DynamicMotionMagicTorqueCurrent leftMotorPositionRequest = {};
        leftMotorPositionRequest.Position = (m_TargetPosition + m_LeftMotorParameters.offset.value()) * CONSTANT("PIVOT_GEAR_RATIO");

        CowMotor::Control::DynamicMotionMagicTorqueCurrent rightMotorPositionRequest = {};
        rightMotorPositionRequest.Position = (m_TargetPosition + m_RightMotorParameters.offset.value()) * CONSTANT("PIVOT_GEAR_RATIO");

        leftMotorPositionRequest.Velocity = CONSTANT("PIVOT_UP_V");
        leftMotorPositionRequest.Acceleration = CONSTANT("PIVOT_UP_A");
        leftMotorPositionRequest.Jerk = CONSTANT("PIVOT_UP_J");

        rightMotorPositionRequest.Velocity = CONSTANT("PIVOT_UP_V");
        rightMotorPositionRequest.Acceleration = CONSTANT("PIVOT_UP_A");
        rightMotorPositionRequest.Jerk = CONSTANT("PIVOT_UP_J");

        m_MotorLeft->Set(leftMotorPositionRequest);
        m_MotorRight->Set(rightMotorPositionRequest);
    }
}