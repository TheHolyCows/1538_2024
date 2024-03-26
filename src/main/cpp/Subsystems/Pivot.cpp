#include "Pivot.h"

Pivot::Pivot(const int motorId1, const int motorId2, const int encoderId, double encoderOffset)
{
    m_MotorLeft = std::make_unique<CowMotor::TalonFX>(motorId1, "cowdrive");
    m_MotorLeft->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    m_MotorLeft->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);

    m_MotorRight = std::make_unique<CowMotor::TalonFX>(motorId2, "cowdrive");
    m_MotorRight->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    m_MotorRight->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);

    m_Encoder = std::make_unique<CowLib::CowCANCoder>(encoderId, "cowdrive");
    m_Encoder->ConfigAbsoluteOffset(encoderOffset);

    ResetConstants();
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

std::vector<ctre::phoenix6::BaseStatusSignal*> Pivot::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> pivot1Signals = m_MotorLeft->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> pivot2Signals = m_MotorRight->GetSynchronizedSignals();

    signals.insert(signals.end(), pivot1Signals.begin(), pivot1Signals.end());
    signals.insert(signals.end(), pivot2Signals.begin(), pivot2Signals.end());

    return signals;
}

double Pivot::GetAngle()
{
    return GetAbsoluteEncoderPosition() * 360.0;
}

double Pivot::GetAngularVelocity()
{
    return GetAbsoluteEncoderVelocity() * 360.0;
}

double Pivot::GetSetpoint()
{
    return m_TargetPosition * 360.0;
}

void Pivot::SetAngle(double angle)
{
    m_State = State::POSITION;
    m_TargetPosition = std::clamp(angle, CONSTANT("PIVOT_MIN_ANGLE"), CONSTANT("PIVOT_MAX_ANGLE")) / 360.0;
}

void Pivot::BrakeMode(bool brakeMode)
{
    if (brakeMode)
    {
        if (m_PrevBrakeMode)
        {
            m_MotorLeft->ConfigNeutralMode(CowMotor::BRAKE);
            m_MotorRight->ConfigNeutralMode(CowMotor::BRAKE);
            m_PrevBrakeMode = false;
        }
    }
    else
    {
        if (!m_PrevBrakeMode)
        {
            m_MotorLeft->ConfigNeutralMode(CowMotor::COAST);
            m_MotorRight->ConfigNeutralMode(CowMotor::COAST);
            m_PrevBrakeMode = true;
        }
    }
}

bool Pivot::AtTarget()
{
    return std::abs(GetSetpoint() - GetAngle()) < 2.0;

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
                           CONSTANT("PIVOT_V"));

    m_MotorRight->ConfigPID(CONSTANT("PIVOT_P"),
                            CONSTANT("PIVOT_I"),
                            CONSTANT("PIVOT_D"),
                            CONSTANT("PIVOT_S"),
                            CONSTANT("PIVOT_V"));
}

void Pivot::Handle(double elevatorPos)
{
    // Find the upper-edge of the backlash
    m_LeftMotorParameters.offset = std::max(
        m_LeftMotorParameters.offset.value_or(-std::numeric_limits<double>::infinity()),
        -GetLeftMotorPosition() + GetAbsoluteEncoderPosition());

    m_RightMotorParameters.offset = std::max(
        m_RightMotorParameters.offset.value_or(-std::numeric_limits<double>::infinity()),
        -GetLeftMotorPosition() + GetAbsoluteEncoderPosition());

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

        // Adjust motion profile parameters based on which direction the pivot is moving
        if (m_TargetPosition > GetAbsoluteEncoderPosition())
        {
            leftMotorPositionRequest.Velocity = CONSTANT("PIVOT_UP_V");
            leftMotorPositionRequest.Acceleration = CONSTANT("PIVOT_UP_V");
            leftMotorPositionRequest.Jerk = CONSTANT("PIVOT_UP_V");

            leftMotorPositionRequest.Velocity = CONSTANT("PIVOT_UP_V");
            leftMotorPositionRequest.Acceleration = CONSTANT("PIVOT_UP_V");
            leftMotorPositionRequest.Jerk = CONSTANT("PIVOT_UP_V");
        }
        else
        {
            leftMotorPositionRequest.Velocity = CONSTANT("PIVOT_DOWN_V");
            leftMotorPositionRequest.Acceleration = CONSTANT("PIVOT_DOWN_A");
            leftMotorPositionRequest.Jerk = CONSTANT("PIVOT_DOWN_J");

            rightMotorPositionRequest.Velocity = CONSTANT("PIVOT_DOWN_V");
            rightMotorPositionRequest.Acceleration = CONSTANT("PIVOT_DOWN_A");
            rightMotorPositionRequest.Jerk = CONSTANT("PIVOT_DOWN_J");
        }

        // If the pivot is near the setpoint, then use one motor to pull out backlash
        // TODO (dustinlieu): Randomly choose a motor for holding backlash to even out the long-term load on both motors
        if (std::fabs(m_TargetPosition - GetAbsoluteEncoderPosition()) > CONSTANT("PIVOT_HOLD_BACKLASH_POS_THRESHOLD"))
        {
            m_MotorLeft->Set(leftMotorPositionRequest);
            m_MotorRight->Set(rightMotorPositionRequest);
        }
        else
        {
            CowMotor::Control::TorqueCurrent holdBacklashRequest = {};
            holdBacklashRequest.Current = CONSTANT("PIVOT_HOLD_BACKLASH_CURRENT");
            holdBacklashRequest.MaxDutyCycle = 1.0;

            m_MotorLeft->Set(leftMotorPositionRequest);
            m_MotorRight->Set(holdBacklashRequest);
        }
    }
}