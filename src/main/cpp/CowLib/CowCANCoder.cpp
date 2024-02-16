#include "CowCANCoder.h"

namespace CowLib
{

    /// @brief Creates a new CowCANCoder. Defaults to unsigned absolute
    /// @param deviceId
    CowCANCoder::CowCANCoder(int deviceId, std::string canbus)
    {
        m_CANCoder = new ctre::phoenix6::hardware::CANcoder(deviceId, canbus);

        // Default config
        m_Config = ctre::phoenix6::configs::CANcoderConfiguration{};
        ApplyConfig();

        m_SynchronizedSignals.Position = &m_CANCoder->GetPosition();
        m_SynchronizedSignals.AbsolutePosition = &m_CANCoder->GetPosition();

        m_UnsynchronizedSignals.Velocity = &m_CANCoder->GetVelocity();

        // This is what 1678 uses for swerve so okay
        // TODO: check if this is correct
        // m_Cancoder->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 255);
        // m_Cancoder->SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, 255);
    }

    std::vector<ctre::phoenix6::BaseStatusSignal*> CowCANCoder::GetSynchronizedSignals()
    {
        std::vector<ctre::phoenix6::BaseStatusSignal*> signals = {
            m_SynchronizedSignals.Position,
            m_SynchronizedSignals.AbsolutePosition
        };

        return signals;
    }

    void CowCANCoder::ApplyConfig()
    {
        m_CANCoder->GetConfigurator().Apply(m_Config);
    }

    void CowCANCoder::ConfigAbsoluteOffset(double offset)
    {
        m_Config.MagnetSensor.MagnetOffset = offset;
        ApplyConfig();
    }

    /// @brief Sets whether the sensor direction is inverted
    /// @param isInverted
    void CowCANCoder::SetInverted(bool isInverted)
    {
        m_Config.MagnetSensor.SensorDirection
            = isInverted ? ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive
                         : ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;

        ApplyConfig();
    }

    /// @brief Sets whether the output is signed [-180, 180] or unsigned [0, 360]
    /// @param isSigned
    void CowCANCoder::SetSigned(bool isSigned)
    {
        m_Config.MagnetSensor.AbsoluteSensorRange
            = isSigned ? ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf
                       : ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1;

        ApplyConfig();
    }

    double CowCANCoder::GetPosition()
    {
        return m_SynchronizedSignals.Position->GetValue().value();
    }

    /// @brief Gets the absolute position in degrees
    /// @return rotation
    double CowCANCoder::GetAbsolutePosition()
    {
        return m_SynchronizedSignals.AbsolutePosition->GetValue().value();
    }

    double CowCANCoder::GetVelocity()
    {
        return m_UnsynchronizedSignals.Velocity->GetValue().value();
    }

    /// @brief Gets internal CANCoder
    /// @return Pointer to internal CANCoder
    ctre::phoenix6::hardware::CANcoder *CowCANCoder::GetInternalCANCoder()
    {
        return m_CANCoder;
    }

    CowCANCoder::~CowCANCoder()
    {
        delete m_CANCoder;
    }

} // namespace CowLib