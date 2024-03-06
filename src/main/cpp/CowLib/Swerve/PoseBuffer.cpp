#include "CowLib/Swerve/PoseBuffer.h"

namespace CowLib
{
    PoseBuffer::PoseBuffer(size_t bufferSize)
        : m_BufferSize(bufferSize),
          m_SumVX(0_mps),
          m_SumVY(0_mps),
          m_SumOmega(0_rad_per_s),
          m_PreviousInput(std::nullopt)
    {

    }

    PoseBuffer::~PoseBuffer()
    {

    }

    std::optional<frc::Pose2d> PoseBuffer::Extrapolate(units::second_t timestamp)
    {
        if (m_Buffer.size() == 0)
        {
            return std::nullopt;
        }

        auto [prevTimestamp, prevPose] = m_PreviousInput.value();

        if (timestamp < prevTimestamp)
        {
            return std::nullopt;
        }

        units::second_t extrapolationTime = timestamp - prevTimestamp;
        units::meters_per_second_t vxAvg = m_SumVX / m_Buffer.size();
        units::meters_per_second_t vyAvg = m_SumVY / m_Buffer.size();
        units::radians_per_second_t omegaAvg = m_SumOmega / m_Buffer.size();

        return frc::Pose2d(
            prevPose.X() + (vxAvg * extrapolationTime),
            prevPose.Y() + (vyAvg * extrapolationTime),
            prevPose.Rotation().Radians() + (omegaAvg * extrapolationTime));
    }

    void PoseBuffer::Update(units::second_t timestamp, frc::Pose2d pose)
    {
        if (m_Buffer.size() == m_BufferSize)
        {
            Sample removedSample = m_Buffer.front();

            m_SumVX -= removedSample.vx;
            m_SumVY -= removedSample.vy;
            m_SumOmega -= removedSample.omega;

            m_Buffer.pop_front();
        }

        if (m_PreviousInput.has_value())
        {
            auto [prevTimestamp, prevPose] = m_PreviousInput.value();
            units::second_t deltaTime = timestamp - prevTimestamp;
            units::meter_t deltaX = pose.X() - prevPose.X();
            units::meter_t deltaY = pose.Y() - prevPose.Y();
            units::radian_t deltaTheta = pose.Rotation().Radians() - prevPose.Rotation().Radians();

            Sample sample = {
                .vx = deltaX / deltaTime,
                .vy = deltaY / deltaTime,
                .omega = deltaTheta / deltaTime
            };

            m_Buffer.push_back(sample);

            m_SumVX += sample.vx;
            m_SumVY += sample.vy;
            m_SumOmega += sample.omega;
        }

        m_PreviousInput = std::make_tuple(timestamp, pose);
    }
}