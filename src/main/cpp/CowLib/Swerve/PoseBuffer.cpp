#include "PoseBuffer.h"

namespace CowLib
{
    PoseBuffer::PoseBuffer(units::second_t historyDuration)
        : m_HistoryDuration(historyDuration)
    {

    }

    PoseBuffer::~PoseBuffer()
    {

    }

    void PoseBuffer::Add(units::second_t timestamp, frc::Pose2d pose)
    {
        m_Buffer.push_back(std::make_tuple(timestamp, pose));

        while (!m_Buffer.empty())
        {
            units::second_t deltaTime = timestamp - std::get<0>(m_Buffer.front());

            if (deltaTime > m_HistoryDuration)
            {
                m_Buffer.pop_front();
            }
            else
            {
                break;
            }
        }
    }

    std::optional<frc::Pose2d> PoseBuffer::Extrapolate(units::second_t timestamp)
    {
        if (m_Buffer.size() < 2)
        {
            return std::nullopt;
        }

        if (timestamp < std::get<0>(m_Buffer.back()))
        {
            return std::nullopt;
        }

        units::meters_per_second_t vxSum = 0.0_mps;
        units::meters_per_second_t vySum = 0.0_mps;
        units::radians_per_second_t omegaSum = 0.0_rad_per_s;

        for (auto it = m_Buffer.begin(); it < m_Buffer.end() - 1; it++)
        {
            auto [currentTimestamp, currentPose] = it[0];
            auto [nextTimestamp, nextPose] = it[1];

            units::second_t deltaTime = nextTimestamp - currentTimestamp;
            frc::Transform2d transform = nextPose - currentPose;

            vxSum += transform.X() / deltaTime;
            vySum += transform.Y() / deltaTime;
            omegaSum += transform.Rotation().Radians() / deltaTime;
        }

        units::meters_per_second_t vxAvg = vxSum / (m_Buffer.size() - 1);
        units::meters_per_second_t vyAvg = vySum / (m_Buffer.size() - 1);
        units::radians_per_second_t omegaAvg = omegaSum / (m_Buffer.size() - 1);

        auto [lastTimestamp, lastPose] = m_Buffer.back();
        units::second_t extrapolationTime = timestamp - lastTimestamp;

        return frc::Pose2d(vxAvg * extrapolationTime, vyAvg * extrapolationTime, omegaAvg * extrapolationTime);
    }
}