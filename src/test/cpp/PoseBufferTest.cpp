#include "CowLib/Swerve/PoseBuffer.h"

#include <gtest/gtest.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

class PoseBufferTest : public testing::Test
{
public:
    PoseBufferTest() : m_PoseBuffer(10)
    {
        
    }
protected:
    CowLib::PoseBuffer m_PoseBuffer;
};

TEST_F(PoseBufferTest, Simple1)
{
    frc::Pose2d pose;

    pose = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(0_s, pose);

    pose = frc::Pose2d(1_m, 1_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(1_s, pose);

    pose = frc::Pose2d(2_m, 2_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(2_s, pose);

    frc::Pose2d extrapolated = m_PoseBuffer.Extrapolate(3_s).value();

    EXPECT_EQ(extrapolated.X(), 3.0_m);
    EXPECT_EQ(extrapolated.Y(), 3.0_m);
}

TEST_F(PoseBufferTest, Simple2)
{
    frc::Pose2d pose;

    pose = frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(0_s, pose);

    pose = frc::Pose2d(0.5_m, 0.5_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(1_s, pose);

    pose = frc::Pose2d(1.0_m, 1.0_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(2_s, pose);

    frc::Pose2d extrapolated = m_PoseBuffer.Extrapolate(3_s).value();

    EXPECT_EQ(extrapolated.X(), 1.5_m);
    EXPECT_EQ(extrapolated.Y(), 1.5_m);
}

TEST_F(PoseBufferTest, Simple3)
{
    frc::Pose2d pose;

    pose = frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(0_s, pose);

    pose = frc::Pose2d(-0.5_m, -0.5_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(1_s, pose);

    pose = frc::Pose2d(-1.0_m, -1.0_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(2_s, pose);

    frc::Pose2d extrapolated = m_PoseBuffer.Extrapolate(3_s).value();

    EXPECT_EQ(extrapolated.X(), -1.5_m);
    EXPECT_EQ(extrapolated.Y(), -1.5_m);
}

TEST_F(PoseBufferTest, Simple4)
{
    frc::Pose2d pose;

    pose = frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(0.0_s, pose);

    pose = frc::Pose2d(-0.5_m, -0.5_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(0.5_s, pose);

    pose = frc::Pose2d(-1.0_m, -1.0_m, frc::Rotation2d(0_rad));
    m_PoseBuffer.Update(1.0_s, pose);

    frc::Pose2d extrapolated = m_PoseBuffer.Extrapolate(3_s).value();

    EXPECT_EQ(extrapolated.X(), -3.0_m);
    EXPECT_EQ(extrapolated.Y(), -3.0_m);
}