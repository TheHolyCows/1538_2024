#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/time.h>
#include <deque>
#include <tuple>

namespace CowLib
{
    class PoseBuffer
    {
    public:
        PoseBuffer(units::second_t historyDuration);
        ~PoseBuffer();

        void Add(units::second_t timestamp, frc::Pose2d pose);
        std::optional<frc::Pose2d> Extrapolate(units::second_t timestamp);

    private:
        std::deque<std::tuple<units::second_t, frc::Pose2d>> m_Buffer;
        units::second_t m_HistoryDuration;
    };
}