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
        PoseBuffer(size_t bufferSize);
        ~PoseBuffer();

        std::optional<frc::Pose2d> Extrapolate(units::second_t timestamp);
        void Update(units::second_t timestamp, frc::Pose2d pose);

    private:
        struct Sample
        {
            units::meters_per_second_t vx;
            units::meters_per_second_t vy;
            units::radians_per_second_t omega;
        };

        std::deque<Sample> m_Buffer;
        size_t m_BufferSize;

        units::meters_per_second_t m_SumVX;
        units::meters_per_second_t m_SumVY;
        units::radians_per_second_t m_SumOmega;

        std::optional<std::tuple<units::second_t, frc::Pose2d>> m_PreviousInput;
    };
}