#ifndef __COW_PIGEON_H__
#define __COW_PIGEON_H__

#include <ctre/phoenix6/Pigeon2.hpp>

class CowPigeon
{
private:
    struct SynchronizedSignals
    {
        ctre::phoenix6::StatusSignal<units::degree_t> *Yaw;
        ctre::phoenix6::StatusSignal<units::degree_t> *Pitch;
        ctre::phoenix6::StatusSignal<units::degree_t> *Roll;

        ctre::phoenix6::StatusSignal<units::degrees_per_second_t> *YawVelocity;
    };

    static CowPigeon *s_Instance;

    CowPigeon();
    ~CowPigeon() = default;

    ctre::phoenix6::hardware::Pigeon2 *m_Pigeon;

    SynchronizedSignals m_SynchronizedSignals;

    bool m_Inverted;

    double m_YawOffset;

public:
    static CowPigeon *GetInstance();

    std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();

    void SetInverted(bool inverted);

    units::degree_t GetYaw();
    units::degree_t GetPitch();
    units::degree_t GetRoll();

    units::degrees_per_second_t GetYawVelocity();

    double GetYawDegrees();
    double GetPitchDegrees();
    double GetRollDegrees();

    double GetYawVelocityDegrees();

    void SetYaw(units::degree_t angle);
    void SetYaw(double angle);
};

#endif /* __COW_PIGEON_H__ */