#ifndef __COWLIB_COWCANCODER_H__
#define __COWLIB_COWCANCODER_H__

#include <ctre/phoenix6/CANcoder.hpp>
#include <functional>
#include <units/length.h>

namespace CowLib
{

    class CowCANCoder
    {
    private:
        struct SynchronizedSignals
        {
            ctre::phoenix6::StatusSignal<ctre::phoenix6::signals::MagnetHealthValue> *MagnetHealth;
            ctre::phoenix6::StatusSignal<units::turn_t> *Position;
            ctre::phoenix6::StatusSignal<units::turn_t> *AbsolutePosition;
            ctre::phoenix6::StatusSignal<units::turns_per_second_t> *Velocity;
        };

        ctre::phoenix6::hardware::CANcoder *m_CANCoder;
        ctre::phoenix6::configs::CANcoderConfiguration m_Config;

        SynchronizedSignals m_SynchronizedSignals;

        void ApplyConfig();

    public:
        CowCANCoder(int device, std::string canbus);

        std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();

        void ConfigAbsoluteOffset(double offset);

        void SetSigned(bool isSigned);
        void SetInverted(bool isInverted);
        void SetInitToAbsolute(bool isAbsolute);

        bool GetMagnetIsHealthy();
        double GetPosition();
        double GetAbsolutePosition();
        double GetVelocity();

        ctre::phoenix6::hardware::CANcoder *GetInternalCANCoder();

        ~CowCANCoder();
    };

} // namespace CowLib

#endif /* __COWLIB_COWCANCODER_H__ */