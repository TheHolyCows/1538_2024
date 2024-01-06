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
        ctre::phoenix6::hardware::CANcoder *m_Cancoder;

        // struct Signals
        // {
        std::function<units::turn_t()> m_PositionSupplier;
        std::function<units::turn_t()> m_AbsolutePositionSupplier;
        std::function<units::turns_per_second_t()> m_VelocitySupplier;
        // ctre::phoenix6::StatusSignalValue<units::turn_t>;
        // ctre::phoenix6::StatusSignalValue<units::turn_t>;
        // ctre::phoenix6::StatusSignalValue<units::turns_per_second_t>;

        // Signals m_Signals;

        ctre::phoenix6::configs::CANcoderConfiguration m_Config;

        void ApplyConfig();

    public:
        CowCANCoder(int device);

        void SetSigned(bool isSigned);
        void SetInverted(bool isInverted);
        void SetInitToAbsolute(bool isAbsolute);

        double GetPosition();
        double GetAbsolutePosition();
        double GetVelocity();

        ctre::phoenix6::hardware::CANcoder *GetInternalCANCoder();

        ~CowCANCoder();
    };

} // namespace CowLib

#endif /* __COWLIB_COWCANCODER_H__ */