#pragma once

#include "../CowLib/CowMotor/TalonFX.h"

#include <iostream>

class Testbench
{
public:

    Testbench (int motorID1, int motorID2);
    void Handle();
    void SetMotor1 (double percent);
    void SetMotor2 (double percent);

private:

    CowMotor::TalonFX *m_Motor1;
    CowMotor::TalonFX *m_Motor2;
    CowMotor::Control::DutyCycle m_Motor1ControlRequest{ 0 };
    CowMotor::Control::DutyCycle m_Motor2ControlRequest{ 0 };

};
