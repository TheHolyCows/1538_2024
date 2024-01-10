#pragma once

#include <iostream>
#include "../CowLib/CowMotorController.h"


class Testbench
{
public:

    Testbench (int motorID1, int motorID2);
    void Handle();
    void SetMotor1 (double percent);
    void SetMotor2 (double percent);

private:

    CowLib::CowMotorController *m_Motor1;
    CowLib::CowMotorController *m_Motor2;
    CowMotor::PercentOutput m_Motor1ControlRequest{ 0 };
    CowMotor::PercentOutput m_Motor2ControlRequest{ 0 };

};
