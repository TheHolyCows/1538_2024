#include "Testbench.h"


Testbench::Testbench(int motorID1, int motorID2) 
{

    m_Motor1  = new CowLib::CowMotorController(motorID1, CowMotor::PHOENIX_V6);
    m_Motor2  = new CowLib::CowMotorController(motorID2, CowMotor::PHOENIX_V6);
    m_Motor1->SetNeutralMode(CowMotor::COAST);
    m_Motor2->SetNeutralMode(CowMotor::COAST);

}

void Testbench::SetMotor1(double percent)
{

    m_Motor1ControlRequest.PercentOut = percent;

}

void Testbench::SetMotor2(double percent)
{

    m_Motor2ControlRequest.PercentOut = percent;

}

void Testbench::Handle()
{

    if(m_Motor1)
    {
        m_Motor1->Set(m_Motor1ControlRequest);
    }

    if(m_Motor2)
    {
        m_Motor2->Set(m_Motor2ControlRequest);
    }



}

