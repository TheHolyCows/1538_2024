#include "Testbench.h"


Testbench::Testbench(int motorID1, int motorID2, int motorID3, int motorID4, int motorID5) 
{

    m_Motor1  = new CowLib::CowMotorController(motorID1, CowMotor::PHOENIX_V6);
    m_Motor2  = new CowLib::CowMotorController(motorID2, CowMotor::PHOENIX_V6);
    m_Motor3  = new CowLib::CowMotorController(motorID3, CowMotor::PHOENIX_V6);
    m_Motor4  = new CowLib::CowMotorController(motorID4, CowMotor::PHOENIX_V6);
    m_Motor5  = new CowLib::CowMotorController(motorID5, CowMotor::PHOENIX_V6);
    m_Motor1->SetNeutralMode(CowMotor::COAST);
    m_Motor2->SetNeutralMode(CowMotor::COAST);
    m_Motor3->SetNeutralMode(CowMotor::COAST);
    m_Motor4->SetNeutralMode(CowMotor::COAST);
    m_Motor5->SetNeutralMode(CowMotor::COAST);

}

void Testbench::SetMotor1(double percent)
{

    m_Motor1ControlRequest.PercentOut = percent;

}

void Testbench::SetMotor2(double percent)
{

    m_Motor2ControlRequest.PercentOut = percent;

}

void Testbench::SetMotor3(double percent)
{

    m_Motor3ControlRequest.PercentOut = percent;

}

void Testbench::SetMotor4(double percent)
{

    m_Motor4ControlRequest.PercentOut = percent;

}

void Testbench::SetMotor5(double percent)
{

    m_Motor5ControlRequest.PercentOut = percent;
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

    if(m_Motor3)
    {
        m_Motor3->Set(m_Motor3ControlRequest);
    }

    if(m_Motor4)
    {
        m_Motor4->Set(m_Motor4ControlRequest);
    }

    if(m_Motor5)
    {
        m_Motor5->Set(m_Motor5ControlRequest);
    }



}

