#include "Testbench.h"

Testbench::Testbench(int motorID1, int motorID2)
{
    m_Motor1  = new CowMotor::TalonFX(motorID1, "cowbus");
    m_Motor2  = new CowMotor::TalonFX(motorID2, "cowbus");

    m_Motor1->ConfigNeutralMode(CowMotor::NeutralMode::COAST);
    m_Motor2->ConfigNeutralMode(CowMotor::NeutralMode::COAST);
}

void Testbench::SetMotor1(double percent)
{

    m_Motor1ControlRequest.DutyCycle = percent;

}

void Testbench::SetMotor2(double percent)
{

    m_Motor2ControlRequest.DutyCycle = percent;

}

void Testbench::Handle()
{
    if (m_Motor1)
    {
        m_Motor1->Set(m_Motor1ControlRequest);
    }

    if (m_Motor2)
    {
        m_Motor2->Set(m_Motor2ControlRequest);
    }
}

