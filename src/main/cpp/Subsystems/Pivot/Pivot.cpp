//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Pivot.h"

#include "../../CowLib/CowLogger.h"

Pivot::Pivot(const int motorID)
{
    m_PivotMotor = new CowLib::CowMotorController(motorID,CowMotor::PHOENIX_V5,"cowbus");
    m_PivotMotor->SetNeutralMode(CowMotor::BRAKE);

    m_TargetAngle = 0;
    m_TickCount   = 0;

    ResetConstants();
}

void Pivot::RequestAngle(double angle)
{
    m_MotorRequest.Position = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("PIVOT_GEAR_RATIO"));
}

double Pivot::GetSetpoint()
{
    return CowLib::Conversions::FalconToDegrees(m_TargetAngle, CONSTANT("PIVOT_GEAR_RATIO"));
}

bool Pivot::AtTarget()
{
    return fabs(GetSetpoint() - GetAngle() < CONSTANT("PIVOT_TOLERANCE"));
}

double Pivot::GetAngle()
{
    return CowLib::Conversions::FalconToDegrees(m_PivotMotor->GetPosition(),
                                                CONSTANT("PIVOT_GEAR_RATIO"));
}

void Pivot::UpdatePID(double armExt)
{
    double p
        = CONSTANT("PIVOT_P_BASE") + (CONSTANT("PIVOT_P_EXTENSION") * armExt) + (CONSTANT("PIVOT_P_ANGLE") * armExt);

    m_PivotMotor->SetPID(p,CONSTANT("PIVOT_I"),CONSTANT("PIVOT_D"),CONSTANT("PIVOT_F"));
}

void Pivot::ResetConstants()
{
    // rewrite
    m_PivotMotor->SetPID(0,0,0,0);
    m_PivotMotor->SetMotionMagic(CONSTANT("PIVOT_A"),CONSTANT("PIVOT_V"));
}

void Pivot::Handle()
{
    m_PivotMotor->Set(m_MotorRequest);

    if (m_TickCount++ % 10 == 0) // 200 miliseconds
    {
        m_TickCount = 1;

        CowLib::CowLogger::LogMotor(9, 0, m_PivotMotor->GetPosition());
    }
}

void Pivot::BrakeMode(bool brakeMode)
{
    if (brakeMode)
    {
        m_PivotMotor->SetNeutralMode(CowMotor::BRAKE);
    }
    else
    {
        m_PivotMotor->SetNeutralMode(CowMotor::COAST);
    }
}
