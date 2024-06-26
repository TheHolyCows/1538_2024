//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __OPERATOR_CONTROLLER_H__
#define __OPERATOR_CONTROLLER_H__

#include "../ControlBoards/ButtonMap.h"
#include "../ControlBoards/GenericControlBoard.h"
#include "../CowConstants.h"
#include "../CowLib/CowExponentialFilter.h"
#include "../CowLib/CowLatch.h"
#include "../CowLib/CowLib.h"
#include "../CowRobot.h"
#include "../Declarations.h"
// #include "../Subsystems/Vision.h"
#include "frc/controller/PIDController.h"

#include <iostream>
#include <math.h>
#include <stdio.h>

class OperatorController : public GenericController
{
private:
    OperatorController();
    GenericControlBoard *m_CB;

    bool m_ClimberLatch;

public:
    OperatorController(GenericControlBoard *controlboard);
    void Handle(CowRobot *bot);

    double m_TrackingCooldownTimer;

    double m_LastShotDistance;
    double m_LastShotPivot;
    double m_LastShotWrist;
};

#endif /* __OPERATOR_CONTROLLER_H__ */
