#include "SwerveDriveController.h"

template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>;

double FilterAxis(double value, double outputMin, double outputMax)
{
    // Deadband
    double filteredValue = CowLib::Deadband(value, CONSTANT("STICK_DEADBAND"));

    if (filteredValue == 0.0)
    {
        return 0.0;
    }
    else if (filteredValue > 0.0)
    {
        filteredValue -= CONSTANT("STICK_DEADBAND");
    }
    else if (filteredValue < 0.0)
    {
        filteredValue += CONSTANT("STICK_DEADBAND");
    }

    // Normalize values after deadband
    filteredValue = filteredValue / (1.0 - CONSTANT("STICK_DEADBAND"));

    // Exponential filter
    filteredValue = CowLib::ExponentialFilter(filteredValue, CONSTANT("STICK_EXPONENTIAL_MODIFIER"));

    // Scale values to outputMin and outputMax
    return std::copysign(outputMin + (std::fabs(filteredValue) * (outputMax - outputMin)), filteredValue);
}

std::tuple<double, double> FilterXY(double inputX, double inputY, double outputMin, double outputMax)
{
    double magnitude = std::sqrt(std::pow(inputX, 2.0) + std::pow(inputY, 2.0));
    double filteredMagnitude = FilterAxis(magnitude, outputMin, outputMax);
    double theta = std::atan2(inputY, inputX);

    return std::make_tuple(filteredMagnitude * std::cos(theta), filteredMagnitude * std::sin(theta));
}

SwerveDriveController::SwerveDriveController(SwerveDrive &drivetrain)
    : m_Drivetrain(drivetrain),
      m_Gyro(*CowPigeon::GetInstance()),
      m_State(IdleState{}),
      m_HeadingPIDController(0.0, 0.0, 0.0, frc::TrapezoidProfile<units::degrees>::Constraints(), 10_ms),
      m_IsOnTarget(false),
      m_LPF(CONSTANT("HEADING_LPF_BETA"))
{
    m_HeadingPIDController.EnableContinuousInput(units::degree_t{ 0.0 }, units::degree_t{ 360.0 });

    ResetConstants();
}

void SwerveDriveController::ResetConstants()
{
    m_HeadingPIDController.SetPID(CONSTANT("HEADING_P"), CONSTANT("HEADING_I"), CONSTANT("HEADING_D"));
    m_HeadingPIDController.SetConstraints(frc::TrapezoidProfile<units::degrees>::Constraints{
        units::degrees_per_second_t{ CONSTANT("HEADING_V") },
        units::degrees_per_second_squared_t{ CONSTANT("HEADING_A") } });

    m_LPF.UpdateBeta(CONSTANT("HEADING_LPF_BETA"));
}

void SwerveDriveController::ResetHeadingLock()
{
    // Resetting the heading lock is only applicable in the manual and lock
    // heading states
    if (std::holds_alternative<DriveManualState>(m_State))
    {
        DriveManualState &state = std::get<DriveManualState>(m_State); 
        state.targetHeading = std::nullopt;
    }
    else if (std::holds_alternative<DriveLockHeadingState>(m_State))
    {
        DriveLockHeadingState &state = std::get<DriveLockHeadingState>(m_State); 
        state.targetHeading = std::nullopt;
    }
}

bool SwerveDriveController::IsOnTarget()
{
    return m_IsOnTarget;
}

void SwerveDriveController::Request(DriveManualRequest req)
{
    if (std::holds_alternative<DriveManualState>(m_State))
    {
        // Already in drive manual state, just update the inputs
        DriveManualState &state = std::get<DriveManualState>(m_State); 
        state.req = req;
    }
    else
    {
        // Set new state
        m_State = DriveManualState {
            .req = req,
            .targetHeading = std::nullopt
        };

        // Reset the heading PID controller to clear previous error and integral accumulator
        m_HeadingPIDController.Reset(units::degree_t{ m_Gyro.GetYawDegrees() });
    }
}

void SwerveDriveController::Request(DriveLockHeadingRequest req)
{
    if (std::holds_alternative<DriveLockHeadingState>(m_State))
    {
        // Already in drive lock heading state, just update the inputs
        DriveLockHeadingState &state = std::get<DriveLockHeadingState>(m_State); 
        state.req = req;
    }
    else
    {
        // Set new state
        DriveLockHeadingState newState = {
            .req = req,
            .targetHeading = std::nullopt
        };

        m_State = newState;

        // Reset the heading PID controller to clear previous error and integral accumulator
        m_HeadingPIDController.Reset(units::degree_t{ m_Drivetrain.GetPoseRot() });
    }
}

void SwerveDriveController::Request(DriveLookAtRequest req)
{
    // If this is a state transition, reset the heading PID controller to clear
    // previous error and integral accumulator
    if (!std::holds_alternative<DriveLookAtState>(m_State))
    {
        printf("resetting pid with %f\n", m_Drivetrain.GetPoseRot());
        m_HeadingPIDController.Reset(units::degree_t{ m_Drivetrain.GetPoseRot() });
        m_LPF.ReInit(m_Drivetrain.GetPoseRot(), m_Drivetrain.GetPoseRot());
    }

    m_State = DriveLookAtState {
        .req = req
    };
}

void SwerveDriveController::Handle()
{
    std::visit(overload {
        [&](IdleState &state) {
            m_Drivetrain.SetVelocity(0.0, 0.0, 0.0);
            m_IsOnTarget = true;
        },
        [&](DriveManualState &state) {            
            auto [xVel, yVel] = FilterXY(
                state.req.inputX,
                state.req.inputY,
                CONSTANT("DESIRED_MIN_SPEED"),
                CONSTANT("DESIRED_MAX_SPEED"));

            double filteredRotation = FilterAxis(
                state.req.inputRotation,
                CONSTANT("DESIRED_MIN_ANG_VEL"),
                CONSTANT("DESIRED_MAX_ANG_VEL"));

            double omega = 0.0;

            m_IsOnTarget = false;

            if (filteredRotation == 0.0)
            {
                if (state.targetHeading.has_value())
                {
                    double targetHeading = state.targetHeading.value();
                    omega = m_HeadingPIDController.Calculate(
                        units::degree_t{ m_Gyro.GetYawDegrees() },
                        units::degree_t{ targetHeading });

                    m_IsOnTarget = std::fabs(m_HeadingPIDController.GetPositionError().value()) < CONSTANT("SHOOTING_THRESHOLD_HEADING_ERROR");
                }
                else if (m_Gyro.GetYawVelocityDegrees() < CONSTANT("HEADING_CAPTURE_THRESHOLD"))
                {
                    state.targetHeading = m_Gyro.GetYawDegrees();
                    m_HeadingPIDController.Reset(units::degree_t{ state.targetHeading.value() });
                }
            }
            else
            {
                omega = filteredRotation;
                state.targetHeading = std::nullopt;
            }

            if (xVel != 0.0 || yVel != 0.0 || filteredRotation != 0.0)
            {
                m_Drivetrain.SetVelocity(xVel, yVel, omega);
            }
            else
            {
                m_Drivetrain.SetVelocity(0.0, 0.0, 0.0);
            }
        },
        [&](DriveLockHeadingState &state) {
            if (!state.targetHeading.has_value())
            {
                state.targetHeading = m_Drivetrain.GetPoseRot();
            }

            auto [xVel, yVel] = FilterXY(
                state.req.inputX,
                state.req.inputY,
                CONSTANT("DESIRED_MIN_SPEED"),
                CONSTANT("DESIRED_MAX_SPEED"));

            double omega = m_HeadingPIDController.Calculate(
                units::degree_t{ m_Drivetrain.GetPoseRot() },
                units::degree_t{ state.targetHeading.value() });

            m_Drivetrain.SetVelocity(xVel, yVel, omega);
            m_IsOnTarget = std::fabs(m_HeadingPIDController.GetPositionError().value()) < CONSTANT("SHOOTING_THRESHOLD_HEADING_ERROR");
        },
        [&](DriveLookAtState &state) {
            auto [xVel, yVel] = FilterXY(
                state.req.inputX,
                state.req.inputY,
                CONSTANT("DESIRED_MIN_SPEED"),
                CONSTANT("DESIRED_MAX_SPEED"));

            frc::Pose2d lookaheadPose = m_Drivetrain.Odometry()->Lookahead(state.req.lookaheadTime).value_or(m_Drivetrain.GetPose());

            double targetAngle = std::atan2(
                state.req.targetY - lookaheadPose.Y().convert<units::foot>().value(),
                state.req.targetX - lookaheadPose.X().convert<units::foot>().value());
                
            targetAngle = (targetAngle / 3.1415) * 180;

            if (state.req.robotSide == RobotSide::FRONT)
            {
                targetAngle += 0;
            }
            else if (state.req.robotSide == RobotSide::RIGHT)
            {
                targetAngle += 90;
            }
            else if (state.req.robotSide == RobotSide::BACK)
            {
                targetAngle += 180;
            }
            else if (state.req.robotSide == RobotSide::LEFT)
            {
                targetAngle += 270;
            }

            // targetAngle = m_LPF.Calculate(targetAngle);

            double omega = m_HeadingPIDController.Calculate(
                units::degree_t{ m_Drivetrain.GetPoseRot() },
                units::degree_t{ targetAngle });

            m_Drivetrain.SetVelocity(xVel, yVel, omega);
            m_IsOnTarget = std::fabs(m_HeadingPIDController.GetPositionError().value()) < CONSTANT("SHOOTING_THRESHOLD_HEADING_ERROR");
        }
    }, m_State);
}