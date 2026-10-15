#include <frc/TimedRobot.h>
#include <frc/RobotBase.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <array>
#include <cmath>

#include "Constants.h"
#include "DriveTrain.h"
#include "GyroSensor.h"
#include "IRSensor.h"
#include "LineFollower.h"
#include "RobotDiagnostics.h"
#include "UltrasonicSensor.h"


class Robot : public frc::TimedRobot
{
public:
    void RobotInit() override
    {
        driveTrain.Stop();
        gyroXAngleDeg = 0.0;
    }

    void AutonomousInit() override
    {
        gyroXAngleDeg = 0.0;
        StartAutonomous();
    }

    void AutonomousPeriodic() override
    {
        UpdateGyro();
        RunAutonomous();
        PublishDashboard();
    }

    void TeleopInit() override
    {
        StopAutonomous();

        ltWasPressed = false;
        gyroXAngleDeg = 0.0;
    }

    void TeleopPeriodic() override
    {
        UpdateGyro();

        const bool ltPressed =
            controller.GetRawButton(kLTButton);

        const bool rtPressed =
            controller.GetRawButton(kRTButton);

        // RT cancels autonomous and returns control to the driver.
        if (rtPressed)
        {
            StopAutonomous();
            PublishDashboard();

            ltWasPressed = ltPressed;
            return;
        }

        // LT starts the autonomous sequence while in teleop.
        if (ltPressed &&
            !ltWasPressed &&
            !IsAutoRunning())
        {
            StartAutonomous();
        }

        ltWasPressed = ltPressed;

        if (IsAutoRunning())
        {
            RunAutonomous();
        }
        else
        {
            RunManualDrive();
        }

        PublishDashboard();
    }

    void DisabledInit() override
    {
        StopAutonomous();
    }


private:
    // =========================================================
    // HARDWARE / NAVIGATION SUBSYSTEMS
    // =========================================================

    frc::Joystick controller{
        Constants::kControllerPort
    };

    DriveTrain driveTrain;

    LineFollower lineFollower;

    GyroSensor gyro;

    IRSensor frontIR{
        Constants::kLeftWallIrAnalogPort
    };

    UltrasonicSensor leftUltrasonic{
        Constants::kUltrasonicTriggerChannel,
        Constants::kUltrasonicEchoChannel
    };

    RobotDiagnostics diagnostics{
        driveTrain,
        gyro
    };


    // =========================================================
    // CONTROLLER
    // =========================================================

    static constexpr int kLeftXAxis = 0;
    static constexpr int kLeftYAxis = 1;
    static constexpr int kRightXAxis = 2;

    static constexpr int kLTButton = 7;
    static constexpr int kRTButton = 8;

    static constexpr double kDriveMaxPower = 0.55;
    static constexpr double kJoystickDeadband = 0.08;


    // =========================================================
    // AUTONOMOUS NAVIGATION STATES
    //
    // AwaitPickup and AwaitPlacement are deliberate hand-off
    // points. The manipulator-integration branch will replace
    // those holds with the lift/gripper pickup and placement
    // state sequences.
    // =========================================================

    enum class AutoState
    {
        Idle,

        // Initial trip to source
        StrafeRight450,
        DriveForward650,
        FindLeftWall,
        DriveForwardUntilIR,

        // Source-column acquisition
        FindFirstStrip,
        ScanForTargetStrip,
        AlignTargetStrip,

        // Manipulator hand-off
        AwaitPickup,

        // Travel to flag
        TurnClockwise90,
        DriveToFlag1200,
        StrafeToFlagSlot,

        // Manipulator hand-off
        AwaitPlacement,

        // Leave flag
        BackAwayFlag200,
        StrafeBackFromFlagSlot,

        // Return to source
        ReturnFromFlagRemaining,
        TurnCounterClockwise90,
        ReturnForward350,
        ReturnAlignLeftWall,
        ReturnForwardUntilIR,

        Finished,
        Fault
    };

    AutoState autoState = AutoState::Idle;


    // =========================================================
    // NAVIGATION CALIBRATION
    // =========================================================

    static constexpr double kAutoStrafeRightMm = 450.0;
    static constexpr double kAutoForwardFirstMm = 650.0;

    static constexpr double kFlagTravelFromC1Mm = 1200.0;
    static constexpr double kSourceColumnSpacingMm = 105.0;

    static constexpr double kFlagClearanceBackMm = 200.0;
    static constexpr double kFlagSlotSpacingMm = 105.0;
    static constexpr double kFlagSlotDirection = +1.0;

    static constexpr double kReturnForwardApproachMm = 350.0;

    static constexpr double kAutoDrivePower = 0.35;
    static constexpr double kAutoSearchPower = 0.30;
    static constexpr double kFlagTravelPower = 0.35;
    static constexpr double kFlagSlotPower = 0.10;
    static constexpr double kFlagRetreatPower = 0.22;
    static constexpr double kReturnDrivePower = 0.30;
    static constexpr double kReturnApproachPower = 0.22;
    static constexpr double kReturnWallPower = 0.16;
    static constexpr double kLineSearchPower = 0.10;
    static constexpr double kLineAlignPower = 0.07;

    static constexpr double kLeftWallStopMm = 225.0;
    static constexpr double kWallToleranceMm = 10.0;
    static constexpr double kSourceIRStopVoltage = 1.30;
    static constexpr double kBlackThreshold = 4.0;

    static constexpr int kConfirmLoops = 5;

    static constexpr double kGyroXScale = 1.06184;
    static constexpr double kTurnKp = 0.012;
    static constexpr double kMaxTurnOutput = 0.22;
    static constexpr double kMinTurnOutput = 0.055;
    static constexpr double kTurnToleranceDeg = 1.0;


    // =========================================================
    // STATE VARIABLES
    // =========================================================

    int currentSourceColumn = 1;

    int stripCount = 0;
    bool wasOnBlack = false;

    int confirmLoops = 0;

    bool ltWasPressed = false;

    double gyroXAngleDeg = 0.0;
    double turnStartAngleDeg = 0.0;

    double flagReturnRemainingMm = 0.0;

    std::array<double, 3> moveStart{
        0.0,
        0.0,
        0.0
    };


    // =========================================================
    // GENERAL HELPERS
    // =========================================================

    void UpdateGyro()
    {
        // Preserve the calibrated X-axis integration used by the
        // working monolithic Robot.cpp.
        gyroXAngleDeg +=
            gyro.GetRawGyroX()
            *
            0.02
            *
            kGyroXScale;
    }

    double ApplyDeadband(double value)
    {
        if (std::abs(value) < kJoystickDeadband)
        {
            return 0.0;
        }

        return value;
    }


    // =========================================================
    // MANUAL DRIVE
    // =========================================================

    void RunManualDrive()
    {
        const double vx =
            ApplyDeadband(
                controller.GetRawAxis(kLeftXAxis)
            );

        const double vy =
            ApplyDeadband(
                controller.GetRawAxis(kLeftYAxis)
            );

        const double omega =
            ApplyDeadband(
                -controller.GetRawAxis(kRightXAxis)
            );

        driveTrain.DriveRobotRelative(
            vx,
            vy,
            omega,
            kDriveMaxPower
        );
    }


    // =========================================================
    // AUTO CONTROL
    // =========================================================

    bool IsAutoRunning() const
    {
        return
            autoState != AutoState::Idle &&
            autoState != AutoState::Finished &&
            autoState != AutoState::Fault;
    }

    void StartAutonomous()
    {
        driveTrain.Stop();

        currentSourceColumn = 1;
        confirmLoops = 0;
        stripCount = 0;
        wasOnBlack = false;
        flagReturnRemainingMm = 0.0;

        // Manipulator integration will insert HomeLift before this.
        autoState = AutoState::StrafeRight450;

        StartMove();
    }

    void StopAutonomous()
    {
        driveTrain.Stop();

        autoState = AutoState::Idle;
        confirmLoops = 0;
        stripCount = 0;
    }

    void ChangeAutoState(AutoState newState)
    {
        driveTrain.Stop();

        autoState = newState;
        confirmLoops = 0;

        StartMove();
    }


    // =========================================================
    // DRIVE ENCODER HELPERS
    // =========================================================

    void StartMove()
    {
        moveStart =
            driveTrain.GetWheelDistancesMm();
    }

    // +X = ROBOT RIGHT
    double GetDeltaX()
    {
        const auto now =
            driveTrain.GetWheelDistancesMm();

        const double dRear =
            now[0] - moveStart[0];

        const double dFR =
            now[1] - moveStart[1];

        const double dFL =
            now[2] - moveStart[2];

        return
            (dFR + dFL - 2.0 * dRear)
            / 3.0;
    }

    // +Y = ROBOT FORWARD
    double GetDeltaY()
    {
        const auto now =
            driveTrain.GetWheelDistancesMm();

        const double dFR =
            now[1] - moveStart[1];

        const double dFL =
            now[2] - moveStart[2];

        return
            (dFL - dFR)
            / std::sqrt(3.0);
    }


    // =========================================================
    // ROUTE GEOMETRY
    // =========================================================

    double GetFlagTravelTargetMm() const
    {
        const double compensation =
            (currentSourceColumn - 1)
            *
            kSourceColumnSpacingMm;

        return
            kFlagTravelFromC1Mm
            -
            compensation;
    }

    double GetFlagSlotOffsetMm() const
    {
        return
            (currentSourceColumn - 1)
            *
            kFlagSlotSpacingMm;
    }


    // =========================================================
    // COBRA HELPERS
    // =========================================================

    bool IsCobraOnBlack()
    {
        return
            lineFollower.GetVoltage(0) >= kBlackThreshold ||
            lineFollower.GetVoltage(1) >= kBlackThreshold ||
            lineFollower.GetVoltage(2) >= kBlackThreshold ||
            lineFollower.GetVoltage(3) >= kBlackThreshold;
    }

    bool AreMiddleLineSensorsOnBlack()
    {
        return
            lineFollower.GetVoltage(1) >= kBlackThreshold &&
            lineFollower.GetVoltage(2) >= kBlackThreshold;
    }

    enum class CobraAlignment
    {
        Unknown,
        StripLeft,
        Centered,
        StripRight
    };

    CobraAlignment GetCobraAlignment()
    {
        const bool left =
            lineFollower.GetVoltage(0) >= kBlackThreshold;

        const bool centerLeft =
            lineFollower.GetVoltage(1) >= kBlackThreshold;

        const bool centerRight =
            lineFollower.GetVoltage(2) >= kBlackThreshold;

        const bool right =
            lineFollower.GetVoltage(3) >= kBlackThreshold;

        if (centerLeft &&
            centerRight &&
            !left &&
            !right)
        {
            return CobraAlignment::Centered;
        }

        if (left)
        {
            return CobraAlignment::StripLeft;
        }

        if (right)
        {
            return CobraAlignment::StripRight;
        }

        if (centerLeft && !centerRight)
        {
            return CobraAlignment::StripLeft;
        }

        if (centerRight && !centerLeft)
        {
            return CobraAlignment::StripRight;
        }

        return CobraAlignment::Unknown;
    }


    // =========================================================
    // TURN CONTROL
    // =========================================================

    bool RunTurnToRelativeAngle(double targetDeg)
    {
        const double turned =
            gyroXAngleDeg -
            turnStartAngleDeg;

        const double error =
            targetDeg -
            turned;

        if (std::abs(error) <= kTurnToleranceDeg)
        {
            driveTrain.Stop();
            return true;
        }

        double power =
            std::abs(error)
            *
            kTurnKp;

        if (power > kMaxTurnOutput)
        {
            power = kMaxTurnOutput;
        }

        if (power < kMinTurnOutput)
        {
            power = kMinTurnOutput;
        }

        const double direction =
            (error > 0.0)
            ? -1.0
            : +1.0;

        driveTrain.DriveRobotRelative(
            0.0,
            0.0,
            direction,
            power
        );

        return false;
    }


    // =========================================================
    // AUTONOMOUS NAVIGATION
    // =========================================================

    void RunAutonomous()
    {
        switch (autoState)
        {
            case AutoState::StrafeRight450:
            {
                if (GetDeltaX() >= kAutoStrafeRightMm)
                {
                    ChangeAutoState(
                        AutoState::DriveForward650
                    );
                    break;
                }

                driveTrain.DriveRobotRelative(
                    +1.0,
                    0.0,
                    0.0,
                    kAutoDrivePower
                );

                break;
            }


            case AutoState::DriveForward650:
            {
                if (GetDeltaY() >= kAutoForwardFirstMm)
                {
                    ChangeAutoState(
                        AutoState::FindLeftWall
                    );
                    break;
                }

                driveTrain.DriveRobotRelative(
                    0.0,
                    -1.0,
                    0.0,
                    kAutoDrivePower
                );

                break;
            }


            case AutoState::FindLeftWall:
            {
                if (!leftUltrasonic.IsRangeValid())
                {
                    driveTrain.Stop();
                    confirmLoops = 0;
                    break;
                }

                if (leftUltrasonic.GetDistanceMillimeters()
                    <= kLeftWallStopMm)
                {
                    driveTrain.Stop();
                    ++confirmLoops;

                    if (confirmLoops >= kConfirmLoops)
                    {
                        ChangeAutoState(
                            AutoState::DriveForwardUntilIR
                        );
                    }
                }
                else
                {
                    confirmLoops = 0;

                    driveTrain.DriveRobotRelative(
                        -1.0,
                        0.0,
                        0.0,
                        kAutoSearchPower
                    );
                }

                break;
            }


            case AutoState::DriveForwardUntilIR:
            {
                if (frontIR.GetVoltage()
                    >= kSourceIRStopVoltage)
                {
                    driveTrain.Stop();
                    ++confirmLoops;

                    if (confirmLoops >= kConfirmLoops)
                    {
                        ChangeAutoState(
                            AutoState::FindFirstStrip
                        );
                    }
                }
                else
                {
                    confirmLoops = 0;

                    driveTrain.DriveRobotRelative(
                        0.0,
                        -1.0,
                        0.0,
                        kAutoSearchPower
                    );
                }

                break;
            }


            case AutoState::FindFirstStrip:
            {
                if (AreMiddleLineSensorsOnBlack())
                {
                    driveTrain.Stop();
                    ++confirmLoops;

                    if (confirmLoops >= kConfirmLoops)
                    {
                        ChangeAutoState(
                            AutoState::AwaitPickup
                        );
                    }
                }
                else
                {
                    confirmLoops = 0;

                    driveTrain.DriveRobotRelative(
                        +1.0,
                        0.0,
                        0.0,
                        kLineSearchPower
                    );
                }

                break;
            }


            case AutoState::ScanForTargetStrip:
            {
                const bool onBlack =
                    IsCobraOnBlack();

                if (onBlack && !wasOnBlack)
                {
                    ++stripCount;

                    if (stripCount == currentSourceColumn)
                    {
                        driveTrain.Stop();
                        autoState = AutoState::AlignTargetStrip;
                        confirmLoops = 0;
                        break;
                    }
                }

                wasOnBlack = onBlack;

                driveTrain.DriveRobotRelative(
                    +1.0,
                    0.0,
                    0.0,
                    kLineSearchPower
                );

                break;
            }


            case AutoState::AlignTargetStrip:
            {
                const CobraAlignment alignment =
                    GetCobraAlignment();

                if (alignment == CobraAlignment::Centered)
                {
                    driveTrain.Stop();
                    ++confirmLoops;

                    if (confirmLoops >= kConfirmLoops)
                    {
                        ChangeAutoState(
                            AutoState::AwaitPickup
                        );
                    }
                }
                else if (alignment == CobraAlignment::StripLeft)
                {
                    confirmLoops = 0;

                    driveTrain.DriveRobotRelative(
                        -1.0,
                        0.0,
                        0.0,
                        kLineAlignPower
                    );
                }
                else if (alignment == CobraAlignment::StripRight)
                {
                    confirmLoops = 0;

                    driveTrain.DriveRobotRelative(
                        +1.0,
                        0.0,
                        0.0,
                        kLineAlignPower
                    );
                }
                else
                {
                    driveTrain.Stop();
                    confirmLoops = 0;
                }

                break;
            }


            case AutoState::AwaitPickup:
            {
                // Integration hand-off:
                // teammate adds lift/gripper pickup states here.
                driveTrain.Stop();
                break;
            }


            case AutoState::TurnClockwise90:
            {
                if (RunTurnToRelativeAngle(+90.0))
                {
                    ChangeAutoState(
                        AutoState::DriveToFlag1200
                    );
                }

                break;
            }


            case AutoState::DriveToFlag1200:
            {
                const double target =
                    GetFlagTravelTargetMm();

                if (GetDeltaY() >= target)
                {
                    ChangeAutoState(
                        AutoState::StrafeToFlagSlot
                    );
                    break;
                }

                driveTrain.DriveRobotRelative(
                    0.0,
                    -1.0,
                    0.0,
                    kFlagTravelPower
                );

                break;
            }


            case AutoState::StrafeToFlagSlot:
            {
                const double offset =
                    GetFlagSlotOffsetMm();

                if (offset <= 1.0)
                {
                    ChangeAutoState(
                        AutoState::AwaitPlacement
                    );
                    break;
                }

                const double moved =
                    kFlagSlotDirection
                    *
                    GetDeltaX();

                if (moved >= offset)
                {
                    ChangeAutoState(
                        AutoState::AwaitPlacement
                    );
                    break;
                }

                driveTrain.DriveRobotRelative(
                    kFlagSlotDirection,
                    0.0,
                    0.0,
                    kFlagSlotPower
                );

                break;
            }


            case AutoState::AwaitPlacement:
            {
                // Integration hand-off:
                // teammate adds lift/gripper placement states here.
                driveTrain.Stop();
                break;
            }


            case AutoState::BackAwayFlag200:
            {
                if (-GetDeltaY() >= kFlagClearanceBackMm)
                {
                    flagReturnRemainingMm =
                        GetFlagTravelTargetMm()
                        -
                        kFlagClearanceBackMm;

                    if (flagReturnRemainingMm < 0.0)
                    {
                        flagReturnRemainingMm = 0.0;
                    }

                    ChangeAutoState(
                        AutoState::StrafeBackFromFlagSlot
                    );

                    break;
                }

                driveTrain.DriveRobotRelative(
                    0.0,
                    +1.0,
                    0.0,
                    kFlagRetreatPower
                );

                break;
            }


            case AutoState::StrafeBackFromFlagSlot:
            {
                const double offset =
                    GetFlagSlotOffsetMm();

                if (offset <= 1.0)
                {
                    ChangeAutoState(
                        AutoState::ReturnFromFlagRemaining
                    );
                    break;
                }

                const double movedBack =
                    -kFlagSlotDirection
                    *
                    GetDeltaX();

                if (movedBack >= offset)
                {
                    ChangeAutoState(
                        AutoState::ReturnFromFlagRemaining
                    );
                    break;
                }

                driveTrain.DriveRobotRelative(
                    -kFlagSlotDirection,
                    0.0,
                    0.0,
                    kFlagSlotPower
                );

                break;
            }


            case AutoState::ReturnFromFlagRemaining:
            {
                if (-GetDeltaY() >= flagReturnRemainingMm)
                {
                    driveTrain.Stop();

                    turnStartAngleDeg =
                        gyroXAngleDeg;

                    autoState =
                        AutoState::TurnCounterClockwise90;

                    break;
                }

                driveTrain.DriveRobotRelative(
                    0.0,
                    +1.0,
                    0.0,
                    kReturnDrivePower
                );

                break;
            }


            case AutoState::TurnCounterClockwise90:
            {
                if (RunTurnToRelativeAngle(-90.0))
                {
                    ChangeAutoState(
                        AutoState::ReturnForward350
                    );
                }

                break;
            }


            case AutoState::ReturnForward350:
            {
                if (GetDeltaY() >= kReturnForwardApproachMm)
                {
                    ChangeAutoState(
                        AutoState::ReturnAlignLeftWall
                    );
                    break;
                }

                driveTrain.DriveRobotRelative(
                    0.0,
                    -1.0,
                    0.0,
                    kReturnApproachPower
                );

                break;
            }


            case AutoState::ReturnAlignLeftWall:
            {
                if (!leftUltrasonic.IsRangeValid())
                {
                    driveTrain.Stop();
                    confirmLoops = 0;
                    break;
                }

                const double distance =
                    leftUltrasonic.GetDistanceMillimeters();

                const double error =
                    distance -
                    kLeftWallStopMm;

                if (std::abs(error) <= kWallToleranceMm)
                {
                    driveTrain.Stop();
                    ++confirmLoops;

                    if (confirmLoops >= kConfirmLoops)
                    {
                        ChangeAutoState(
                            AutoState::ReturnForwardUntilIR
                        );
                    }
                }
                else if (error > 0.0)
                {
                    confirmLoops = 0;

                    driveTrain.DriveRobotRelative(
                        -1.0,
                        0.0,
                        0.0,
                        kReturnWallPower
                    );
                }
                else
                {
                    confirmLoops = 0;

                    driveTrain.DriveRobotRelative(
                        +1.0,
                        0.0,
                        0.0,
                        kReturnWallPower
                    );
                }

                break;
            }


            case AutoState::ReturnForwardUntilIR:
            {
                if (frontIR.GetVoltage()
                    >= kSourceIRStopVoltage)
                {
                    driveTrain.Stop();
                    ++confirmLoops;

                    if (confirmLoops >= kConfirmLoops)
                    {
                        if (currentSourceColumn >= 4)
                        {
                            autoState = AutoState::Finished;
                            break;
                        }

                        ++currentSourceColumn;

                        stripCount = 0;
                        wasOnBlack = IsCobraOnBlack();
                        confirmLoops = 0;

                        autoState =
                            AutoState::ScanForTargetStrip;
                    }
                }
                else
                {
                    confirmLoops = 0;

                    driveTrain.DriveRobotRelative(
                        0.0,
                        -1.0,
                        0.0,
                        kReturnWallPower
                    );
                }

                break;
            }


            case AutoState::Finished:
            case AutoState::Fault:
            case AutoState::Idle:
            {
                driveTrain.Stop();
                break;
            }
        }
    }


    // =========================================================
    // DASHBOARD
    // =========================================================

    void PublishDashboard()
    {
        diagnostics.Publish();

        frc::SmartDashboard::PutBoolean(
            "Auto/Running",
            IsAutoRunning()
        );

        frc::SmartDashboard::PutNumber(
            "Auto/State",
            static_cast<int>(autoState)
        );

        frc::SmartDashboard::PutNumber(
            "Auto/Current Source Column",
            currentSourceColumn
        );

        frc::SmartDashboard::PutNumber(
            "Auto/Strip Count",
            stripCount
        );

        frc::SmartDashboard::PutNumber(
            "Auto/Delta X mm",
            GetDeltaX()
        );

        frc::SmartDashboard::PutNumber(
            "Auto/Delta Y mm",
            GetDeltaY()
        );

        frc::SmartDashboard::PutNumber(
            "Flag/Travel Target mm",
            GetFlagTravelTargetMm()
        );

        frc::SmartDashboard::PutNumber(
            "Flag/Slot Offset mm",
            GetFlagSlotOffsetMm()
        );

        frc::SmartDashboard::PutNumber(
            "Flag/Return Remaining mm",
            flagReturnRemainingMm
        );

        frc::SmartDashboard::PutNumber(
            "Gyro/X Angle deg",
            gyroXAngleDeg
        );

        frc::SmartDashboard::PutBoolean(
            "Ultrasonic/Valid",
            leftUltrasonic.IsRangeValid()
        );

        frc::SmartDashboard::PutNumber(
            "Ultrasonic/Left mm",
            leftUltrasonic.GetDistanceMillimeters()
        );

        frc::SmartDashboard::PutNumber(
            "IR/Front Voltage",
            frontIR.GetVoltage()
        );

        frc::SmartDashboard::PutNumber(
            "Cobra/0",
            lineFollower.GetVoltage(0)
        );

        frc::SmartDashboard::PutNumber(
            "Cobra/1",
            lineFollower.GetVoltage(1)
        );

        frc::SmartDashboard::PutNumber(
            "Cobra/2",
            lineFollower.GetVoltage(2)
        );

        frc::SmartDashboard::PutNumber(
            "Cobra/3",
            lineFollower.GetVoltage(3)
        );

        frc::SmartDashboard::PutBoolean(
            "Cobra/On Black",
            IsCobraOnBlack()
        );

        frc::SmartDashboard::PutBoolean(
            "Cobra/Middle Black",
            AreMiddleLineSensorsOnBlack()
        );
    }
};


#ifndef RUNNING_FRC_TESTS

int main()
{
    return frc::StartRobot<Robot>();
}

#endif
