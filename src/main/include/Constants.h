#pragma once

namespace Constants {

// =====================================================
// TITAN / MOTOR MAPPING
// Shared hardware identifiers used by the subsystem files.
// =====================================================

constexpr int kTitanId = 42;

constexpr int kFrontLeftMotor  = 2;  // Titan M0
constexpr int kLiftMotor       = 1;  // Titan M1
constexpr int kFrontRightMotor = 3;  // Titan M2
constexpr int kRearMotor       = 0;  // Titan M3

// Lift.cpp already depends on this shared safety clamp.
// More detailed lift/manipulator constants are added later
// on the manipulator-integration branch.
constexpr double kLiftMaxOutput = 0.45;


// =====================================================
// CONTROLLER / TELEOP DRIVE
// =====================================================

constexpr int kControllerPort = 0;

constexpr int kLeftXAxis  = 0;
constexpr int kLeftYAxis  = 1;
constexpr int kRightXAxis = 2;

constexpr double kJoystickDeadband = 0.08;
constexpr double kTeleopMaxOutput = 0.5;


// =====================================================
// D-PAD TEST DRIVE
// =====================================================

constexpr double kDpadDriveOutput = 0.18;


// =====================================================
// DRIVE ENCODERS
// =====================================================

constexpr double kDriveWheelRadiusMm = 51.0;
constexpr double kEncoderCountsPerRev = 1464.0;
constexpr double kDriveGearRatio = 1.0;

constexpr double kPi =
    3.14159265358979323846;

constexpr double kDriveDistancePerTickMm =
    (2.0 * kPi * kDriveWheelRadiusMm) /
    (kEncoderCountsPerRev * kDriveGearRatio);


// =====================================================
// AUTONOMOUS NAVIGATION SPEEDS
// =====================================================

constexpr double kAutoDriveOutput = 0.18;
constexpr double kAutoApproachOutput = 0.10;
constexpr double kAutoSearchOutput = 0.08;
constexpr double kAutoAlignOutput = 0.05;
constexpr double kAutoLineFollowOutput = 0.08;
constexpr double kFlagApproachOutput = 0.07;


// =====================================================
// MASTER ROUTE ENABLES
// =====================================================

constexpr bool kStartToSourceRouteEnabled = false;
constexpr bool kSourceToFlagRouteEnabled = false;
constexpr bool kFlagToSourceRouteEnabled = false;


// =====================================================
// START -> SOURCE
// =====================================================

constexpr double kStartStrafeRightMm = 500.0;
constexpr double kStartForwardMm = 615.0;


// =====================================================
// IR REFERENCE
// =====================================================

constexpr int kLeftWallIrAnalogPort = 0;
constexpr double kLeftWallRobotClearanceMm = 30.0;
constexpr bool kLeftWallReferenceEnabled = false;
constexpr double kLeftWallStopVoltage = 0.0;
constexpr int kLeftWallConfirmLoops = 5;


// =====================================================
// ULTRASONIC
// =====================================================

constexpr int kUltrasonicTriggerChannel = 9;
constexpr int kUltrasonicEchoChannel = 8;


// =====================================================
// FRONT WALL REFERENCE
// =====================================================

constexpr bool kFrontWallReferenceEnabled = false;
constexpr double kFrontWallStopDistanceMm = 280.0;
constexpr int kFrontWallConfirmLoops = 5;


// =====================================================
// SOURCE SHELF / COBRA
// =====================================================

constexpr bool kSourceShelfAutoStopEnabled = false;
constexpr double kSourceShelfStopDistanceMm = 0.0;
constexpr int kSourceShelfConfirmLoops = 5;

constexpr bool kCobraAutoDetectionEnabled = false;


// =====================================================
// SOURCE RETREAT
// =====================================================

constexpr double kSourcePostStripRetreatMm = 580.0;
constexpr int kSourceStripExitConfirmLoops = 5;


// =====================================================
// GYRO TURN
// =====================================================

constexpr bool kFlagTurnControlEnabled = false;

constexpr double kFlagTurnDegrees = 90.0;
constexpr double kReturnTurnDegrees = 90.0;

constexpr double kAutoTurnOutput = 0.15;
constexpr double kAutoTurnSlowOutput = 0.08;
constexpr double kTurnSlowZoneDeg = 15.0;
constexpr double kTurnToleranceDeg = 2.0;


// =====================================================
// HEADING HOLD
// =====================================================

constexpr bool kHeadingHoldEnabled = false;
constexpr double kHeadingHoldKp = 0.02;
constexpr double kHeadingHoldToleranceDeg = 1.0;
constexpr double kHeadingHoldMaxOmega = 0.25;


// =====================================================
// SOURCE -> FLAG NAVIGATION
// =====================================================

constexpr double kFlagFirstForwardMm = 500.0;
constexpr double kFlagStrafeDownMm = 150.0;
constexpr double kFlagForwardBaseMm = 800.0;
constexpr double kSourceColumnSpacingMm = 105.0;


// =====================================================
// FLAG STAND / RETURN NAVIGATION
// =====================================================

constexpr bool kFlagStandAutoStopEnabled = false;
constexpr double kFlagStandStopDistanceMm = 0.0;
constexpr int kFlagStandConfirmLoops = 5;

constexpr bool kFlagFinalApproachEnabled = false;
constexpr double kFlagFinalApproachMm = 0.0;

constexpr double kFlagRetreatMm = 200.0;

}  // namespace Constants
