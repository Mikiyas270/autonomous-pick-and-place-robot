#include "AutonomousController.h"


AutonomousController::AutonomousController()
    : currentState(State::START)
{
}


void AutonomousController::Reset() {
    currentState = State::START;
}


void AutonomousController::Update() {

    // No motors are commanded yet.
    // We will add transitions and movement after
    // tomorrow's encoder/gyro/sensor calibration.

    switch (currentState) {

        case State::START:
            // Eventually:
            // - zero gyro
            // - reset encoders
            // - home lift
            // - open gripper
            break;


        case State::LEAVE_START_AREA:
            // Move clear of initial starting position.
            break;


        case State::NAVIGATE_AROUND_WALL:
            // Encoders + gyro:
            // follow a known safe path around
            // the internal arena wall.
            break;


        case State::MOVE_TO_SHELF_STAGING:
            // Reach a known position where the
            // camera has a clear view of the shelf.
            break;


        case State::SEARCH_FOR_CUBE:
            // Camera identifies required cube.
            break;


        case State::ALIGN_WITH_CUBE:
            // Kiwi-drive lateral motion +
            // camera horizontal error.
            break;


        case State::APPROACH_CUBE:
            // Camera + ultrasonic + IR.
            break;


        case State::GRAB_CUBE:
            // Close geared gripper.
            break;


        case State::LIFT_CUBE:
            // Raise lift safely.
            break;


        case State::RETREAT_FROM_SHELF:
            // Back away before navigating elsewhere.
            break;


        case State::NAVIGATE_TO_TARGET:
            // Encoders + gyro navigate around
            // arena geometry toward delivery area.
            break;


        case State::MOVE_TO_TARGET_STAGING:
            // Reach known staging point in front
            // of target / flag stand.
            break;


        case State::ALIGN_WITH_COLUMN:
            // Use line follower / gyro / other
            // local sensing for precise column alignment.
            break;


        case State::LOWER_CUBE:
            // Lower lift to placement height.
            break;


        case State::RELEASE_CUBE:
            // Open gripper.
            break;


        case State::RETREAT_FROM_TARGET:
            // Back away safely after placement.
            break;


        case State::COMPLETE:
            // Robot stopped.
            break;
    }
}


AutonomousController::State
AutonomousController::GetState() const {
    return currentState;
}