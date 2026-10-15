#pragma once

class AutonomousController {
public:

    enum class State {

        // Initial setup
        START,

        // Navigate from starting area to a position
        // where the shelf/cubes can actually be seen
        LEAVE_START_AREA,
        NAVIGATE_AROUND_WALL,
        MOVE_TO_SHELF_STAGING,

        // Cube acquisition
        SEARCH_FOR_CUBE,
        ALIGN_WITH_CUBE,
        APPROACH_CUBE,
        GRAB_CUBE,
        LIFT_CUBE,

        // Leave shelf safely
        RETREAT_FROM_SHELF,

        // Navigate to delivery area
        NAVIGATE_TO_TARGET,
        MOVE_TO_TARGET_STAGING,
        ALIGN_WITH_COLUMN,

        // Place cube
        LOWER_CUBE,
        RELEASE_CUBE,
        RETREAT_FROM_TARGET,

        COMPLETE
    };

    AutonomousController();

    void Reset();
    void Update();

    State GetState() const;

private:
    State currentState;
};