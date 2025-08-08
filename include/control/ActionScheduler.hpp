#pragma once

#include "units/units.hpp"
#include <functional>
#include <vector>

namespace control {

/**
 * @brief Enum defining the type of action trigger
 */
enum class ActionTrigger {
    TIME_FROM_START,     // Trigger based on elapsed time from start
    TIME_FROM_END,       // Trigger based on remaining time to end
    DISTANCE_FROM_START, // Trigger based on distance traveled from start
    DISTANCE_FROM_END    // Trigger based on distance remaining to end
    // We can add more trigger types in the future as needed
};

/**
 * @brief A class that manages scheduled actions during motion
 */
class ActionScheduler {
public:
    /**
     * @brief Construct a new Action Scheduler
     */
    ActionScheduler();

    /**
     * @brief Add an action to be executed during motion
     * 
     * @param action Function to execute (takes no parameters, returns void)
     * @param trigger Type of trigger
     * @param value Trigger value in seconds (for time) or inches (for distance)
     * @return true if action was added successfully
     */
    bool addAction(std::function<void()> action, ActionTrigger trigger, double value);

    /**
     * @brief Add an action to be executed at a specific time from start
     * 
     * @param action Function to execute
     * @param t Time from motion start
     * @return true if action was added successfully
     */
    bool addActionAtTimeFromStart(std::function<void()> action, Time t);

    /**
     * @brief Add an action to be executed at a specific time from end
     * 
     * @param action Function to execute
     * @param t Time before motion end
     * @return true if action was added successfully
     */
    bool addActionAtTimeFromEnd(std::function<void()> action, Time t);

    /**
     * @brief Add an action to be executed at a specific distance from start
     * 
     * @param action Function to execute
     * @param d Distance from motion start
     * @return true if action was added successfully
     */
    bool addActionAtDistanceFromStart(std::function<void()> action, Length d);

    /**
     * @brief Add an action to be executed at a specific distance from end
     * 
     * @param action Function to execute
     * @param d Distance before motion end
     * @return true if action was added successfully
     */
    bool addActionAtDistanceFromEnd(std::function<void()> action, Length d);

    /**
     * @brief Clear all scheduled actions
     */
    void clearActions();

    /**
     * @brief Reset action execution states (for reuse)
     */
    void resetActions();

    /**
     * @brief Process scheduled actions based on current state
     * 
     * @param currentTime Current time in trajectory/motion
     * @param totalTime Total time of trajectory/motion
     * @param currentDistance Current distance traveled
     * @param totalDistance Total distance to travel
     */
    void processActions(Time currentTime, Time totalTime, Length currentDistance, Length totalDistance);

private:
    /**
     * @brief Structure to represent an action to be executed during motion
     */
    struct ScheduledAction {
        std::function<void()> action;    // Function to execute
        ActionTrigger trigger;           // Type of trigger
        double value;                    // Value for the trigger (time or distance)
        bool executed = false;           // Whether the action has been executed
    };

    std::vector<ScheduledAction> m_scheduledActions;  // List of scheduled actions
};

} // namespace control
