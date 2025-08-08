#include "control/ActionScheduler.hpp"

namespace control {

ActionScheduler::ActionScheduler() {}

bool ActionScheduler::addAction(std::function<void()> action, ActionTrigger trigger, double value) {
    // Validate value based on trigger type
    if (value < 0) {
        return false; // Negative values don't make sense for any trigger
    }
    
    // Add the action to our list
    m_scheduledActions.push_back({action, trigger, value, false});
    return true;
}

bool ActionScheduler::addActionAtTimeFromStart(std::function<void()> action, Time t) {
    return addAction(action, ActionTrigger::TIME_FROM_START, to_sec(t));
}

bool ActionScheduler::addActionAtTimeFromEnd(std::function<void()> action, Time t) {
    return addAction(action, ActionTrigger::TIME_FROM_END, to_sec(t));
}

bool ActionScheduler::addActionAtDistanceFromStart(std::function<void()> action, Length d) {
    return addAction(action, ActionTrigger::DISTANCE_FROM_START, to_in(d));
}

bool ActionScheduler::addActionAtDistanceFromEnd(std::function<void()> action, Length d) {
    return addAction(action, ActionTrigger::DISTANCE_FROM_END, to_in(d));
}

void ActionScheduler::clearActions() {
    m_scheduledActions.clear();
}

void ActionScheduler::resetActions() {
    for (auto& action : m_scheduledActions) {
        action.executed = false;
    }
}

void ActionScheduler::processActions(Time currentTime, Time totalTime, Length currentDistance, Length totalDistance) {
    for (auto& action : m_scheduledActions) {
        if (!action.executed) {
            bool shouldExecute = false;
            
            switch (action.trigger) {
                case ActionTrigger::TIME_FROM_START:
                    shouldExecute = to_sec(currentTime) >= action.value;
                    break;
                
                case ActionTrigger::TIME_FROM_END:
                    shouldExecute = to_sec(totalTime - currentTime) <= action.value;
                    break;
                
                case ActionTrigger::DISTANCE_FROM_START:
                    shouldExecute = to_in(currentDistance) >= action.value;
                    break;
                
                case ActionTrigger::DISTANCE_FROM_END:
                    shouldExecute = to_in(totalDistance - currentDistance) <= action.value;
                    break;
            }
            
            if (shouldExecute) {
                action.executed = true;
                action.action(); // Execute the action
            }
        }
    }
}

} // namespace control
