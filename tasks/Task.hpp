#pragma once

#include <math.h>
#include <base-logging/Logging.hpp>
#include <base-logging/Logging.hpp>
#include <base/commands/Joints.hpp>
#include <base/commands/Motion2D.hpp>
#include <base/samples/Joints.hpp>
#include <vector>
#include "locomotion_control/LocomotionControl.h"
#include "locomotion_control/TaskBase.hpp"

namespace locomotion_control
{

enum CommandingState
{
    NO_COMMAND,
    NEW_COMMAND,
    PREP_COMMAND,
    EXEC_COMMAND
};

class Task : public TaskBase
{
    friend class TaskBase;

  protected:
    // Configuration Variables
    double window;
    double position_limit;
    double velocity_limit;

    // Order of CAN Nodes
    std::vector<std::string> canNodesNames;

    LocomotionControl locCtrl;
    CommandingState state;
    PltfDrivingMode mode;
    double currentDeployAngles[6];
    int deploy_mode;
    double steeringPositionReadings[6];

    double linearVelocity;   // [m/s] of rover in respect to rover inertial frame
    double headingAngle;     // [rad] of rover in respect to rover intertial frame
    double angularVelocity;  // [rad/s] of rover in respect to rover inertial frame

    // Input Variables
    base::commands::Motion2D motion_command;
    double bema_command;
    double walking_command;
    base::samples::Joints joints_readings;

    // Output Variables
    base::commands::Joints joints_commands;
    base::samples::Joints bema_joints;

  public:
    Task(std::string const& name = "locomotion_control::Task",
         TaskCore::TaskState initial_state = Stopped);
    Task(std::string const& name,
         RTT::ExecutionEngine* engine,
         TaskCore::TaskState initial_state = Stopped);
    ~Task();
    void sendCommands();
    void sendSteeringCommands();
    void sendBemaJoints();
    bool targetReached();
    void getSteeringPositionReadings(base::samples::Joints joints_readings,
                                     double* steeringPositionReadings);
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}
