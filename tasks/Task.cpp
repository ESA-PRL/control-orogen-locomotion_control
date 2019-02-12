#include "Task.hpp"

#define BEMA 0
#define FRONT 1
#define REAR 2

using namespace locomotion_control;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
    state=NO_COMMAND;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
    state=NO_COMMAND;
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Read configuration **/
    window=_target_window.value();
    position_limit=_position_limit.value();
    velocity_limit=_velocity_limit.value();
    canNodesNames = _canNodesNames.value();

    /** Initialize the locomotion control library **/
    locCtrl.setRoverParams(_rover_parameters);
    locCtrl.commands.resize(canNodesNames.size());
    for (size_t i=0;i<locCtrl.commands.size();i++)
    {
        locCtrl.commands[i].mode=UNSET_COMMAND;
    }

    /** Initialize the joints variables **/
    joints_readings.resize(_number_motors.value());
    joints_commands.resize(canNodesNames.size());
    joints_commands.names = canNodesNames;
    bema_joints.resize(6);

    /** Initial motion commad is NaN **/
    motion_command.translation = base::NaN<double>();
    motion_command.rotation = base::NaN<double>();
    state=NO_COMMAND;
    mode=STOPPED_WHEELS;

    return true;

}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
    base::commands::Motion2D current_motion_command;
    if (_joints_readings.read(joints_readings) == RTT::NewData)
        sendBemaJoints();
    if (_bema_command.read(bema_command) == RTT::NewData)
    {
        if (mode!=WHEEL_WALKING)
        {
            locCtrl.setDrivingMode(WHEEL_WALKING);
            LOG_DEBUG_S << "entered walking mode";
            sendCommands();
            mode=WHEEL_WALKING;
            deploy_mode=BEMA;
        }
        locCtrl.pltfBemaDeploy(bema_command, currentDeployAngles);
        motion_command.translation = base::NaN<double>();
        motion_command.rotation = base::NaN<double>();
        state=PREP_COMMAND;
    }
    if (_walking_command_front.read(bema_command) == RTT::NewData)
    {
        if (mode!=WHEEL_WALKING)
        {
            locCtrl.setDrivingMode(WHEEL_WALKING);
            LOG_DEBUG_S << "entered walking mode";
            sendCommands();
            mode=WHEEL_WALKING;
            deploy_mode=FRONT;
        }
        locCtrl.pltfWalkingDeployFront(bema_command, currentDeployAngles);
        motion_command.translation = base::NaN<double>();
        motion_command.rotation = base::NaN<double>();
        state=PREP_COMMAND;
    }
    if (_walking_command_rear.read(bema_command) == RTT::NewData)
    {
        if (mode!=WHEEL_WALKING)
        {
            locCtrl.setDrivingMode(WHEEL_WALKING);
            LOG_DEBUG_S << "entered walking mode";
            sendCommands();
            mode=WHEEL_WALKING;
            deploy_mode=REAR;
        }
        locCtrl.pltfWalkingDeployRear(bema_command, currentDeployAngles);
        motion_command.translation = base::NaN<double>();
        motion_command.rotation = base::NaN<double>();
        state=PREP_COMMAND;
    }

    /** Read the high level command and send information to the joints **/
    if ( (_motion_command.read(current_motion_command) == RTT::NewData) )
    {

        if ((current_motion_command.rotation != motion_command.rotation) ||
                (current_motion_command.translation != motion_command.translation))
        {
            //std::cout<<"locomotion_control::Task::motion_commandCallback: new command received..."<<std::endl;
            state=NEW_COMMAND;
            /** Take the new motion command **/
            motion_command = current_motion_command;
        }
    }

    if (state==NEW_COMMAND)
    {
        if (motion_command.rotation==0 && motion_command.translation==0)    //! stop command
        {
            LOG_DEBUG_S << "stopped rover";
            locCtrl.setDrivingMode(STOPPED_WHEELS);
            sendCommands();
            mode=STOPPED_WHEELS;
            state=NO_COMMAND;
        }
        else
        {
            if (motion_command.rotation==0)  //! straight line command
                // E.B: I changed the straigth line to do Ackerman with an almost "infinite" arc.
            {
                /*if (mode!=STRAIGHT_LINE)
                  {
                  locCtrl.setDrivingMode(STRAIGHT_LINE);
                  std::cout<<"locomotion_control::Task:: entered straight line mode" <<std::endl;
                  sendCommands();
                  mode=STRAIGHT_LINE;
                  }
                  locCtrl.pltfDriveStraightVelocity(motion_command.translation);*/

                // M.A: Enabled staight line command for egrees testing

                if (mode!=ACKERMAN)
                {
                    locCtrl.setDrivingMode(ACKERMAN);
                    LOG_DEBUG_S << "entered ackermann mode";
                    sendCommands();
                    mode=ACKERMAN;
                }
                motion_command.rotation=motion_command.rotation+0.00000001;
                double vel=motion_command.translation;
                //! Point to Control set to be always the centre of the rover
                double PtC[]={0,0};
                //!Instantaneous center of rotation
                double CoR[]={0,motion_command.translation/motion_command.rotation};
                locCtrl.pltfDriveGenericAckerman(vel,CoR,PtC);
                sendSteeringCommands();

            }
            else if (motion_command.translation==0)  //! point turn command
            {
                if (mode!=SPOT_TURN)
                {
                    locCtrl.setDrivingMode(SPOT_TURN);
                    LOG_DEBUG_S << "entered spot turn mode";
                    sendCommands();
                    mode=SPOT_TURN;
                }
                locCtrl.pltfDriveSpotTurn(motion_command.rotation);
                sendSteeringCommands();
            }
            else  //! ackerman command
            {
                if (mode!=ACKERMAN)
                {
                    locCtrl.setDrivingMode(ACKERMAN);
                    LOG_DEBUG_S << "entered ackerman mode";
                    sendCommands();
                    mode=ACKERMAN;
                }
                double vel=motion_command.translation;
                //! Point to Control set to be always the centre of the rover
                double PtC[]={0,0};
                //!Instantaneous center of rotation
                double CoR[]={0,motion_command.translation/motion_command.rotation};
                locCtrl.pltfDriveGenericAckerman(vel,CoR,PtC);
                sendSteeringCommands();
            }
            state=PREP_COMMAND;
        }
    }

    if (state==PREP_COMMAND)
    {
        if(targetReached())
        {
            //LOG_DEBUG_S << "target reached";
            state=EXEC_COMMAND;
        }
    }

    if(state==EXEC_COMMAND)
    {
        if (mode!=WHEEL_WALKING)
        {
            sendCommands();
            //LOG_DEBUG_S << "sent command";
            state=NO_COMMAND;
        }
        else
        {
            if (deploy_mode==BEMA)
            {
                locCtrl.pltfBemaDeploy(bema_command, currentDeployAngles);
            }
            else if (deploy_mode==FRONT)
            {
                locCtrl.pltfWalkingDeployFront(bema_command, currentDeployAngles);
            }
            else if (deploy_mode==REAR)
            {
                locCtrl.pltfWalkingDeployRear(bema_command, currentDeployAngles);
            }
            sendCommands();
        }
    }
}

void Task::sendCommands()
{
    for (size_t i=0;i<locCtrl.commands.size();i++)
    {
        switch (locCtrl.commands[i].mode)
        {
            case UNSET_COMMAND:
                if (joints_commands[i].hasPosition())
                    joints_commands[i].position = base::unset<double>();
                if (joints_commands[i].hasSpeed())
                    joints_commands[i].speed = base::unset<float>();
                if (joints_commands[i].hasEffort())
                    joints_commands[i].effort = base::unset<float>();

                break;
            case MODE_POSITION:
                if (locCtrl.commands[i].pos>position_limit)
                {
                    joints_commands[i].position = position_limit;
                }
                else if (locCtrl.commands[i].pos<-position_limit)
                {
                    joints_commands[i].position = -position_limit;
                }
                else
                {
                    joints_commands[i].position = locCtrl.commands[i].pos;
                }
                locCtrl.commands[i].mode=UNSET_COMMAND;
                break;
            case MODE_SPEED:
                if (locCtrl.commands[i].vel>velocity_limit)
                {
                    joints_commands[i].speed = velocity_limit;
                }
                else if (locCtrl.commands[i].vel<-velocity_limit)
                {
                    joints_commands[i].speed = -velocity_limit;
                }
                else
                {
                    joints_commands[i].speed = locCtrl.commands[i].vel;
                }
                locCtrl.commands[i].mode=UNSET_COMMAND;
                break;
            default:
                break;
        }
    }

    ///  QUICK FIX TO COMMAND WHEEL WALKING MOTORS TO FIXED POSITION ///
    //  joints_commands[COMMAND_WHEEL_WALK_FL].position=-0.68; // 20 deg in rad
    //  joints_commands[COMMAND_WHEEL_WALK_FR].position=-0.68;
    //  joints_commands[COMMAND_WHEEL_WALK_CL].position=-0.35;
    //  joints_commands[COMMAND_WHEEL_WALK_CR].position=-0.35;
    //  joints_commands[COMMAND_WHEEL_WALK_BL].position=-0.35;
    //  joints_commands[COMMAND_WHEEL_WALK_BR].position=-0.35;
    ////////               END OF QUICK FIX                    /////////

    joints_commands.time = base::Time::now();
    _joints_commands.write(joints_commands);
}

void Task::sendSteeringCommands()
{
    /*
       for (size_t i=0;i<locCtrl.commands.size();i++)
       {
       if (joints_commands[i].hasPosition())
       joints_commands[i].position = base::unset<double>();
       if (joints_commands[i].hasSpeed())
       joints_commands[i].speed = base::unset<float>();
       if (joints_commands[i].hasEffort())
       joints_commands[i].effort = base::unset<float>();
       }
     */

    if (locCtrl.commands[COMMAND_WHEEL_STEER_FL].pos>position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_FL].position=position_limit;
    }
    else if (locCtrl.commands[COMMAND_WHEEL_STEER_FL].pos<-position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_FL].position=-position_limit;
    }
    else
    {
        joints_commands[COMMAND_WHEEL_STEER_FL].position=locCtrl.commands[COMMAND_WHEEL_STEER_FL].pos;
    }
    locCtrl.commands[COMMAND_WHEEL_STEER_FL].mode=UNSET_COMMAND;

    if (locCtrl.commands[COMMAND_WHEEL_STEER_FR].pos>position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_FR].position=position_limit;
    }
    else if (locCtrl.commands[COMMAND_WHEEL_STEER_FR].pos<-position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_FR].position=-position_limit;
    }
    else
    {
        joints_commands[COMMAND_WHEEL_STEER_FR].position=locCtrl.commands[COMMAND_WHEEL_STEER_FR].pos;
    }
    locCtrl.commands[COMMAND_WHEEL_STEER_FR].mode=UNSET_COMMAND;

    if (locCtrl.commands[COMMAND_WHEEL_STEER_BL].pos>position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_BL].position=position_limit;
    }
    else if (locCtrl.commands[COMMAND_WHEEL_STEER_BL].pos<-position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_BL].position=-position_limit;
    }
    else
    {
        joints_commands[COMMAND_WHEEL_STEER_BL].position=locCtrl.commands[COMMAND_WHEEL_STEER_BL].pos;
    }
    locCtrl.commands[COMMAND_WHEEL_STEER_BL].mode=UNSET_COMMAND;

    if (locCtrl.commands[COMMAND_WHEEL_STEER_BR].pos>position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_BR].position=position_limit;
    }
    else if (locCtrl.commands[COMMAND_WHEEL_STEER_BR].pos<-position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_BR].position=-position_limit;
    }
    else
    {
        joints_commands[COMMAND_WHEEL_STEER_BR].position=locCtrl.commands[COMMAND_WHEEL_STEER_BR].pos;
    }
    locCtrl.commands[COMMAND_WHEEL_STEER_BR].mode=UNSET_COMMAND;

    joints_commands.time = base::Time::now();
    _joints_commands.write(joints_commands);
}

void Task::sendBemaJoints()
{
    for (int i=0;i<6;i++)
    {
        bema_joints[i].position=joints_readings[10+i].position;
        currentDeployAngles[i]=joints_readings[10+i].position;
    }
    _bema_joints.write(bema_joints);
}


bool Task::targetReached()
{
    for (unsigned int i=0;i<joints_readings.size();i++)
    {
        switch (joints_commands[i].getMode())
        {
            case base::JointState::UNSET:
                //! Means no command was send for this motor. Don't need to check target.
                break;
            case base::JointState::POSITION:
                if (((joints_readings[i].position-joints_commands[i].position)>window) || ((joints_commands[i].position-joints_readings[i].position)>window))
                {
                    //std::cout<<"locomotion_control::Task::targetReached " << i << " : Target position is: "<< joints_commands[i].position << " and current position is: " << joints_readings[i].position <<std::endl;
                    return false;
                }
                break;
            case base::JointState::SPEED:
                if (((joints_readings[i].speed-joints_commands[i].speed)>window) || ((joints_commands[i].speed-joints_readings[i].speed)>window))
                {
                    //std::cout<<"locomotion_control::Task::targetReached " << i << " : Target velocity is: "<< joints_commands[i].speed << " and current velocity is: " << joints_readings[i].speed <<std::endl;
                    return false;
                }
                break;
            case base::JointState::RAW:
                //! Not implemented
                break;
            default:
                break;
        }
    }
    //std::cout << "target reached!" << std::endl;
    return true;
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
