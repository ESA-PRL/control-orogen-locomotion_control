#include "Task.hpp"

#define BEMA 0
#define FRONT 1
#define REAR 2

using namespace locomotion_control;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
  state = NO_COMMAND;
  force_mode = ACKERMAN;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
	state = NO_COMMAND;
  force_mode = ACKERMAN;
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
    // MV: so whenever a joint position is changed it enters wheel walking mode?
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
                (current_motion_command.translation != motion_command.translation) ||
              (current_motion_command.heading.getRad() != motion_command.heading.getRad()))
        {
            //std::cout<<"locomotion_control::Task::motion_commandCallback: new command received..."<<std::endl;
            state=NEW_COMMAND;
            /** Take the new motion command **/
            motion_command = current_motion_command;
        }
    }

    if (state==NEW_COMMAND)
    {

        if (motion_command.rotation==0 && motion_command.translation==0 && motion_command.heading.getRad()==0) 	//! stop command
        {
            std::cout << "stopped rover"<<std::endl;
            locCtrl.setDrivingMode(STOPPED_WHEELS);
            sendCommands();
            mode=STOPPED_WHEELS;
            state=NO_COMMAND;
        }
        else
        {

            // Set force mode depending on the input
            if (motion_command.translation == 42 && motion_command.rotation == 42) {
                force_mode = GENERIC_CRAB;
            }
            else if (motion_command.translation == -42 || motion_command.rotation == -42)
            {
                force_mode = ACKERMAN;
            }

//
//                           ,,,,.         ,,,,...,,,,      ,,,,,,,,,,,        ,,,,
//                           ,,,,,,,      ,,. .  .,, ,,   ,,,  .  *,, ,,     ,,,,,,       ,,,.
//                 ,*        ,,,,,,,,    ,,.       ...,,  ,,        .. ,.   ,,,,,,,,     ,,,,,
//                ,,,,,,     ,,,,,,,,,   ,,.  .     ,.,,  ,,   ,     , ,.  ,,,,,,,,,.  ,,,,,,,.
//                ,,,,,,,,,,,,,,,,,,,,,  .,,         ,,   .,,         ,,   ,,,,,,,,,,,,,,,,,,,.
//                ,,,,,,,,,,,,,,,,,,,,,.   ,,,,,,,,,,       ,,,,,,,,,,     ,,,,,,,,,,,,,,,,,,,
//                 ,,,,,,,,,,,,,,,,,,,,         ,,.            .,,         ,,,,,,,,,,,,,,,,,,,
//                 .,,,,,,,,,,,,,,,,,,          ,,,            ,,,          ,,,,,,,,,,,,,,,,.
//                   ,,,,,,,,,,,,,,,,           .,,,,,,,,,,,,,,,,,           .,,,,,,,,,,,,,.
//                     ,,,,,,,,,,,,        .,,,,,,,,,,,,,,,,,,,,,,,,,,,*       ,,,**,,,,.
//                               ,,,,.   ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, ,,,,,,
//                                ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,.
//                                    ,,,,,,,*,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
//                                   ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, ,,,,,,,,,
//                                   ,,,,,,,,,,,,,*,,,,,,,,,,,,,,,,,,,,,,,,,,,
//                                   ,,,,,,,,,,,,,,,,*, ,,,, ,.,,,,,,,,,,,,,,,.
//                                   ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,*.
//                                  ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,  .,,,.
//                               ,,,, ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,  ,,
//                              ,,,  ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,  *,,,,
//                              ,. ,,,, .,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,   ,,,
//                                ,,,.  ,,,, .,,,,,,,,,,,,,,,,,,,,.        ,,,.   ,,
//                                ,,   .,,,                                 ,,,
//                                ..    ,,                                   ,,
//                                      .,


            // Activate Generic Crabbing Mode in a really ugly way
            if (force_mode == GENERIC_CRAB)
            {

                if (mode!=GENERIC_CRAB)
                {
                  std::cout<<"locomotion_control::Task:: entered generic crab mode" <<std::endl;
                }
                locCtrl.setDrivingMode(GENERIC_CRAB);
                sendCommands();
                mode=GENERIC_CRAB;

                getSteeringPositionReadings(joints_readings, steeringPositionReadings);

                locCtrl.pltfDriveGenericCrab(motion_command.translation, motion_command.heading.getRad(), motion_command.rotation, steeringPositionReadings);
                sendSteeringCommands();
                state=PREP_COMMAND;
            }

            else if (force_mode == ACKERMAN) {

                if (motion_command.heading.getRad() != 0.0)
                {
                    if (mode!=CRAB)
                    {
                        std::cout << "locomotion_control::Task::entered crab mode" <<std::endl;
                    }
                    locCtrl.setDrivingMode(CRAB);
                    sendCommands();
                    mode=CRAB;
                    locCtrl.pltfDriveCrab(motion_command.translation, motion_command.heading.getRad());
                    sendSteeringCommands();
                }
                else if (motion_command.rotation==0 )                     //! straight line command
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
                        std::cout<<"locomotion_control::Task:: entered ackerman mode" <<std::endl;
                    }
                        locCtrl.setDrivingMode(ACKERMAN);
                        sendCommands();
                        mode=ACKERMAN;
                    motion_command.rotation=motion_command.rotation+0.00000001;
                    double vel=motion_command.translation;
                    //! Point to Control set to be always the centre of the rover
                    double PtC[]={0,0};
                    //!Instantaneous center of rotation
                    double CoR[]={0,motion_command.translation/motion_command.rotation};
                    locCtrl.pltfDriveGenericAckerman(vel,CoR,PtC);
                    sendSteeringCommands();

                }
                else if (motion_command.translation==0)             //! point turn command
                {
                    if (mode!=SPOT_TURN)
                    {
                        std::cout<<"locomotion_control::Task:: entered spot turn mode" <<std::endl;
                    }
                    locCtrl.setDrivingMode(SPOT_TURN);
                    sendCommands();
                    mode=SPOT_TURN;
                    locCtrl.pltfDriveSpotTurn(motion_command.rotation);
                    sendSteeringCommands();
                }
                else                                    //! ackerman command
                {
                    if (mode!=ACKERMAN)
                    {
                      std::cout<<"locomotion_control::Task:: entered ackerman mode" <<std::endl;
                    }
                    locCtrl.setDrivingMode(ACKERMAN);
                    sendCommands();
                    mode=ACKERMAN;
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
            //std::cout<<"locomotion_control::Task:: sent command"<<std::endl;
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

    for (int i = 0; i < joints_commands.size(); i++)
    {
        LOG_DEBUG_S << "Joint " << i << " command: Position " << joints_commands[i].position << ", speed " << joints_commands[i].speed;
    }

    // send commands
    joints_commands.time = base::Time::now();
    _joints_commands.write(joints_commands);
}

void Task::sendCommands()
{
	for (size_t i=0;i<locCtrl.commands.size();i++)
	{
    joints_commands[i].position = base::unset<double>();
    joints_commands[i].speed = base::unset<float>();
    joints_commands[i].effort = base::unset<float>();

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
//	joints_commands[COMMAND_WHEEL_WALK_FL].position=-0.68; // 20 deg in rad
//	joints_commands[COMMAND_WHEEL_WALK_FR].position=-0.68;
//	joints_commands[COMMAND_WHEEL_WALK_CL].position=-0.35;
//	joints_commands[COMMAND_WHEEL_WALK_CR].position=-0.35;
//	joints_commands[COMMAND_WHEEL_WALK_BL].position=-0.35;
//	joints_commands[COMMAND_WHEEL_WALK_BR].position=-0.35;
	////////               END OF QUICK FIX                    /////////

    // std::cout << "Sending Commands\n";

    // for (int j = 0; j<12; j++)
    // {
    //     std::cout << "Sending:      " << joints_commands[j].position << "\t" << joints_commands[j].speed << std::endl;
    // }
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
    //locCtrl.commands[COMMAND_WHEEL_STEER_FL].mode=UNSET_COMMAND;


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
    //locCtrl.commands[COMMAND_WHEEL_STEER_FR].mode=UNSET_COMMAND;

    if (locCtrl.commands[COMMAND_WHEEL_STEER_CL].pos>position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_CL].position=position_limit;
    }
    else if (locCtrl.commands[COMMAND_WHEEL_STEER_CL].pos<-position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_CL].position=-position_limit;
    }
    else
    {
        joints_commands[COMMAND_WHEEL_STEER_CL].position=locCtrl.commands[COMMAND_WHEEL_STEER_CL].pos;
    }
    //locCtrl.commands[COMMAND_WHEEL_STEER_CL].mode=UNSET_COMMAND;

    if (locCtrl.commands[COMMAND_WHEEL_STEER_CR].pos>position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_CR].position=position_limit;
    }
    else if (locCtrl.commands[COMMAND_WHEEL_STEER_CR].pos<-position_limit)
    {
        joints_commands[COMMAND_WHEEL_STEER_CR].position=-position_limit;
    }
    else
    {
        joints_commands[COMMAND_WHEEL_STEER_CR].position=locCtrl.commands[COMMAND_WHEEL_STEER_CR].pos;
    }
    //locCtrl.commands[COMMAND_WHEEL_STEER_CR].mode=UNSET_COMMAND;

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
    //locCtrl.commands[COMMAND_WHEEL_STEER_BL].mode=UNSET_COMMAND;

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
    //locCtrl.commands[COMMAND_WHEEL_STEER_BR].mode=UNSET_COMMAND;

}

void Task::sendBemaJoints()
{
    for (int i=0;i<6;i++)
    {
        // TODO: The joint readings are probably shifted as two more steering joints were added??

        // bema_joints[i].position=joints_readings[12+i].position;
        bema_joints[i].position=joints_readings[10+i].position;
        // currentDeployAngles[i]=joints_readings[12+i].position;
        currentDeployAngles[i]=joints_readings[10+i].position;
    }
    _bema_joints.write(bema_joints);
}


bool Task::targetReached()
{
    for (unsigned int i=0;i<joints_readings.size();i++)
    {

        // TODO: Fix the following problem:
        // When there are no joint_readings, they are "nan". Thus, all the following if-checks of the position and velocities are false.
        // Therefore the program says the target has been reached even though there is NO information about the real state of the system available.
        // std::cout << (joints_readings[i].position-joints_commands[i].position) << "\t";
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

void Task::getSteeringPositionReadings(base::samples::Joints joints_readings, double *steeringPositionReadings)
{
    steeringPositionReadings[0] = joints_readings[COMMAND_WHEEL_STEER_FL].position;
    steeringPositionReadings[1] = joints_readings[COMMAND_WHEEL_STEER_FR].position;
    steeringPositionReadings[2] = joints_readings[COMMAND_WHEEL_STEER_CL].position;
    steeringPositionReadings[3] = joints_readings[COMMAND_WHEEL_STEER_CR].position;
    steeringPositionReadings[4] = joints_readings[COMMAND_WHEEL_STEER_BL].position;
    steeringPositionReadings[5] = joints_readings[COMMAND_WHEEL_STEER_BR].position;

    if (std::isnan(steeringPositionReadings[0])) {
        steeringPositionReadings[0] = 0;
        steeringPositionReadings[1] = 0;
        steeringPositionReadings[2] = 0;
        steeringPositionReadings[3] = 0;
        steeringPositionReadings[4] = 0;
        steeringPositionReadings[5] = 0;
        std::cout << "WARNING: locomotion_control/Task.cpp joint_readings is empty." << std::endl;
    }
}
