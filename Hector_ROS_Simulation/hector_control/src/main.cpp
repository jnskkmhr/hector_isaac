#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <thread>

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/CheatIO.h"
#include "../include/FSM/FSM.h"

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

void runFSMController(FSM* _FSMController)
{
    ros::Rate rate(1000); 
    while (running)
    {
        _FSMController->run();
        rate.sleep();
    }
}

int main(int argc, char ** argv)
{

    std::string robot_name = "hector";
    std::cout << "robot name " << robot_name << std::endl;

    ros::init(argc, argv, "hector_control", ros::init_options::AnonymousName);
    ros::Rate rate(1000);

    IOInterface *ioInter;
    ioInter = new CheatIO(robot_name);

    double dt = 0.001;
    Biped biped;

    LegController* legController = new LegController(biped);
    LowlevelCmd* cmd = new LowlevelCmd();
    LowlevelState* state = new LowlevelState();

    std::cout << "start setup " << std::endl;
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);

    stateEstimator->addEstimator<CheaterOrientationEstimator>();   
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();   

    std::cout << "setup state etimator" << std::endl;                                                             

    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_biped = &biped;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;

    FSM* _FSMController = new FSM(_controlData);

    // FSM in a separate thread
    std::thread fsm_thread(runFSMController, _FSMController);    

    signal(SIGINT, ShutDown);
    
    // main thread (ioInter)
    // send low level commands as unitree_legged_msgs::LowCmd copied from "cmd"
    // receive high level state (motor state, imu, position, velocity) and update "state"
    while(running)
    {   
        ioInter->sendRecv(cmd, state);
        rate.sleep();
    }

    // ros shutdown 
    ros::shutdown();
    
    delete _controlData;
    return 0;

}
