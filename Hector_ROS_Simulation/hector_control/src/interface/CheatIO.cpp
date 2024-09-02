#include "../../include/interface/CheatIO.h"
#include "../../include/interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

inline void RosShutDown(int sig){
    ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

CheatIO::CheatIO(std::string robot_name):IOInterface(), _subSpinner(1)
{
    // int argc; char **argv;
    // ros::init(argc, argv, "unitree_gazebo_servo");
    std::cout << "The control interface for Isaac Sim" << std::endl;
    _robot_name = robot_name;

    // start subscriber
    initRecv();
    _subSpinner.start();
    usleep(3000);     //wait for subscribers start

    // initialize publisher
    initSend();   
    signal(SIGINT, RosShutDown);
    cmdPanel = new KeyBoard();

    // initialize msg
    std::vector<std::string> joint_names = {"L_hip", "L_hip2", "L_thigh", "L_calf", "L_toe", "R_hip", "R_hip2", "R_thigh", "R_calf", "R_toe"};
    _jointCmd.name = joint_names;
    _jointCmd.position.resize(joint_names.size(), 0.0);
    _jointCmd.velocity.resize(joint_names.size(), 0.0);
    _jointCmd.effort.resize(joint_names.size(), 0.0);
}

CheatIO::~CheatIO()
{
    ros::shutdown();
}

void CheatIO::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    sendCmd(cmd);
    recvState(state);
    cmdPanel->updateVelCmd(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void CheatIO::sendCmd(const LowlevelCmd *cmd)
{
    for(int i = 0; i < 10; i++){
        // set value of jointstate
        _jointCmd.header.stamp = ros::Time::now();
        _jointCmd.position[i] = cmd->motorCmd[i].q;
        _jointCmd.velocity[i] = cmd->motorCmd[i].dq;
        _jointCmd.effort[i] = cmd->motorCmd[i].tau;
    }
    _jstate_pub.publish(_jointCmd);

}

void CheatIO::recvState(LowlevelState *state)
{
    for(int i = 0; i < 10; i++)
    {
        state->motorState[i].q = _highState.motorState[i].q;
        state->motorState[i].dq = _highState.motorState[i].dq;
        state->motorState[i].tauEst = _highState.motorState[i].tauEst;
    }
    for(int i = 0; i < 3; i++){
        state->imu.quaternion[i] = _highState.imu.quaternion[i];
        state->imu.gyroscope[i] = _highState.imu.gyroscope[i];
        state->position[i] = _highState.position[i];
        state->vWorld[i] = _highState.velocity[i];
    }
    state->imu.quaternion[3] = _highState.imu.quaternion[3];
}

void CheatIO::initSend(){
    _jstate_pub = _nm.advertise<sensor_msgs::JointState>("/" + _robot_name + "/llcontroller/command", 1);
}

// gazebo controller subscribes to these messagess
void CheatIO::initRecv(){
    _state_sub = _nm.subscribe("/" + _robot_name + "/Odometry", 1, &CheatIO::StateCallback, this);
    _jstate_sub = _nm.subscribe("/" + _robot_name + "/llcontroller/state", 1, &CheatIO::JstateCallback, this);
}

void CheatIO::StateCallback(const nav_msgs::Odometry& msg)
{
    // subscribe to /hector/Odometry (nav_msgs::Odometry) and plug this into the highState

    _highState.position[0] = msg.pose.pose.position.x;
    _highState.position[1] = msg.pose.pose.position.y;
    _highState.position[2] = msg.pose.pose.position.z;

    _highState.velocity[0] = msg.twist.twist.linear.x;
    _highState.velocity[1] = msg.twist.twist.linear.y;
    _highState.velocity[2] = msg.twist.twist.linear.z;

    _highState.imu.quaternion[0] = msg.pose.pose.orientation.w;
    _highState.imu.quaternion[1] = msg.pose.pose.orientation.x;
    _highState.imu.quaternion[2] = msg.pose.pose.orientation.y;
    _highState.imu.quaternion[3] = msg.pose.pose.orientation.z;

    _highState.imu.gyroscope[0] = msg.twist.twist.angular.x;
    _highState.imu.gyroscope[1] = msg.twist.twist.angular.y;
    _highState.imu.gyroscope[2] = msg.twist.twist.angular.z;
}

void CheatIO::JstateCallback(const sensor_msgs::JointState& msg)
{
    // subscribe to /hector/llcontroller/state (sensor_msgs::JointState) and plug this into the highState
    for(int i = 0; i < 10; i++){
        _highState.motorState[i].mode = 0X0A; // alwasy set it to 0X0A
        _highState.motorState[i].q = msg.position[i];
        _highState.motorState[i].dq = msg.velocity[i];
        _highState.motorState[i].tauEst = msg.effort[i];
    }
}