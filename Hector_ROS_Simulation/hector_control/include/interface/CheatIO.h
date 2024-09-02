#ifndef CHEATIO_H
#define CHEATIO_H

#include "ros/ros.h"
#include <ros/time.h>
#include<boost/array.hpp>
#include "IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "unitree_legged_msgs/HighState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <string>

class CheatIO : public IOInterface
{
    public:
        CheatIO(std::string robot_name);
        ~CheatIO();
        void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    private:
        void sendCmd(const LowlevelCmd *cmd);
        void recvState(LowlevelState *state);
        ros::NodeHandle _nm;
        ros::Subscriber _state_sub, _jstate_sub;
        ros::Publisher _jstate_pub;
        unitree_legged_msgs::HighState _highState;
        sensor_msgs::JointState _jointCmd;
        ros::AsyncSpinner _subSpinner; 


        std::string _robot_name;
        void initRecv(); // initialize subscribers
        void initSend(); // initialize publishers
    
        void StateCallback(const nav_msgs::Odometry& msg);
        void JstateCallback(const sensor_msgs::JointState& msg);
};   

#endif