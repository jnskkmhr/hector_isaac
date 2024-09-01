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
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
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
        ros::Subscriber _servo_sub[10], _state_sub;
        ros::Publisher _servo_pub[10];
        unitree_legged_msgs::LowCmd _lowCmd;
        unitree_legged_msgs::HighState _highState;
        ros::AsyncSpinner _subSpinner; 


        std::string _robot_name;
        void initRecv(); // initialize subscribers
        void initSend(); // initialize publishers
    
        void StateCallback(const nav_msgs::Odometry& msg);

        void LhipCallback(const unitree_legged_msgs::MotorState& msg);
        void Lhip2Callback(const unitree_legged_msgs::MotorState& msg);
        void LthighCallback(const unitree_legged_msgs::MotorState& msg);
        void LcalfCallback(const unitree_legged_msgs::MotorState& msg);
        void LtoeCallback(const unitree_legged_msgs::MotorState& msg);
        void RhipCallback(const unitree_legged_msgs::MotorState& msg);
        void Rhip2Callback(const unitree_legged_msgs::MotorState& msg);
        void RthighCallback(const unitree_legged_msgs::MotorState& msg);
        void RcalfCallback(const unitree_legged_msgs::MotorState& msg);
        void RtoeCallback(const unitree_legged_msgs::MotorState& msg);

};   

#endif