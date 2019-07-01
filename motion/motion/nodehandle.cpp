#include "nodehandle.h"

NodeHandle::NodeHandle()
{
    shoot_sub = nh.subscribe("/motion/tb3/shoot", 1, &NodeHandle::shootcall, this);
    arm_sub = nh.subscribe("/motion/tb3/arm", 1, &NodeHandle::armcall, this);
    reset_sub = nh.subscribe("/motion/reset", 1, &NodeHandle::resetcall, this);
    cmdvel_sub = nh.subscribe("/motion/cmd_vel", 1, &NodeHandle::cmdvelcall, this);
    
    shoot_pub = nh.advertise<std_msgs::Empty>("/tb3/shoot", 1);
    arm_pub = nh.advertise<std_msgs::Int32>("/tb3/arm", 1);
    reset_pub = nh.advertise<std_msgs::Empty>("/reset", 1);
    cmdvel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}
NodeHandle::~NodeHandle()
{
}
void NodeHandle::shootcall(const std_msgs::Empty msg)
{
    //std::cout<<"shoot\n";
    shoot_pub.publish(msg);
}
void NodeHandle::resetcall(const std_msgs::Empty msg)
{
    //std::cout<<"reset\n";
    reset_pub.publish(msg);
}
void NodeHandle::cmdvelcall(const geometry_msgs::Twist msg)
{
    //std::cout<<"move\n";
    cmdvel_pub.publish(msg);
}
void NodeHandle::armcall(const std_msgs::Int32 msg)
{
    //std::cout<<"arm\n";
    arm_pub.publish(msg);
}

