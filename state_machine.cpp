/*
 * Copyright (c) 2018, Youming Qin
 * DJI Corp, RoboMaster, All rights reserved
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
//#include <nav_msgs/Odometry.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "messages/Odometry.h"
#include "ros/console.h"
//
static float new_x;
static float new_y;

void publishSpeed(ros::Publisher vel_pub_, float x, float y);
float getDistance(float last_position[2]);
void OdomCB(const messages::Odometry::ConstPtr &msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_rosmachine");

    enum MachineState{initialize1,x1y0,initialize2,x0y1,stop};
    MachineState state= initialize1;

    double distance = 0;
    float last_position[2];
    ros::NodeHandle ph_, nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber odom_sub_;


    vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    odom_sub_ = nh_.subscribe<messages::Odometry>("/odom", 1, OdomCB);


    while(ros::ok()){
C        switch (state)
        {
            case initialize1:
                //Initialize starting position
                ros::spinOnce();
                last_position[0]=new_x;
                last_position[1]=new_y;
                state = x1y0;
                break;

            case x1y0:
                distance = 1.7;
                if (getDistance(last_position)<=distance) {
                    publishSpeed(vel_pub_, 1, 0);//keep going forward at speed of 0.5
                }
                else
                    state=initialize2;
                ros::spinOnce();
                break;
            case initialize2:
                ros::spinOnce();
                last_position[0]=new_x;
                last_position[1]=new_y;
                state = x0y1;
                break;
            case x0y1:
                distance = 1.7;
                if (getDistance(last_position)<=distance) {
                    publishSpeed(vel_pub_, 0, 1);//keep going forward at speed of 0.5
                }
                else
                    state=stop;
                ros::spinOnce();
                break;
            case stop:
                publishSpeed(vel_pub_, 0, 0);
                break;
        }


    }
}


/**
 * \brief This function will publish a speed with
 * \pre vel_pub_ -- a predefined Publisher
 */
void publishSpeed(ros::Publisher vel_pub_, float x, float y) {
    geometry_msgs::Twist vel;
    vel.linear.y = y;
    vel.linear.x = x;
    std::cout << "pub cmd_vel: x["<<x<<"]    y["<<y<<"]"<< std::endl;
    vel_pub_.publish(vel);
}


/**
 * \brief This function calculate the distance traveled from the "last_position"
 * \param float last_position[0] -- the x location
 *        float last_position[1] -- the y location
 */
float getDistance(float last_position[2]) {
    return sqrt((new_x - last_position[0])*(new_x - last_position[0]) + (new_y - last_position[1])*(new_y - last_position[1]));
}


/**
 * \brief Odometry callback Interrupt service routine
 * This function will be excuted once the "ROS::spin()" or "ROS::spinOnce()" is called
 * It will update the odom readings and store them into new_x and new_y
 */
void OdomCB(const messages::Odometry::ConstPtr &msg) {
  new_x = msg->pose.pose.position.x;
  new_y = msg->pose.pose.position.y;
}
