/*
 * Copyright (c) 2018, Youming Qin
 * DJI corp, RomoMaster
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class StateMachine
{
public:
  StateMachine();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_y_, linear_, angular_, deadman_axis_;
  double l_scale_, y_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

StateMachine::StateMachine():
  ph_("~"),
{

  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::publish, this));
}

void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;

  vel.angular.z = a_scale_*joy->axes[angular_];

  vel.linear.x = l_scale_*joy->axes[linear_];
  vel.linear.y = y_scale_*joy->axes[linear_y_];
  
  last_published_ = vel;
  // deadman_pressed_ = joy->buttons[deadman_axis_];
}

void TurtlebotTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  vel_pub_.publish(last_published_);
  zero_twist_published_=false;
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_machine");
  StateMachine state_machine;

  ros::spin();
}
