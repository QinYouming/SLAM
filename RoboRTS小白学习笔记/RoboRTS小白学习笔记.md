# RoboRTS小白学习笔记

Copyright (c) 2018, DJI Corp, RoboMaster, All rights reserved

## 简介

RoboRTS项目是基于ROS（Robot Operating System）实现的。为了现场调试便利，首先要做的是配置网络。配置完成后我们即可用一台装有linux系统的笔记本（主机）通过路由器远程与车载电脑（从机）通讯。



## 准备

### 知识储备

本文档虽然针对小白，但是推荐读者把ROS官方提供了的教程中的beginner level过一遍，从而对ROS中基础概念和定义有大致的了解：

> http://wiki.ros.org/ROS/Tutorials



### 配置网络

#### 1. 确保两台电脑在统一网络

将两台电脑接入同一WI-FI。在terminal中通过ping的方式验证，并记两台电脑IP地址：

```
#在主机上输入
$ ping [从机hostname]
```

```
#在从机上输入
$ ping [主机hostname]
```

#### 2. 在host文件填写

分别在主机和从机上打开此文件

```
$ sudo vim /etc/hosts
```

分别在主机和从机上将另外一台电脑的IP和hostname加入文件。例：

```
$ 192.168.1.1   [主机hostname]
```

*注：如不知道本机的hostname可在terminal里输入hostname查询*

#### 3. 配置从机

在从机上输入如下指令：

```
$ export ROS_HOSTNAME=[从机hostname]
$ roscore
```

#### 4. 配置主机

在主机上输入如下指令：

```
$ export ROS_HOSTNAME=[主机hostname]
$ export ROS_MASTER_URI=http://[从机hostname]:11311
```

#### 5. 测试网络链接

```
$ rostopic list
```

此时在主机和从机上的terminal里分别查看rostopic list应该可以看到同样的输出

```
	/rosout
	/rosout_agg
```

如果验证完成，两台电脑的网络链接配置已经完成。

### 远程操作

搭建好网络后，ssh可以远程在从机中输入指令。也就是说，你可以把战车上机载电脑上连接的显示器，键盘鼠标都拔了照样可以控制它。具体操作如下：

```
$ ssh [从机id]@[从机IP]
```



## 程序实例

RoboRTS是由一个一个node（ROS里的程序，通常写在.cpp文件里）组成的。在ROS的世界里，这些node使用rostopic方式沟通。你可以把ROS想象成Youtube，这些node就是up主，他们会制造一些话题（rostopic）。当up主发布（pub）新的视频的时候，订阅的用户（subscribe）就能收到相对应的消息，而没有订阅的用户就收不到。

本章的目的是通过自己写一个node，读取xbox游戏手柄和键盘的读数来在 /cmd_vel 这个rostopic上面发消息，以达到控制小车运动的目的。

### 准备

#### 自启程序bashrc

每次你打开一个新的terminal都需要其实都需要setup一遍主机。然而bashrc这个文件在每一次开terminal的时候都会自动运行一遍，因此我们把下面这两行代码加入bashrc文件后就不需麻烦了。首先先启动bashrc文件：

```
$ sudo vim ~/.bashrc
```

然后在主机文件的最后加入以下两行代码：

```
export ROS_MASTER_URI=http://manifold2:11311
source [主机RoboRTS所在路经]/RoboRTS_ros/devel/setup.bash
```

*如用其他terminal，如zsh，请source setup.zsh*

#### 安装RoboRTS包

建立一个ROS workspace，在src目录下输入以下command：
```
$ git clone https://github.com/RoboMaster/RoboRTS.git
```

#### 安装teleop包

为了实现用键盘和遥杆控制战车的目的，我们需要用到ROS官方tutorial里控制小乌龟程序的那个包。在terminal里输入以下命令安装teleop包：

    $ sudo apt install ros-kinetic-turtlebot-teleop

### 驱动地盘测试

做完了准备工作，我们先通过发一个简单速度指令来测试地盘机械和电控连接的可靠性。

在RoboRTS里 /cmd_vel 这个topic是控制地盘速度的。一旦有人pub速度信息，底盘这个subscriber就会动。首先确保上一章的网络配置已经完成，在从机上启动roscore，并输入以下指令打开从机和战车的下位机（单片机）的通讯：

```
$ rosrun roborts serial_com_node
```

rosrun 这个指令是让ros跑node，roborts是这个node所在的package（node的上一层封装）。serial_com_node做的就是开启从机和战车上下位机（单片机）的通讯。

把战车通上电，并且确保战车在开阔地带，或者把战车地盘架起来。开启遥控器，左边拨杆放在中间（空档），右边拨杆放在最下面（机载电脑控制模式）。输入以下数据地盘就开跑了：

```
 rostopic pub /cmd_vel -r 50 geometry_msgs/Twist "linear:
  x: 0.5 
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

此命令是在/cmd_vel这个topic上发布（pub）一个类型为geometry_msgs/Twist的参数。-r 50的意思是以50hz的频率重复发送此指令。linear x和y分别控制麦克纳姆轮小车在x方向和y方向的速度。速度可以为负，但是建议速度不要超过3。angular参数的z控制的是转弯的角速度，也可以为正或负，但建议不要超过2.5。其余参数没有用。

*注：如主机提示 “ERROR: Unable to communicate with master!” 请重新在此terminal里配置一遍主机。*

输入以下指令，也可以查看（echo）机器收到的指令：

```
$ rostopic echo /cmd_vel
```

如果硬件连接没有问题，现在战车应该在以0.5的速度在往前跑。



### 手柄控车node （C++）

假设你手上有一个可连接到电脑上的游戏手柄，那我们可以在ROS环境下运行一个node，实现通过手柄去控制战车。详细地说，这个node的作用是去subscribe手柄的按键信息，再根据这些信息publish相对应的速度信息（cmd_vel），使得战车上的电脑可以接收到速度信息。

这个node的代码是来源于Turtlebot包里的turtlebot_joy.cpp，在这里我们稍作了修改，加入了左右平移的功能。
先贴完整代码：
（如果想复现这个功能，可以在RobotRTS包内的/tools目录下新建一个名为joy_teleop.cpp的文档并将以下代码复制到文档中。）
```c++
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class TurtlebotTeleop
{
public:
  TurtlebotTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_y_, linear_, angular_, deadman_axis_;
  double l_scale_, y_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
joy
  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

TurtlebotTeleop::TurtlebotTeleop():
  ph_("~"),
  linear_(4), // Up/Down Axis stick right
  linear_y_(3), //  Left/Right Axis stick right
  angular_(0), //  Left/Right Axis stick left
  l_scale_(2.5),
  y_scale_(2.5),
  a_scale_(3)
{

  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);
joy
  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::publish, this));
}

void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;

  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  vel.linear.y = y_scale_*joy->axes[linear_y_];
  
  last_published_ = vel;
}joy

void TurtlebotTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  vel_pub_.publish(last_published_);
  zero_twist_published_=false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_teleop");
  TurtlebotTeleop turtlebot_teleop;

  ros::spin();
}

```

#### 需要关注的代码

创建Publisher和Subscriber
```
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
```

定义Publisher将发布”cmd_vel"话题;
定义Subscriber将关注"joy“话题。
```
  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);
```

将手柄按键信息赋予vel （geometry_msgs::Twist)
```
void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;

  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.linear.x = l_scale_*joy->axes[linear_];
  vel.linear.y = y_scale_*joy->axes[linear_y_];
  
  last_published_ = vel;
}
```

Publisher发布cmd_vel信息
```
  vel_pub_.publish(last_published_);
```

添加了新文档后需要更新CMakelists并重新编译

更新CMakelists:
```
add_executable(joy_teleop tools/joy_teleop.cpp)
target_link_libraries(joy_teleop ${catkin_LIBRARIES})
```

返回到workspace最上级目录并重新编译：
```
$ catkin_make
$ source devel/setup.bash
```

连接好游戏手柄至电脑后，在terminal中跑这个node即可
```
$ rosrun roborts joy_teleop
```


### 键盘控车node （python）

这个node的代码是来源于Turtlebot包里的turtlebot_teleop_key （python），在这里我们稍作了修改，加入了左右平移的功能。
先贴完整代码：
（如果想复现这个功能，可以在RobotRTS包内的/tools目录下新建一个名为key_teleop的文档并将以下代码复制到文档中。）

```python
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
---------------------------
Moving around:
   foward/backward : w/s
   shift left/right :a/d
   spin left/right : j/k

u/m : increase/decrease max speeds by 10%
i/, : increase/decrease only linear speed by 10%
o/. : increase/decrease only angular speed by 10%
space key : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0),  # index: 0-forward/backward   1-spinning left/right   2-shifting left/right
        's':(-1,0,0),
        'a':(0,0,1),
        'd':(0,0,-1),
        'j':(0,1,0),
        'k':(0,-1,0),
           }

speedBindings={
        'u':(1.1,1.1),
        'm':(.9,.9),
        'i':(1.1,1),
        ',':(.9,1),
        'o':(1,1.1),
        '.':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
y_speed = .2
turn = 1

def vels(speed,y_speed,turn):
    return "currently:\tx_speed %s \ty_speed %s \tturn %s " % (speed,y_speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    x = 0
    y = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_yspeed = 0
    target_turn = 0
    control_speed = 0
    control_yspeed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed,y_speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][2]  # added to allow car's shifting left and right
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                y_speed = y_speed * speedBindings[key][0]  
                turn = turn * speedBindings[key][1]
                count = 0

                print(vels(speed,y_speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ':
                x = 0
                y = 0
                th = 0
                control_speed = 0
                control_yspeed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    y = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_yspeed = y_speed * y
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_yspeed > control_yspeed:
                control_yspeed = min( target_yspeed, control_yspeed + 0.02 )
            elif target_yspeed < control_yspeed:
                control_yspeed = max( target_yspeed, control_yspeed - 0.02 )
            else:
                control_yspeed = target_yspeed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = control_yspeed; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

```

#### 需要关注的代码

获取键盘信息
```
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
```

创建Publisher并发布“cmd_vel"话题
```
    rospy.init_node('turtlebot_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
```

创建twist（message class Twist），
并将速度信息赋予twist，
最后发布twist
```
            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = control_yspeed; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)
```

添加新python文档后无需改动CMakelists，
重新编译后，直接跑以下command即可：
```
$ rosrun roborts key_teleop
```

如果跑上面这个command时遇到以下问题：
```
[rosrun] Couldn't find executable named key_teleop below <the destination of where you put your key_teleop file>
[rosrun] Found the following, but they're either not files,
[rosrun] or not executable:
[rosrun]   <the destination of where you put your key_teleop file>
```

可以尝试以下解决方案：

到key_teleop文档所在的目录下，输入一下command：
```
$ chmod +x key_teleop
```



### 状态机 State Machine

本章的目的是实现一个开环的状态机，使得小车能前进指定距离，刹车，再平移一段指定距离。在tools的simulator文件夹下新建一个名为state_machine.cpp的文件，并再CmakeList里添加相关executable。state_machine.cpp的代码如下：

```c++
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


    vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 20, true);
    odom_sub_ = nh_.subscribe<messages::Odometry>("/odom", 1, OdomCB);


    while(ros::ok()){
        switch (state)
        {
            case initialize1:
                //Initialize starting position
		sleep(1);//wait for the robot to initialize, or odom will not be accurate
                ros::spinOnce();
                last_position[0]=new_x;
                last_position[1]=new_y;
                state = x1y0;
                std::cout << "initialize1" << std::endl;
                break;

            case x1y0:
                distance = 1.7;
                if (getDistance(last_position)<=distance) {
                    publishSpeed(vel_pub_, 0.5, 0);//keep going forward at speed of 0.5
		    std::cout<<"Moving Forward         " << std::endl;
		}
                else {
                    std::cout << "new_x: " <<new_x<< std::endl;
                    std::cout << "new_y: " <<new_x<< std::endl;
                    std::cout << "last_position0: " <<last_position[0]<< std::endl;
                    std::cout << "last_position1: " <<last_position[1]<< std::endl;
                    state=initialize2;
                }
                ros::spinOnce();
                break;
            case initialize2:
                std::cout << "initialize2" << std::endl;
                ros::spinOnce();
                last_position[0]=new_x;
                last_position[1]=new_y;

                state = x0y1;
                break;
            case x0y1:
                distance = 1.7;
                if (getDistance(last_position)<=distance) {
                    publishSpeed(vel_pub_, 0, 0.5);//keep going forward at speed of 0.5
		    std::cout<<"Moving Left           ";
		}
                else
                    state=stop;
                ros::spinOnce();
                break;
            case stop:
                    std::cout<<"Stop              "<< std::endl;
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
    //std::cout << "            x["<<x<<"]    y["<<y<<"]"<< std::endl;
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
```

#### 需要关注的代码

此程序与两个rostopic相关：

    vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    odom_sub_ = nh_.subscribe<messages::Odometry>("/odom", 1, OdomCB);
odom数据是从3508电机的码盘获得的，通过对速度的积分会反馈出来一个行走的距离的数据。这个数据在每次spinOnce() 之后都会更新。cmd_vel数据控制底盘速度输出。因此我们在这里subscribe odom输入，并以此为依据advertise相关的底盘速度输出。



状态机一共有5个state。具体原理为一开始初始化起点位置（initialize1/initialize2），发送相应速度直至已走路径到达指定地点(x1y0/x0y1)，重复以上两个动作直至终点（stop）。

    enum MachineState{initialize1,x1y0,initialize2,x0y1,stop};
    MachineState state= initialize1;

在首次初始化的时候请务必sleep(1)，否则odom的数据会不准确。



在重新成功编译之后， 启动roboRTS package里的模拟机:

```
$ roslaunch roborts navigation_stage.launch 
```

随后运行roboRTS package 里的state_machine node:

```
$ roslaunch roborts navigation_stage.launch 
```

此时应该能看到小车向前行走1.7米后向左平移1.7米