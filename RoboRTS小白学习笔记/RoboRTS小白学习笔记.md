# RoboRTS小白学习笔记

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



## 手柄/键盘控车

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

### 手柄控车node

