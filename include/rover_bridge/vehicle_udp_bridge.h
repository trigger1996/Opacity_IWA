#ifndef __Vehicle_UDP_Bridge_H  //①---一种技巧，注释请看最后一行代码
#define __Vehicle_UDP_Bridge_H  //②---一种技巧，注释请看最后一行代码

//常用的成员
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdint.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>

#include <ros/ros.h> //惯例添加

//需要用到的消息类型，有.msg和.srv两类，要用到两种通信机制
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h> // 可以用rossrv show std_srvs/Trigger查看消息类型

#include <tf/transform_datatypes.h>

typedef struct {
  double roll;
  double pitch;
  double yaw;
} __rpy;

// 现在开始定义一个类，包括构造函数，成员函数和成员变量
//构造函数是在类里面使用的函数，构造函数的名称与类额名称相同
class __vehicle
{
public:
    __vehicle(ros::NodeHandle* nodehandle); //main函数需要一个ROS的节点句柄，并通过这个节点句柄连接到这个构造函数

    __vehicle(ros::NodeHandle* nodehandle, uint8_t vehicle_id, uint16_t client_port);

    ~__vehicle();

    bool receive_refine_data();
    bool send_data_in_callback();

    void stop_vehicle();

    inline bool is_vicon_pos_updated() {
        return is_vicon_pos_updated_;
    }

    inline bool is_vicon_vel_updated() {
        return is_vicon_vel_updated_;
    }

    inline bool is_control_updated() {
        return is_control_updated_;
    }

    inline void reset_is_vicon_pos_updated() {
        is_vicon_pos_updated_ = false;
    }

    inline void reset_is_vicon_vel_updated() {
        is_vicon_vel_updated_ = false;
    }

    inline void reset_is_control_updated() {
        is_control_updated_ = false;
    }

protected:
    // 私有的数据成员只能在该类中被调用
    ros::NodeHandle nh_; // 通过这个节点句柄连接main函数和构造函数
    //定义一些subscriber、publisher和service的变量
    ros::Subscriber minimal_subscriber_; //这里三个变量后面都加了下划线，作用是提醒这三个变量只能在该类（__vehicle）中被调用
    ros::ServiceServer minimal_service_;
    ros::Publisher  minimal_publisher_;

    ros::Subscriber vicon_pose_sub;
    ros::Subscriber vicon_vel_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher  odom_pub;

    double val_from_subscriber_; //定义变量接收subscriber回调函数的变量，有优于使用全局变量，便于将数据从subscriber处传到其他成员函数
    double val_to_remember_; // 储存变量数据


    uint8_t     vehicle_id;
    std::string client_ip;
    uint16_t    client_port;

    int sock_fd;
    struct sockaddr_in dest_addr;
    int len;


    geometry_msgs::PoseStamped vicon_pose;  // commander_computer -> vehicle
    geometry_msgs::Twist       vicon_vel;   // commander_computer -> vehicle
    geometry_msgs::Twist       cmd_vel;     // commander_computer -> vehicle
    nav_msgs::Odometry         odom;        // vehicle            -> commander_computer

    __rpy vicon_rpy, odom_rpy;

    bool is_vicon_pos_updated_;
    bool is_vicon_vel_updated_;
    bool is_control_updated_;

//-------------------------------------------------------
    void initializeSubscribers(); 
    void initializePublishers();
    void initializeServices();

    bool initialize_UDP();

    void subscriberCallback(const std_msgs::Float32& message_holder); //subscriber回调函数的原型
    bool serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);//service回调函数的原型

    void vicon_pose_cb(const geometry_msgs::PoseStamped& msg);
    void vicon_vel_cb(const geometry_msgs::Twist& msg);
    void cmd_vel_cb(const geometry_msgs::Twist& msg);

    bool refine_VehicleData(std::string recv, nav_msgs::Odometry &odom, __rpy &rpy);

//-------------------------------------------------------
//以上分割线内的五个函数，尽量避免将执行代码写入函数中，应将对应的执行代码写入另外的独立的一个或多个 .cpp 文件中
}; // 注意，类定义的结尾处除了要加大括号还要记得加上一个分号

#endif  // __Vehicle_UDP_Bridge_H

//https://blog.51cto.com/u_15127667/3757940
