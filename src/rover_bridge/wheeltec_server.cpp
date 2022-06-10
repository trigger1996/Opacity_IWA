#include <ros/ros.h>
#include "include/rover_bridge/vehicle_udp_bridge.h"

using namespace std;

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "rover_bridge");                  //节点名

    ros::NodeHandle nh("~");                                     // 必须在main函数中创建一个节点句柄给构造函数使用，并且需要把它传递到构造函数中
    ros::Rate loop_rate(10);

    string   server_ip;
    int      server_port;
    nh.getParam("/server_ip",   server_ip);
    nh.getParam("/server_port", server_port);

    const int vehicle_num = 3;
    __vehicle *vehicle[vehicle_num];
    vehicle[0] = new __vehicle(&nh, 3, 8003);       //实例化一个__vehicle对象并将指针传递给nodehandle以供构造函数使用
    vehicle[1] = new __vehicle(&nh, 4, 8004);
    vehicle[2] = new __vehicle(&nh, 5, 8005);


    while (ros::ok()) {
        for(int i = 0; i < vehicle_num; i++) {
            vehicle[i] -> receive_refine_data();
        }

        for(int i = 0; i < vehicle_num; i++) {
            if(vehicle[i] -> is_vicon_pos_updated()) {
                vehicle[i] -> reset_is_vicon_pos_updated();
            }

            if (vehicle[i]->is_vicon_pos_updated() && vehicle[i]->is_vicon_vel_updated()) {

                vehicle[i]->reset_is_vicon_pos_updated();
                vehicle[i]->reset_is_vicon_vel_updated();
            }

            if (vehicle[i]->is_vicon_pos_updated() && vehicle[i]->is_vicon_vel_updated() && vehicle[i]->is_control_updated()) {

                vehicle[i]->reset_is_vicon_pos_updated();
                vehicle[i]->reset_is_vicon_vel_updated();
                vehicle[i]->reset_is_control_updated();
            }

            if (vehicle[i]->is_control_updated()) {
                //if (true) {
                vehicle[i]->reset_is_control_updated();
                vehicle[i]->send_data_in_callback();
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // stop vehicle_1
    for (int i = 0; i < 5; i++) {
        for (int j = 0; i < vehicle_num; j++) {
            vehicle[j]->stop_vehicle();
            vehicle[j]->send_data_in_callback();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 

