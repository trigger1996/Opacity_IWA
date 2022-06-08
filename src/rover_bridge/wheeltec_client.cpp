#include <iostream>
#include <ostream>
#include <cmath>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <ros/ros.h>    // https://blog.csdn.net/qq_40207976/article/details/113337366?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_utm_term~default-0.no_search_link&spm=1001.2101.3001.4242
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;

geometry_msgs::PoseStamped vicon_pose;  // commander_computer -> vehicle
geometry_msgs::Twist       vicon_vel;   // commander_computer -> vehicle
geometry_msgs::Twist       cmd_vel;     // commander_computer -> vehicle
nav_msgs::Odometry odom;                // vehicle            -> commander_computer
sensor_msgs::Imu   imu;                 // vehicle            -> commander_computer

typedef struct {
  double roll;
  double pitch;
  double yaw;
} __rpy;

__rpy vicon_rpy, odom_rpy;

inline __rpy quat_2_rpy(double x, double y, double z, double w, __rpy &rpy) {
    geometry_msgs::Pose pose_t;

    pose_t.orientation.x = x;
    pose_t.orientation.y = y;
    pose_t.orientation.z = z;
    pose_t.orientation.w = w;

    tf::Quaternion q;
    tf::quaternionMsgToTF(pose_t.orientation, q);
    tf::Matrix3x3(q).getRPY(rpy.roll, rpy.pitch, rpy.yaw);

    return rpy;
}

bool is_local_odom_updated = false;
void local_odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    odom.pose.pose.position.x = msg->pose.pose.position.x;
    odom.pose.pose.position.y = msg->pose.pose.position.y;
    odom.pose.pose.position.z = msg->pose.pose.position.z;

    odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    odom.twist.twist.linear.x  = msg->twist.twist.linear.x;
    odom.twist.twist.angular.z = msg->twist.twist.angular.z;

    is_local_odom_updated = true;
}

bool is_local_imu_updated = false;
void local_imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
    imu.orientation.x = msg->orientation.x;
    imu.orientation.y = msg->orientation.y;
    imu.orientation.z = msg->orientation.z;
    imu.orientation.w = msg->orientation.w;

    imu.angular_velocity.x = msg->angular_velocity.x;
    imu.angular_velocity.y = msg->angular_velocity.y;
    imu.angular_velocity.z = msg->angular_velocity.z;

    is_local_imu_updated = true;
}

bool refine_CenteralizedData(string recv, 
                             geometry_msgs::PoseStamped &vicon_pos, geometry_msgs::Twist &vicon_vel, __rpy &vicon_rpy,
                             geometry_msgs::Twist &cmd_vel);


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "wheeltec_interface_client");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    string server_ip;
    int server_port, client_port;
    string vehicle_id;

    nh.getParam("/server_ip",   server_ip);
    nh.getParam("/server_port", server_port);
    nh.getParam("vehicle_id",   vehicle_id);
    nh.getParam("client_port",  client_port);    

    string vicon_pos_topic_name = string("/vehicle") + string("_") + string(vehicle_id) + string("/vicon_pose");
    string vicon_vel_topic_name = string("/vehicle") + string("_") + string(vehicle_id) + string("/vicon_vel");
    string cmd_vel_topic_name   = string("/vehicle") + string("_") + string(vehicle_id) + string("/cmd_vel");
    ros::Publisher    vicon_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(vicon_pos_topic_name.data(), 1);
    ros::Publisher    vicon_vel_pub  = nh.advertise<geometry_msgs::Twist>(vicon_vel_topic_name.data(), 1);
    ros::Publisher    cmd_vel_pub    = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name.data(), 1);
    ros::Subscriber   local_odom_sub = nh.subscribe("/odom", 1, local_odom_cb);                      // /odom 要改topic name
    ros::Subscriber   local_imu_sub  = nh.subscribe("/imu",  1, local_imu_cb);

    odom.pose.pose.orientation.z = 1;                                                                // 为了得到一个单位四元数, 运行的时候不报错

    /* socket文件描述符 */
    int sock_fd;


    sleep(2);

    cout << "[Client] server_ip:   " << server_ip   << endl;
    cout << "[Client] server_port: " << server_port << endl;
    cout << "[Client] client_port: " << client_port << endl;

    /* 建立udp socket */
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
    int on = 1;
    setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR | SO_BROADCAST, &on, sizeof(on));

    /* 设置address */
    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(server_ip.data());
    addr_serv.sin_port = htons(client_port);
    len = sizeof(addr_serv);


    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr(server_ip.data());
    dest_addr.sin_port = htons(client_port);                                   /// PORT Critical


    //printf("client send: %s\n", send_buf);

    while(ros::ok()) {

        if (is_local_odom_updated && is_local_imu_updated) {
        //if (true) {          

        ostringstream ostr_odom;
        quat_2_rpy(imu.orientation.x, imu.orientation.y,
                   imu.orientation.z, imu.orientation.w,
                   odom_rpy);

	    // For Testing
            /*
            odom.pose.pose.position.x = -123.4567;
            odom.pose.pose.position.y =  0.2155;
            odom.pose.pose.position.z =  670.2155;
            odom_rpy.roll  = -M_PI / 4;
            odom_rpy.pitch = -M_PI / 6;
            odom_rpy.yaw   =  M_PI / 4 * 3;
            odom.twist.twist.linear.x = 0.225;
            odom.twist.twist.linear.y = -0.425;
            odom.twist.twist.linear.z = 0.5425;
            odom.twist.twist.angular.x = 1.465;
            odom.twist.twist.angular.y = -34.475;
            odom.twist.twist.angular.z = -0.126;
            */

            ostr_odom << "x: "     << odom.pose.pose.position.x  << " "
                      << "y: "     << odom.pose.pose.position.y  << " "
                      << "z: "     << odom.pose.pose.position.z  << " "
                      << "roll: "  << odom_rpy.roll              << " "
                      << "pitch: " << odom_rpy.pitch             << " "
                      << "yaw: "   << odom_rpy.yaw               << " "
                      << "vx: "    << odom.twist.twist.linear.x  << " "
                      << "vy: "    << odom.twist.twist.linear.y  << " "         // 小车只要vx和wz, 这个为了可能适配多旋翼
                      << "vz: "    << odom.twist.twist.linear.z  << " "
                      << "wx: "    << imu.angular_velocity.x     << " "
                      << "wy: "    << imu.angular_velocity.y     << " "
                      << "wz: "    << imu.angular_velocity.z     << " "
                      << endl;
            string data_to_send = ostr_odom.str();

            int send_num = sendto(sock_fd, data_to_send.data(), strlen(data_to_send.data()), MSG_DONTWAIT, (struct sockaddr *)&dest_addr, len);

            if(send_num < 0) {
                //perror("sendto error:");
                //exit(1);
            }

            is_local_odom_updated = false;
            is_local_imu_updated  = false;
        }

        char recv_buf[320] = { 0 };
        int recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT, (struct sockaddr *)&dest_addr, (socklen_t *)&len);	// recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&dest_addr, (socklen_t *)&len);
        
        if(recv_num < 0)
        {
          //perror("recvfrom error:");
          //exit(1);
        }
        else {
        //if (true) {

            string recv_str = string(recv_buf);
            refine_CenteralizedData(recv_str, vicon_pose, vicon_vel, vicon_rpy, cmd_vel);


            tf::Quaternion q = tf::createQuaternionFromRPY(odom_rpy.roll, odom_rpy.pitch, odom_rpy.yaw);
            vicon_pose.pose.orientation.x = q.x();
            vicon_pose.pose.orientation.y = q.y();
            vicon_pose.pose.orientation.z = q.z();
            vicon_pose.pose.orientation.w = q.w();


            // 坐标系变换?


            vicon_pose_pub.publish(vicon_pose);
            vicon_vel_pub.publish(vicon_vel);
            cmd_vel_pub.publish(cmd_vel);
        }

        recv_buf[recv_num] = '\0';
        //printf("client receive %d bytes: %s\n", recv_num, recv_buf);

        ros::spinOnce();
        loop_rate.sleep();
    }

  close(sock_fd);
  return 0;
}


bool refine_CenteralizedData(string recv, 
                             geometry_msgs::PoseStamped &vicon_pos, geometry_msgs::Twist &vicon_vel, __rpy &vicon_rpy,
                             geometry_msgs::Twist &cmd_vel) {

    // vicon_x: 0 vicon_y: 0 vicon_z: 0 vicon_roll: 0 vicon_pitch: 0 vicon_yaw: 0 vicon_vx: 0 vicon_vy: 0 vicon_vz: 0 vicon_wx: 0 vicon_wy: 0 vicon_wz: 0 vx_cmd: 0 vy_cmd: 0 vz_cmd: 0 wz_cmd: 0
    int current_pos = -1, next_pos = -1;
    char str[25] = { 0 };

    if (recv.find("vicon_x: ") != 0)
        return false;               // 检查数据包正确性

    // vicon_x
    current_pos = recv.find("vicon_x: ") + strlen("vicon_x: ");
    next_pos    = recv.find("vicon_y: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_pos.pose.position.x = atof(str);
    
    // vicon_y
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vicon_y: ") + strlen("vicon_y: ");
    next_pos    = recv.find("vicon_z: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_pos.pose.position.y = atof(str);

    // vicon_z
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vicon_z: ")    + strlen("vicon_z: ");
    next_pos    = recv.find("vicon_roll: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_pos.pose.position.z = atof(str);

    // vicon_roll
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vicon_roll: ")  + strlen("vicon_roll: ");
    next_pos    = recv.find("vicon_pitch: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_rpy.roll = atof(str);

    // vicon_pitch
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vicon_pitch: ") + strlen("vicon_pitch: ");
    next_pos    = recv.find("vicon_yaw: ")   - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_rpy.pitch = atof(str);

    // vicon_yaw
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vicon_yaw: ")  + strlen("vicon_yaw: ");
    next_pos    = recv.find("vicon_vx: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_rpy.yaw = atof(str);

    // vicon_vx
    current_pos = recv.find("vicon_vx: ") + strlen("vicon_vx: ");
    next_pos    = recv.find("vicon_vy: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_vel.linear.x = atof(str);
    
    // vicon_vy
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vicon_vy: ") + strlen("vicon_vy: ");
    next_pos    = recv.find("vicon_vz: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_vel.linear.y = atof(str);

    // vicon_vz
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vicon_vz: ") + strlen("vicon_vz: ");
    next_pos    = recv.find("vicon_wx: ")   - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_vel.linear.z = atof(str);

    // vicon_wx
    current_pos = recv.find("vicon_wx: ") + strlen("vicon_wx: ");
    next_pos    = recv.find("vicon_wy: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_vel.angular.x = atof(str);
    
    // vicon_wy
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vicon_wy: ") + strlen("vicon_wy: ");
    next_pos    = recv.find("vicon_wz: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_vel.angular.y = atof(str);

    // vicon_wz
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vicon_wz: ") + strlen("vicon_wz: ");
    next_pos    = recv.find("vx_cmd: ")   - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    vicon_vel.angular.z = atof(str);

    // vx_cmd
    current_pos = recv.find("vx_cmd: ") + strlen("vx_cmd: ");
    next_pos    = recv.find("vy_cmd: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    cmd_vel.linear.x = atof(str);
    
    // vy_cmd
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vy_cmd: ") + strlen("vy_cmd: ");
    next_pos    = recv.find("vz_cmd: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    cmd_vel.linear.y = atof(str);

    // vz_cmd
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vz_cmd: ") + strlen("vz_cmd: ");
    next_pos    = recv.find("wz_cmd: ")   - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    cmd_vel.linear.z = atof(str);

    // wz
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("wz_cmd: ") + strlen("wz_cmd: ");
    next_pos    = recv.length();
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    cmd_vel.angular.z = atof(str);    

    /*
    cout << "vicon_x: "     << vicon_pose.pose.position.x  << " "
         << "vicon_y: "     << vicon_pose.pose.position.y  << " "
         << "vicon_z: "     << vicon_pose.pose.position.z  << " "
         << "vicon_roll: "  << vicon_rpy.roll              << " "
         << "vicon_pitch: " << vicon_rpy.pitch             << " "
         << "vicon_yaw: "   << vicon_rpy.yaw               << " "
         << "vicon_vx: "    << vicon_vel.linear.x  << " "
         << "vicon_vy: "    << vicon_vel.linear.y  << " "
         << "vicon_vz: "    << vicon_vel.linear.z  << " "
         << "vicon_wx: "    << vicon_vel.angular.x << " "
         << "vicon_wy: "    << vicon_vel.angular.y << " "
         << "vicon_wz: "    << vicon_vel.angular.z << " "
         << "vx_cmd: "      << cmd_vel.linear.x  << " "
         << "vy_cmd: "      << cmd_vel.linear.y  << " "           // vy, vz为了多旋翼拓展
         << "vz_cmd: "      << cmd_vel.linear.z  << " "
         << "wz_cmd: "      << cmd_vel.angular.z << " "
         << endl;
    */

   return true;

}
