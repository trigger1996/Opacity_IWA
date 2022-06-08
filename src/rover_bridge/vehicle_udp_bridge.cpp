#include "include/rover_bridge/vehicle_udp_bridge.h"

#define UDP_CLIENT_PORT_DEFAULT 8000

using namespace std;

inline void quat_2_rpy(double x, double y, double z, double w, __rpy &rpy) {
    geometry_msgs::Pose pose_t;

    tf::Quaternion q(x, y, z, w);
    q.normalize();

    tf::Matrix3x3(q).getRPY(rpy.roll, rpy.pitch, rpy.yaw);
}

// 不得不通过节点句柄指针进入构造函数再有构造函数去构建subscriber
__vehicle::__vehicle(ros::NodeHandle* nodehandle):nh_(*nodehandle) // 构造函数
{ 
    ROS_INFO("in class constructor of ExampleRosClass");

    this->vehicle_id  = 1;
    this->client_port = UDP_CLIENT_PORT_DEFAULT + this->vehicle_id;

    initializeSubscribers(); // 需要通过成员协助函数帮助构建subscriber，在构造函数中做初始化的工作
    initializePublishers();
    initializeServices();

    initialize_UDP();    

    is_vicon_pos_updated_ = false;
    is_vicon_vel_updated_ = false;
    is_control_updated_   = false;

    val_to_remember_= 0.0;                      //初始化储存数据的变量的值
    
    ROS_INFO("Vehicle %d UDP bridge initialized!, port: %d", this->vehicle_id, this->client_port);
}

__vehicle::__vehicle(ros::NodeHandle* nodehandle, uint8_t vehicle_id, uint16_t client_port):nh_(*nodehandle) // 构造函数
{ 
    ROS_INFO("in class constructor of ExampleRosClass");

    this->vehicle_id = vehicle_id;
    this->client_port = client_port;

    initializeSubscribers();                    // 需要通过成员协助函数帮助构建subscriber，在构造函数中做初始化的工作
    initializePublishers();
    initializeServices();

    initialize_UDP();

    is_vicon_pos_updated_ = false;
    is_vicon_vel_updated_ = false;
    is_control_updated_   = false;

    val_to_remember_= 0.0;                      //初始化储存数据的变量的值


    ROS_INFO("Vehicle %d UDP bridge initialized!", this->vehicle_id);
}

__vehicle::~__vehicle() {
    close(sock_fd);
}

void __vehicle::stop_vehicle() {
    cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = cmd_vel.angular.z = 0;

}

//以下是一个成员协助函数，帮助构建subscriber
void __vehicle::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    //minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &__vehicle::subscriberCallback,this);  
    //订阅了名称为"example_class_input_topic"的topic
    //&__vehicle::subscriberCallback 是一个指向成员函数“__vehicle”的指针
    // “this”的用意是指向当前实例（__vehicle）

    string vicon_pos_topic_name = string("/server/vicon_pose") + string("_") + to_string(this->vehicle_id);
    string vicon_vel_topic_name = string("/server/vicon_vel")  + string("_") + to_string(this->vehicle_id);
    string cmd_vel_name         = string("/server/cmd_vel")    + string("_") + to_string(this->vehicle_id);
    string odom_name            = string("/server/odom")       + string("_") + to_string(this->vehicle_id);

    vicon_pose_sub = nh_.subscribe(vicon_pos_topic_name.data(), 1, &__vehicle::vicon_pose_cb, this);        // https://wenku.baidu.com/view/d2d39467bd1e650e52ea551810a6f524ccbfcba8.html
    vicon_vel_sub  = nh_.subscribe(vicon_vel_topic_name.data(), 1, &__vehicle::vicon_vel_cb, this);
    cmd_vel_sub    = nh_.subscribe(cmd_vel_name.data(),         1, &__vehicle::cmd_vel_cb, this);
    odom_pub       = nh_.advertise<nav_msgs::Odometry>(odom_name.data(), 1);
}

//与上相同
void __vehicle::initializeServices()
{
    ROS_INFO("Initializing Services");
    //minimal_service_ = nh_.advertiseService("example_minimal_service",
    //                                               &__vehicle::serviceCallback,
    //                                               this);  

}

//与上相同
void __vehicle::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    //minimal_publisher_ = nh_.advertise<std_msgs::Float32>("example_class_output_topic", 1, true); 
}

bool __vehicle::initialize_UDP() {

    /* sock_fd --- socket文件描述符 创建udp套接字*/
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0) {
        perror("socket");
        exit(1);
    }

    // 初始化UDP
    memset(&dest_addr, 0, sizeof(struct sockaddr_in));          //每个字节都用0填充
    dest_addr.sin_family = AF_INET;                             //使用IPV4地址
    dest_addr.sin_port = htons(client_port);                    //端口
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);              //自动获取IP地址
    len = sizeof(dest_addr);

    /* 绑定socket */
    if(bind(sock_fd, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        perror("bind error:");
        exit(1);
    }
    ROS_INFO("UDP Socked Binded! fd: %d, port: %d", sock_fd, client_port);

}

//以下内容与之前在构建publishers/subscribers通信时，编写的publishers和subscribers节点相似
//大部分的工作都是在回调函数中完成的
void __vehicle::subscriberCallback(const std_msgs::Float32& message_holder) {

    val_from_subscriber_ = message_holder.data; // 用成员变量储存回调的数据
    ROS_INFO("myCallback activated: received value %f",val_from_subscriber_);
    std_msgs::Float32 output_msg;
    val_to_remember_ += val_from_subscriber_; //储存每次调用的数据
    output_msg.data= val_to_remember_;

    minimal_publisher_.publish(output_msg); //在这里之所以可以使用publishers的功能，是因为在类中publishers也是其他一个成员函数，所以可以被调用
}


//与构建services/clients通信时相似
bool __vehicle::serviceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
    ROS_INFO("service callback activated");
    response.success = true; // Trigger的消息类型是bool和string，对应的数据类型就是success和message
    response.message = "here is a response string";
    return true;
}

void __vehicle::vicon_pose_cb(const geometry_msgs::PoseStamped& msg) {
    vicon_pose.pose.position.x = msg.pose.position.x;
    vicon_pose.pose.position.y = msg.pose.position.y;
    vicon_pose.pose.position.z = msg.pose.position.z;

    vicon_pose.pose.orientation.x = msg.pose.orientation.x;
    vicon_pose.pose.orientation.y = msg.pose.orientation.y;
    vicon_pose.pose.orientation.z = msg.pose.orientation.z;
    vicon_pose.pose.orientation.w = msg.pose.orientation.w;

    quat_2_rpy(vicon_pose.pose.orientation.x, vicon_pose.pose.orientation.y,
            vicon_pose.pose.orientation.z, vicon_pose.pose.orientation.w, vicon_rpy);

    is_vicon_pos_updated_ = true;
}

void __vehicle::vicon_vel_cb(const geometry_msgs::Twist& msg) {

    is_vicon_vel_updated_ = true;
}

void __vehicle::cmd_vel_cb(const geometry_msgs::Twist& msg) {
    cmd_vel.linear.x  = msg.linear.x;
    cmd_vel.angular.z = msg.angular.z;

    is_control_updated_   = true;
}

bool __vehicle::receive_refine_data() {
    int recv_num;
    char recv_buf[240];

    recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT, (struct sockaddr *)&dest_addr, (socklen_t *)&len);   //(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&dest_addr, (socklen_t *)&len);

    if(recv_num < 0) {
        //perror("recvfrom error:");
        //exit(1);
    } else {

        recv_buf[recv_num] = '\0';
        //printf("server receive %d bytes: %s\n", recv_num, recv_buf);


        string recv_str = string(recv_buf);
        refine_VehicleData(recv_str, odom, odom_rpy);

        tf::Quaternion q = tf::createQuaternionFromRPY(odom_rpy.roll, odom_rpy.pitch, odom_rpy.yaw);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();


        //cout << odom.pose.pose.orientation << endl;

        // 坐标系变换?

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "base_link";           // "map"
        odom_pub.publish(odom);

    }
}

bool __vehicle::send_data_in_callback() {
    ostringstream ostr_odom;

    quat_2_rpy(vicon_pose.pose.orientation.x, vicon_pose.pose.orientation.y,
                vicon_pose.pose.orientation.z, vicon_pose.pose.orientation.w,
                vicon_rpy);

    // for testing
    /*
    vicon_pose.pose.position.x = 0.25;
    vicon_pose.pose.position.y = -0.75;
    vicon_pose.pose.position.z = 1.125;
    vicon_rpy.roll  =  M_PI / 12;
    vicon_rpy.pitch = -M_PI / 24;
    vicon_rpy.yaw   = -M_PI * 0.025;
    vicon_vel.linear.x = 0.15;
    vicon_vel.linear.y = 0.125;
    vicon_vel.linear.z = -0.1285;
    vicon_vel.angular.x = -1.165;
    vicon_vel.angular.y = -15.105;
    vicon_vel.angular.z = -1.355;
    cmd_vel.linear.x = -1.855;
    cmd_vel.linear.y = -15.365;
    cmd_vel.linear.z =  15.35;
    cmd_vel.angular.z =  M_PI / 4;
    */

    ostr_odom << "vicon_x: "     << vicon_pose.pose.position.x  << " "
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
    string data_to_send = ostr_odom.str();

    int send_num = sendto(sock_fd, data_to_send.data(), strlen(data_to_send.data()) + 1, MSG_DONTWAIT, (struct sockaddr *)&dest_addr, len);        // (sock_fd, data_to_send.data(), strlen(data_to_send.data()) + 1, 0, (struct sockaddr *)&dest_addr, len)

    if(send_num < 0) {
        perror("sendto error:");
        exit(1);
    }
            
}

bool __vehicle::refine_VehicleData(string recv, nav_msgs::Odometry &odom, __rpy &rpy) {
    // x: 0 y: 0 z: 0 roll: 0 pitch: 0 yaw: 0 vx: 0 wz: 0
    int current_pos = -1, next_pos = -1;
    char str[25] = { 0 };

    if (recv.find("x: ") != 0)
        return false;               // 检查数据包正确性

    // x
    current_pos = recv.find("x: ") + strlen("x: ");
    next_pos    = recv.find("y: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    odom.pose.pose.position.x = atof(str);
    
    // y
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("y: ") + strlen("y: ");
    next_pos    = recv.find("z: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    odom.pose.pose.position.y = atof(str);

    // z
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("z: ")    + strlen("z: ");
    next_pos    = recv.find("roll: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    odom.pose.pose.position.z = atof(str);

    // roll
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("roll: ")  + strlen("roll: ");
    next_pos    = recv.find("pitch: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    rpy.roll = atof(str);

    // pitch
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("pitch: ") + strlen("pitch: ");
    next_pos    = recv.find("yaw: ")   - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    rpy.pitch = atof(str);

    // yaw
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("yaw: ") + strlen("yaw: ");
    next_pos    = recv.find("vx: ")  - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    rpy.yaw = atof(str);

    // vx
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vx: ") + strlen("vx: ");
    next_pos    = recv.find("vy: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    odom.twist.twist.linear.x = atof(str);

    // vy
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vy: ") + strlen("vy: ");
    next_pos    = recv.find("vz: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    odom.twist.twist.linear.y = atof(str);

    // vz
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("vz: ") + strlen("vz: ");
    next_pos    = recv.find("wx: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    odom.twist.twist.linear.z = atof(str);    

    // wx
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("wx: ") + strlen("wx: ");
    next_pos    = recv.find("wy: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    odom.twist.twist.angular.x = atof(str);    

    // wy
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("wy: ") + strlen("wy: ");
    next_pos    = recv.find("wz: ") - strlen(" ");
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    odom.twist.twist.angular.y = atof(str);    

    // wz
    memset(str, 0, sizeof(char) * 25);
    current_pos = recv.find("wz: ") + strlen("wz: ");
    next_pos    = recv.length();
    for (int i = current_pos, j = 0; i < next_pos; i++, j++) {
        str[j] = recv.data()[i];
    }
    odom.twist.twist.angular.z = atof(str);    

    
    //cout << current_pos << " " << next_pos << endl;
    /*
    cout << "vehicle_id: " << (uint32_t)vehicle_id 
        <<  "x: " << odom.pose.pose.position.x 
        << " y: " << odom.pose.pose.position.y
        << " y: " << odom.pose.pose.position.z 
        << " roll: "  << rpy.roll 
        << " pitch: " << rpy.pitch
        << " yaw: "   << rpy.yaw 
        << " vx: "    << odom.twist.twist.linear.x
        << " vy: "    << odom.twist.twist.linear.y
        << " vz: "    << odom.twist.twist.linear.z
        << " wx: "    << odom.twist.twist.angular.z
        << " wy: "    << odom.twist.twist.angular.y
        << " wz: "    << odom.twist.twist.angular.z << endl;
    */

    return true;

}
