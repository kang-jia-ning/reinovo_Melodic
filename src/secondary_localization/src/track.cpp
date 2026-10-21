#include <ros/ros.h>
#include <relative_move/SetRelativeMove.h>
#include <ar_pose/Track.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"auto_charge");//初始化人ros
    ros::NodeHandle n;//定义句柄
    ros::ServiceClient relative_move_client = n.serviceClient<relative_move::SetRelativeMove>("relative_move");//定义相对运动客户端
    ros::ServiceClient ar_track_client = n.serviceClient<ar_pose::Track>("track");//定义ar码跟踪客户端
    relative_move::SetRelativeMove RelativeMove_data;
    ar_pose::Track Track_data;

    ros::service::waitForService("relative_move");//等待服务启动
    ros::service::waitForService("track");//等待服务启动

    Track_data.request.ar_id = 0;  //跟踪0号ar码
    Track_data.request.goal_dist = 0.3;
    ar_track_client.call(Track_data);//调用服务
    
    //定义请求值
    RelativeMove_data.request.goal.x = -0.1;
    RelativeMove_data.request.global_frame = "odom";

    relative_move_client.call(RelativeMove_data);//调用服务
    
    std::cout<<"___"<<std::endl;
    std::cout<<"___"<<std::endl;

    return 0;
}
