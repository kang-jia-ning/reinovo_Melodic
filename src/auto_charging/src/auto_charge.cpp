#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "auto_charging/SetCharge.h"
#include "relative_move/SetRelativeMove.h"
#include "ar_pose/Track.h"
#include <tf/transform_broadcaster.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double anchor_x,anchor_y, anchor_theta, anchor_qx, anchor_qy, anchor_qz, anchor_qw;

ros::ServiceClient PoseAdjustClient;
ros::ServiceClient RelativeMoveClient;
ros::ServiceClient TrackClient;
MoveBaseClient* ac;


//回到充电桩
bool backoff(float relative_move){
  ROS_INFO("back off ...");
	relative_move::SetRelativeMove relative_srv;
	relative_srv.request.goal.x = relative_move;
	relative_srv.request.goal.y = 0;
	relative_srv.request.goal.theta = 0;
  relative_srv.request.global_frame = "odom";
  relative_srv.request.finishStopObstacle = true;
	RelativeMoveClient.call(relative_srv);
    if(!relative_srv.response.success)
		return false;
	return true;
}
//追踪ar码
bool adjust(int id){
    ar_pose::Track srv;
    //选择跟踪ar码所含有的信息
    srv.request.ar_id=id;
    srv.request.goal_dist= 0.4;
    TrackClient.call(srv);
    if(!srv.response.success)
        return false;
    return true;
}

//导航到目标点
bool nav2anchor() {
    move_base_msgs::MoveBaseGoal goal;
    ros::Time last_time;
    last_time = ros::Time::now();
    while(!ac->waitForServer(ros::Duration(1.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
        if((ros::Time::now()-last_time)>ros::Duration(5.0)) return false;
    }
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(anchor_theta);
    goal.target_pose.pose.position.x = anchor_x;
    goal.target_pose.pose.position.y = anchor_y;
    goal.target_pose.pose.orientation.x = orientation.x;
    goal.target_pose.pose.orientation.y = orientation.y;
    goal.target_pose.pose.orientation.z = orientation.z;
    goal.target_pose.pose.orientation.w = orientation.w;

    ROS_INFO("going to anchor position to charge");
    ac->sendGoal(goal);
    ac->waitForResult();
    if(ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("going to anchor positon fialed, charging faield");
    return false;
    }
    ROS_INFO("arrival_at anchor posistion");
    return true;
}

bool ChargeCallback(auto_charging::SetCharge::Request &req, auto_charging::SetCharge::Response&res){
    res.success =  true;
    bool next_flag = true;
    if(req.nav){
        next_flag = false;
        if(nav2anchor()) {
            res.message = "nav successed , ";
            res.success =  true;
            next_flag  = true;
        }else res.success = false;
    }
    sleep(1);
    if(next_flag&&req.ar_track){
        if(adjust(req.ar_id)){
            res.message = "adjust successed ";
            res.success =  res.success&&true;
            sleep(1);
            if(backoff(-0.20)){
                res.message = "adjust successed ";
                res.success =  true;
            }
        }else res.success = false;
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "auto_charging");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    ros::ServiceServer AutoChargingServer = n.advertiseService("goto_charge", ChargeCallback);
    TrackClient = n.serviceClient<ar_pose::Track>("track");

    RelativeMoveClient = n.serviceClient<relative_move::SetRelativeMove>("relative_move");

    ac = new MoveBaseClient ("move_base", true);
    private_nh.param<double>("anchor_x", anchor_x, 0);
    private_nh.param<double>("anchor_y", anchor_y, 0);
    private_nh.param<double>("anchor_theta", anchor_theta, 0);

    ros::spin();
    delete ac;
    return 0;
}

