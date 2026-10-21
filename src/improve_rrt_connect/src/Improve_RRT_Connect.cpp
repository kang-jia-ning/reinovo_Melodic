#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include "Improve_RRT_Connect.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

GlobalPlanner::GlobalPlanner (){
}
void GlobalPlanner::pathcallback(const nav_msgs::Path &msg){
  for(int i=0;i<msg.poses.size();i++){
    cout<<"receive the msg"<<endl;
    Imporve_RRT_path.push_back(msg.poses[i]);
  }
}
GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  initialize(name, costmap_ros);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  ros::NodeHandle nh;
  //path_sub=nh.subscribe("/RRT_ConnectPath",1,&GlobalPlanner::pathcallback,this);
 }

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
  
  nav_msgs::Path::ConstPtr pathData = ros::topic::waitForMessage<nav_msgs::Path>("Improve_RRT_Connect_Path", ros::Duration(10));
  if(pathData!=NULL){
    Imporve_RRT_path.clear();
    for(int i=0;i<pathData->poses.size();i++){

    Imporve_RRT_path.push_back(pathData->poses[i]);
    }
    plan=Imporve_RRT_path;
    return true;
  }
  //plan=Imporve_RRT_path;
  //for (int i=0;i<plan.size();i++){
  //  cout<<i<<endl;
  //  cout<<plan[i]<<endl;
  //}
  return false;
 }
 };
