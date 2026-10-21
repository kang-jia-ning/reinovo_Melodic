#include <rei_robot_cruise/cruise.h>
bool hello(){
    ROS_INFO("hello");
    return true;

}
bool bye(){
    ROS_INFO("bye");
    return true;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "cruise_test_node");
    ros::NodeHandle nh;
    rei_cruise::RobotCruise& cruise = rei_cruise::RobotCruise::getInstance();
    if(!cruise.Init(nh)) return 0; //初始化失败，结束程序
    if(cruise.AddCallBack("hello", hello)){
        ROS_INFO("set callback hello success");
    }
    if(cruise.AddCallBack("bye", bye)){
        ROS_INFO("set callback bye success");
    }
    cruise.RunNavTask();
    return 0;
}
int main2(int argc, char** argv){//另一种调用方式
  ros::init(argc, argv, "cruise_test_node");
   // rei_cruise::RobotCruise& cruise = rei_cruise::RobotCruise::getInstance();
  ros::NodeHandle nh;
  if(!rei_cruise::RobotCruise::getInstance().Init(nh))
  if(rei_cruise::RobotCruise::getInstance().AddCallBack("hello", hello)){
      ROS_INFO("set callback hello success");
  }
  if(rei_cruise::RobotCruise::getInstance().AddCallBack("bye", bye)){
      ROS_INFO("set callback bye success");
  }
  rei_cruise::RobotCruise::getInstance().RunNavTask();
  return 0;
}
