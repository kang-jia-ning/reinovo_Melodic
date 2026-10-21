/*
 * @Author: Zhaoq 1327153842@qq.com
 * @Date: 2022-06-15 13:53:15
 * @LastEditors: Zhaoq 1327153842@qq.com
 * @LastEditTime: 2023-09-09 07:19:08
 * @Gitee: https://gitee.com/reinovo
 * Copyright (c) 2022 by 深圳市元创兴科技, All Rights Reserved. 
 * @Description: 
 */
#ifndef REI_ROBOT_CRUISE_H_
#define REI_ROBOT_CRUISE_H_

#include <ros/ros.h>
#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <functional>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>
#include "rei_robot_cruise/StartStopNav.h"
#include "rei_robot_cruise/SetGoals.h"
#include "rei_robot_cruise/RemoveGoal.h"
#include "rei_robot_cruise/SetNavOrder.h"
#include "rei_robot_cruise/NavGoal.h"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MOVEBASE;
namespace rei_cruise
{
class RobotCruise
{
private:
  
  // void _DynamicConfigCallback(robot_cruise::navigatorConfig &config, uint32_t level);
  
  /**
   * @brief 目标点设置服务回调函数
   * 
   * @param req 
   * @param res 
   * @return true 
   * @return false 
   */
  bool _SetGoalsCallback(rei_robot_cruise::SetGoals::Request &req,
                          rei_robot_cruise::SetGoals::Response &res);
  /**
   * @brief 设置巡航启停服务回调函数
   * 
   * @param req 
   * @param res 
   * @return true 
   * @return false 
   */
  bool _StartStopGoalsCallback(rei_robot_cruise::StartStopNav::Request &req,
                              rei_robot_cruise::StartStopNav::Response &res);
  /**
   * @brief 设置巡航顺序服务回调函数
   * 
   * @param req 
   * @param res 
   * @return true 
   * @return false 
   */
  bool _SetGoalsOrderCallback(rei_robot_cruise::SetNavOrder::Request &req,
                              rei_robot_cruise::SetNavOrder::Response &res);
  
  /**
   * @brief 设置清除目标点服务回调函数
   * 
   * @param req 
   * @param res 
   * @return true 
   * @return false 
   */
  bool _RemoveGoalCallback(rei_robot_cruise::RemoveGoal::Request &req,
                              rei_robot_cruise::RemoveGoal::Response &res);

  /**
   * @brief 设置在rviz中显示的目标点模型
   * 
   * @param pose 模型坐标
   * @param goal_name 目标点名
   */
  void _SetLandMark(geometry_msgs::Pose pose, std::string goal_name);

  void _EulerToQuart(const double euler[3], geometry_msgs::Quaternion &quart){
    quart = tf::createQuaternionMsgFromRollPitchYaw(euler[0], euler[1], euler[2]);
  }


  RobotCruise();
  ~RobotCruise();
  RobotCruise(const RobotCruise& other) = delete;
  RobotCruise& operator=(const RobotCruise& other) = delete;
  
private:
  ros::NodeHandle nh_;
  std::unique_ptr<MOVEBASE> _moveBaseClientPtr_; //move_base客户端指针，使用智能指针只是防止野指针的出现

  int _loopTimes_, _requestLoop_;//loop_times_：当前剩余圈数，request_loop_：目标圈数
  ros::ServiceServer _setGoalsServer_, _startStopNavServer_, _setGoalsOrderServer_, _removeGoalsServer_;

  ros::Publisher _markerPublisher_;
  bool _flagPublishMarker_;

  std::mutex mtx_;
  /*
  存储目标点，格式为{目标点名字: 目标点信息}
  因大部分操作为无序查找所以选择unordered_map*/
  std::unordered_map<std::string, rei_robot_cruise::NavGoal> _goalsList_;
  /*
  存储目标点巡航顺序，格式为{顺序号: 目标点名字}
  因大部分操作为顺序查找所以选择map
      */
  std::map<int,std::string> _executableOrder_;
  
  std::function<bool()> _startCallback_;

  std::map<int,std::function<bool()>> _executableOrderCb_;
  
  std::unordered_map<std::string, std::function<bool()>> _callbackMap_;

  /*存储在rviz中显示的目标点模型，格式为{目标点名字: 模型形状}*/
  std::map<std::string, visualization_msgs::MarkerArray> _landmarkersMap_;

  visualization_msgs::Marker _marker_;

  bool _readParams_;

  std::string _moveBaseName_;
  int _currentNavingGoal_;
  int _taskState_;//任务当前执行阶段
  int _mode_;//程序目标执行阶段

public:
  
  static RobotCruise& getInstance(){
    static RobotCruise instance;
    return instance;
  }

  bool Start();
  bool Pause();
  bool Stop();

  bool Init(ros::NodeHandle &nh);
  void RunNavTask();
  bool AddCallBack(std::string func_name, const std::function<bool()> &func){
    setlocale(LC_CTYPE, "zh_CN.utf8");
    if(func_name=="default"){
      ROS_WARN("default为内部已设置的默认回调函数, 不可修改或手动添加");
      return false;
    }else{
      auto ret = _callbackMap_.emplace(func_name, func);
      if(!ret.second){
        _callbackMap_.find(func_name)->second = func;
      }
    }
    ROS_INFO("设置回调函数 %s 成功", func_name.c_str());
    return true;
  }
  bool AddStartCallBack(const std::function<bool()> &func){
    setlocale(LC_CTYPE, "zh_CN.utf8");
    _startCallback_ = func;
    ROS_INFO("设置起始回调函数成功");
    return true;
  }
  bool DeleteCallBack(std::string funcName){
    setlocale(LC_CTYPE, "zh_CN.utf8");
    if(funcName=="default"){
      ROS_WARN("default为内部已设置的默认回调函数, 不可删除");
      return false;
    }else{
      auto iter = _callbackMap_.find(funcName);
      if(iter!=_callbackMap_.end()){
        _callbackMap_.erase(iter);
        for(auto executableIt = _executableOrderCb_.begin(); 
                executableIt !=_executableOrderCb_.end(); executableIt++){
          executableIt->second = _callbackMap_["default"];
        }
      }else ROS_WARN("未发现回调函数%s",funcName.c_str());
    }
    ROS_INFO("删除回调函数 %s 成功", funcName.c_str());
    return true;
  }
  int GetTaskState(){
    return _taskState_;
  }
  void GetCurrentGoal(geometry_msgs::Pose &goal){
    std::string name= GetCurrentGoalName();
    goal = _goalsList_[name].target_pose.pose;
  }
  std::string GetCurrentGoalName(){
    return _executableOrder_[_currentNavingGoal_];
  }
};
enum TaskState{
  STOP = 0,
  START,
  PAUSE,
  SENDGOAL,
  CHECKSTATE,
  CALLBACK,
  ACTIVE,
  IDEL
};
} // namespace reinovo
#endif
