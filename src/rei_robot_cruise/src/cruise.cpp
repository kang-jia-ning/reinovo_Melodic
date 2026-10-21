#include <rei_robot_cruise/cruise.h>

namespace rei_cruise
{
RobotCruise::RobotCruise()
      : _mode_(TaskState::IDEL),
     _loopTimes_(1),
     _moveBaseClientPtr_(new MOVEBASE("move_base", false)),
     _taskState_(0),
     _flagPublishMarker_(false),
     _currentNavingGoal_(0),
     _moveBaseName_("move_base") {
}
RobotCruise::~RobotCruise(){
  ROS_WARN("xx");
  this->Stop();
}
// void RobotCruise::_DynamicConfigCallback(robot_cruise::navigatorConfig &config, uint32_t level){

// }
bool RobotCruise::Init(ros::NodeHandle &nh){
  nh_ = nh;
  _setGoalsServer_ = nh_.advertiseService("set_goals", &RobotCruise::_SetGoalsCallback, this);
  _startStopNavServer_ = nh_.advertiseService("start_stop_nav", &RobotCruise::_StartStopGoalsCallback, this);
  _setGoalsOrderServer_ = nh_.advertiseService("set_goals_order", &RobotCruise::_SetGoalsOrderCallback, this);
  _removeGoalsServer_ = nh_.advertiseService("remove_goals", &RobotCruise::_RemoveGoalCallback, this);
  ros::SubscriberStatusCallback pub_marker_cb = std::bind( [&]{
    if (_markerPublisher_.getNumSubscribers()>0)
    {
        _flagPublishMarker_ = true;
        ROS_INFO_STREAM("Starting publish goal marker.");
    }
    else 
    {
        _flagPublishMarker_ = false;
        ROS_INFO_STREAM("Stopping publish goal marker.");
    }
  });

  _markerPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("goals_markers", 1, pub_marker_cb, pub_marker_cb);

  _marker_.header.frame_id = "map";
  _marker_.header.stamp = ros::Time::now();
  _marker_.color.r = 1.0f;
  _marker_.color.g = 0.0f;
  _marker_.color.b = 0.0f;
  _marker_.color.a = 0.5;
  _marker_.lifetime = ros::Duration(3.0);//每个模型的生命周期为3秒
  _marker_.action = visualization_msgs::Marker::ADD;

  _callbackMap_.emplace("default", []()->bool{ return true; });
  _startCallback_ = []()->bool{ return true; };
  return true;
}
void RobotCruise::_SetLandMark(geometry_msgs::Pose pose, std::string goal_name){

  visualization_msgs::MarkerArray markers;
  //该模型是由箭头和方块组合而来的，所有其实有一个目标点模型其实有两个marker
  _marker_.pose = pose;

  _marker_.type = visualization_msgs::Marker::CUBE;
  _marker_.scale.x = 0.2;
  _marker_.scale.y = 0.2;
  _marker_.scale.z = 0.2;
  _marker_.ns = goal_name+"_0";
  markers.markers.push_back(_marker_);

  _marker_.type = visualization_msgs::Marker::ARROW;
  _marker_.scale.x = 0.5;
  _marker_.scale.y = 0.1;
  _marker_.scale.z = 0.1;
  _marker_.ns = goal_name+"_1";
  markers.markers.push_back(_marker_);

  _landmarkersMap_[goal_name] = markers;
}

/*_setGoalsCallback函数逻辑
    检查当前任务是否停止
    如果req.mode为reset即重置所有目标点——清空goals_list_、landmarkers_map_、executable_order_
    依次读取目标点存入goals_list_、landmarkers_map_中，已有的名字会覆盖掉对应坐标信息
*/
bool RobotCruise::_SetGoalsCallback(rei_robot_cruise::SetGoals::Request &req,
                                    rei_robot_cruise::SetGoals::Response &res){
  setlocale(LC_CTYPE, "zh_CN.utf8");
  ROS_INFO("调用set_goals服务");
  if(_taskState_!= TaskState::STOP){
    ROS_WARN("please stop the task");
    res.isSuccess = false;
    return true;
  }
  if(req.mode == "reset"){
    if(!_goalsList_.empty()) _goalsList_.clear();//内存回收由map自身决定，频繁操作可能会造成内存泄漏
    if(!_executableOrder_.empty()) _executableOrder_.clear();
    if(!_landmarkersMap_.empty()) _landmarkersMap_.clear();
      
  }else if(req.mode != "insert"){
    res.isSuccess = false;
    ROS_ERROR("模式指令错误, 可识别的指令为: insert, reset");
  }
  for(auto const &goal: req.goals)
  {
    auto ret = _goalsList_.emplace(goal.point_name, goal);
    if(!ret.second){
      _goalsList_.find(goal.point_name)->second = goal;
    }
    _SetLandMark(goal.target_pose.pose, goal.point_name);
  }
  res.isSuccess = true;
  return true;
}

/*
设置巡航顺序，并将巡航计划内的目标点模型变为绿色
*/
bool RobotCruise::_SetGoalsOrderCallback(rei_robot_cruise::SetNavOrder::Request &req,
                                       rei_robot_cruise::SetNavOrder::Response &res) {
  setlocale(LC_CTYPE, "zh_CN.utf8");
  ROS_INFO("调用set_goal_order服务");
  if (_taskState_!= TaskState::STOP) {
    ROS_WARN("please stop the task");
    res.isSuccess = false;
    return true;
  }
  _requestLoop_ = req.loop_time;
  _loopTimes_ = _requestLoop_;
  std_msgs::ColorRGBA green_color;
  green_color.r = 0.0f;
  green_color.g = 1.0f;
  green_color.b = 0.0f;
  green_color.a = 0.8;
  if (!req.goal_order.empty()) {
    std::vector<std::string> unkown_goals;
    std::vector<std::string> unkown_cbs;
    for (auto const &goal: req.goal_order) { 
      auto iter = _goalsList_.find(goal);
      if(iter == _goalsList_.end()) unkown_goals.push_back(goal);
    }
    for (auto const &goal_cb: req.callback_order){
      auto cb_iter = _callbackMap_.find(goal_cb);
      if(cb_iter == _callbackMap_.end()) unkown_cbs.push_back(goal_cb);
    }
    if (unkown_goals.empty()&&unkown_cbs.empty()) {
      std::map<int,std::string> empty_order_map;
      std::map<int,std::function<bool()>> empty_order_cb;
      _executableOrder_.swap(empty_order_map);
      _executableOrderCb_.swap(empty_order_cb);
      _executableOrder_.clear();
      _executableOrderCb_.clear();
      for (size_t i = 0; i < req.goal_order.size(); i++) {
        _executableOrder_[i] = req.goal_order[i];
        _executableOrderCb_[i] = _callbackMap_["default"];
        _landmarkersMap_[_executableOrder_[i]].markers[0].color = green_color;
        _landmarkersMap_[_executableOrder_[i]].markers[1].color = green_color;
      }
      for (size_t i = 0; i < req.callback_order.size(); i++) {
        _executableOrderCb_[i] = _callbackMap_[req.callback_order[i]];
      }
      res.isSuccess = true;
    } else {
      res.isSuccess = false;
      if (!unkown_goals.empty()) {
        ROS_ERROR("存在未知目标点");
        res.message = "There are some unkown goals: ";
        for (auto const &unkown_goal: unkown_goals) {
          res.message = res.message + unkown_goal+ std::string("; ");
        }
        res.message = res.message + std::string("\n");
        return true;
      }
      if (!unkown_cbs.empty()) {
        ROS_ERROR("存在未知回调任务名");
        res.message = "There are some unkown callback: ";
        for (auto const &unkown_cb: unkown_cbs) {
          res.message = res.message + unkown_cb+ std::string("; ");
        }
        res.message = res.message + std::string("\n");
        return true;
      }
    }
  }
  return true;

}
bool RobotCruise::_RemoveGoalCallback(rei_robot_cruise::RemoveGoal::Request &req,
                                       rei_robot_cruise::RemoveGoal::Response &res){
  setlocale(LC_CTYPE, "zh_CN.utf8");
  if(_taskState_!= TaskState::STOP){
    ROS_ERROR("没有正在执行或暂停中的任务");
    res.isSuccess = false;
    return true;
  }
  if(req.mode=="all"){//req.mode为all则清空所有目标点
    std::unordered_map<std::string, rei_robot_cruise::NavGoal> empty_goals;
    std::map<int,std::string> empty_order_map;
    std::map<int,std::function<bool()>> empty_order_cb;
    std::map<std::string, visualization_msgs::MarkerArray> empty_landmark;
    _executableOrder_.swap(empty_order_map);
    _executableOrderCb_.swap(empty_order_cb);
    _goalsList_.swap(empty_goals);
    _landmarkersMap_.swap(empty_landmark);
    _goalsList_.clear();
    _executableOrder_.clear();
    _executableOrderCb_.clear();
    _landmarkersMap_.clear();
  }else if(req.mode=="not_all"){//如果是not_all,则分别查找并删除goals_list_、landmarkers_map_以及executable_order_中对应的点
    if(!req.goal_names.empty()){
      for(auto const &goal_name: req.goal_names){
        auto goal_iter = _goalsList_.find(goal_name);
        if(goal_iter != _goalsList_.end()){
          _goalsList_.erase(goal_iter);
          _landmarkersMap_.erase(_landmarkersMap_.find(goal_name));
          for (auto order_iter = _executableOrder_.begin(); order_iter != _executableOrder_.end();)
          {
            if(order_iter->second == goal_name){
              _executableOrderCb_.erase(order_iter->first);
              _executableOrder_.erase(order_iter++);
            }else order_iter++;
          }
          ROS_INFO("已删除目标点: %s",goal_name.c_str());
        }else ROS_WARN("没有找到目标点: %s",goal_name.c_str());
        
      }
      res.isSuccess = true;
    }else ROS_WARN("no goal removed ");
  }else{
    res.isSuccess = false;
    ROS_ERROR("remove mode is error, which given param: %s", req.mode.c_str());
  }
  return true;
}
bool RobotCruise::_StartStopGoalsCallback(rei_robot_cruise::StartStopNav::Request &req,
                                       rei_robot_cruise::StartStopNav::Response &res){
  setlocale(LC_CTYPE, "zh_CN.utf8");
  std::unique_lock<std::mutex> myLock(mtx_);
  if(req.mode=="start"){
    if(this->Start()) res.isSuccess = true;
  }else if (req.mode=="pause"){
    if(this->Pause()) res.isSuccess = true;
  }else if(req.mode=="stop"){
    if(this->Stop()) res.isSuccess = true;
  }else {
    ROS_WARN("输入的指令错误, 仅可识别: _start、stop、pause");
  }
  return true;
}

bool RobotCruise::Start(){
  setlocale(LC_CTYPE, "zh_CN.utf8");
  if(_taskState_== TaskState::ACTIVE){
    return false;
    ROS_WARN("已有任务正在执行...");
  }
  if(_executableOrder_.empty()){
    ROS_ERROR("需导航路点列表为空，请先调用'set_goal_order'服务设置导航顺序");
    return false;
  }else{
    if(!_moveBaseClientPtr_->waitForServer(ros::Duration(10.0))) {
      ROS_ERROR("没有检测到move_base服务器: %s", _moveBaseName_.c_str());
      return false;
    }
    if(!_startCallback_()){
      ROS_ERROR("起始回调函数返回失败");
      return false;
    }
    _mode_ = TaskState::SENDGOAL;
    ROS_INFO("开始巡航任务");
  }
  return true;
}
bool RobotCruise::Pause(){
  setlocale(LC_CTYPE, "zh_CN.utf8");
  if(_taskState_== TaskState::ACTIVE){
    ROS_INFO("暂停巡航任务");
    _moveBaseClientPtr_->cancelGoal();
    _taskState_ = TaskState::PAUSE;
    _mode_ = TaskState::IDEL;
  }else{
    ROS_WARN("没有正在执行的任务");
    return false;
  }
  return true;
}
bool RobotCruise::Stop(){
  if(_taskState_ == TaskState::ACTIVE){
    _moveBaseClientPtr_->cancelGoal();
  }
  _currentNavingGoal_ = 0;
  _loopTimes_ = _requestLoop_;
  _taskState_ = TaskState::STOP;
  _mode_ = TaskState::IDEL;
  ROS_INFO("停止任务");
  return true;
}

void RobotCruise::RunNavTask(){
  ros::Rate loop_rate(2);
  setlocale(LC_CTYPE, "zh_CN.utf8");
  ros::NodeHandle nh("~");
  nh.param<bool>("read_params", _readParams_, false);
  
  if(_readParams_){
    ROS_INFO("read_params为true, 开始读取参数");
    ros::NodeHandle cruiseNh(nh_, "robot_cruise_goal");
    cruiseNh.param("loop_time", _requestLoop_, 0);
    _loopTimes_ = _requestLoop_;
    ROS_INFO("loop_time(param): %d", _requestLoop_);
    std::string pose_name;
    cruiseNh.param("goals_name", pose_name, std::string(""));
    ROS_INFO("goals_name(param): %s", pose_name.c_str());
    std::stringstream goalSs(pose_name);
    std::string goal;
    while(goalSs>>goal)
    {
      ros::NodeHandle goal_nh(cruiseNh, goal);
      ROS_INFO("goal: %s", goal.c_str());
      double x, y, theta;
      goal_nh.param("x", x, 0.0);
      ROS_INFO("x: %lf", x);

      goal_nh.param("y", y, 0.0);
      ROS_INFO("y: %lf", y);

      goal_nh.param("theta", theta, 0.0);
      ROS_INFO("theta: %lf", theta);

      double euler[3] = {0.0, 0.0, theta};
      std::string frame_id;
      goal_nh.param("frame_id", frame_id, std::string("map"));
      ROS_INFO("frame_id: %s", frame_id.c_str());

      rei_robot_cruise::NavGoal info;
      info.point_name = goal;
      info.target_pose.header.frame_id = frame_id;
      info.target_pose.pose.position.x = x;
      info.target_pose.pose.position.y = y;
      info.level = 1;
      this->_EulerToQuart(euler, info.target_pose.pose.orientation);
      _goalsList_[goal] = info;
      
      _SetLandMark(info.target_pose.pose, info.point_name);
    }
      
    std_msgs::ColorRGBA green_color;
    green_color.r = 0.0f;
    green_color.g = 1.0f;
    green_color.b = 0.0f;
    green_color.a = 0.8;
    _executableOrder_.clear();
    
    std::string navOrder;
    cruiseNh.param("nav_order", navOrder, std::string(""));
    ROS_INFO("nav_order(param): %s", navOrder.c_str());

    std::stringstream order_ss(navOrder);
    std::string order;

    std::string callbacks;
    cruiseNh.param("callback", callbacks, std::string(""));
    ROS_INFO("callback(param): %s", callbacks.c_str());
    std::stringstream cb_ss(callbacks);
    int mindex = 0;
    while(order_ss>>order){
      ROS_INFO("goal %d(param): %s", mindex, order.c_str());
      auto iter = _goalsList_.find(order);
      if(iter != _goalsList_.end()){
        std::string cb;
        if(cb_ss>>cb){
          ROS_INFO("callback(param): %s", cb.c_str());
          auto cb_iter = _callbackMap_.find(cb);
          if(cb_iter != _callbackMap_.end()) _executableOrderCb_[mindex] = _callbackMap_[cb];
          else{
            ROS_WARN("未检测到函数: %s, 当前点无回调任务,回调任务默认设为默认回调函数default", cb.c_str());
            _executableOrderCb_[mindex] = _callbackMap_["default"];
          }
        }
        else{
          ROS_WARN("当前点(%s)未设置回调任务，回调任务默认设为默认回调函数default", order.c_str());
          _executableOrderCb_[mindex] =_callbackMap_["default"];
        }
        _executableOrder_[mindex] = order;
        _landmarkersMap_[_executableOrder_[mindex]].markers[0].color = green_color;
        _landmarkersMap_[_executableOrder_[mindex]].markers[1].color = green_color;
        mindex++;
      }
    }
    cruiseNh.param("loop_time", _requestLoop_, 0);
    _loopTimes_ = _requestLoop_;
    ROS_INFO("loop_time(param): %d", _requestLoop_);
  }
  ros::AsyncSpinner spinner(2);
  //开始多线程接收
  spinner.start();
  while(ros::ok()){
    move_base_msgs::MoveBaseGoal goal;
    if(_flagPublishMarker_){
        visualization_msgs::MarkerArray landmarks;
        //当所有目标点模型放入markerarry中
        for(auto iter = _landmarkersMap_.begin(); iter!=_landmarkersMap_.end(); iter++){
          landmarks.markers.push_back(iter->second.markers[0]);
          landmarks.markers.push_back(iter->second.markers[1]);
        }
        _markerPublisher_.publish(landmarks);
    }
    ROS_INFO_THROTTLE(20, "_mode_: %d, _taskState_: %d", _mode_, _taskState_);
    switch (_mode_){//当执行SENDGOAL和CHECKSTATE时 _taskState_ 状态都为ACTIVE
      case TaskState::SENDGOAL://发送目标点
        goal.target_pose = _goalsList_[_executableOrder_[_currentNavingGoal_]].target_pose;
        if(_moveBaseClientPtr_->waitForServer(ros::Duration(10.0))){//检查move_base是否运行
          ROS_INFO("发送目标点(%s)导航指令", _executableOrder_[_currentNavingGoal_].c_str());
          _moveBaseClientPtr_->sendGoal(goal);
          _mode_ = TaskState::CHECKSTATE;
          _taskState_ = TaskState::ACTIVE;
        }else{
          ROS_ERROR("没有检测到move_base服务器: %s", _moveBaseName_.c_str());
          this->Stop();
        }
        break;
      case TaskState::CHECKSTATE://检查当前目标点导航状态
        if(_moveBaseClientPtr_->waitForResult(ros::Duration(1.0))){
          if(_moveBaseClientPtr_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("goal %s reached", _executableOrder_[_currentNavingGoal_].c_str());
            _mode_ = TaskState::CALLBACK;
          }else if(_moveBaseClientPtr_->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
            this->Pause();//如果目标点被外部其他导航指令占用，则暂停
          }else if(_moveBaseClientPtr_->getState() == actionlib::SimpleClientGoalState::ABORTED){//导航失败
            if(_goalsList_[_executableOrder_[_currentNavingGoal_]].level == 1){//如果当前导航点为重要点，则暂停
              this->Pause();
            }else _mode_ = TaskState::SENDGOAL;//如果为可忽略点，则跳过开始导航到下一个目标点
          }
        }
        break;
      case TaskState::CALLBACK:
        if(!_executableOrderCb_[_currentNavingGoal_]()){
          ROS_WARN("当前点回调任务失败");
          if(_goalsList_[_executableOrder_[_currentNavingGoal_]].level==1){
            this->Pause();
            break;
          }
        }
        if(_taskState_ !=TaskState::ACTIVE) break;
        if(_currentNavingGoal_<_executableOrder_.size()-1){
          _currentNavingGoal_++;
          _mode_ = TaskState::SENDGOAL;//导航成功后开始导航到下一个目标点
        }else{
          _currentNavingGoal_ = 0;
          _loopTimes_--;
          if(_loopTimes_==0){
            this->Stop();
          }else{
            _mode_ = TaskState::SENDGOAL;
          }
        }
        break;
    }
   // ros::spinOnce();
    loop_rate.sleep();
  }
}
} // namespace reinovo
