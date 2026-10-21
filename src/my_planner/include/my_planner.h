#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace my_planner 
{
    class MyPlanner : public nav_core::BaseLocalPlanner 
    {
        public:
            MyPlanner();
            ~MyPlanner();

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool isGoalReached();

        private:
            ros::Subscriber adjust_sub_;
            ros::Subscriber pose_sub_;
            geometry_msgs::PoseStamped adjusted_goal_; // 调整后的目标点
            tf::TransformListener* tf_listener_; // 确保作为类成员存在
            bool adjusting_ = false;         // 微调标志位
            ros::Time start_adjust_time_; // 超时计时
            ros::Subscriber speed_sub_;
            void speedCallback(const std_msgs::String::ConstPtr& msg);
            void adjustCallback(const std_msgs::String::ConstPtr& msg);

            double speed_multiplier_;
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            int target_index_;
            bool pose_adjusting_;
            bool goal_reached_;
            double dist;
    };
} // namespace my_planner
 
#endif // MY_PLANNER_H_
