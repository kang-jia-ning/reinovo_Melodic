#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>

class AutoCostmapClearer {
public:
    AutoCostmapClearer() : nh_("~"), last_clear_time_(0), cool_down_(2.0) {
        clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base_node/clear_costmaps");
        
        path_sub_ = nh_.subscribe("/move_base_node/NavfnROS/plan", 1, 
                                &AutoCostmapClearer::pathCallback, this);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 检测空路径并排除初始状态（header.seq == 0）
        if (msg->poses.empty() && msg->header.seq != 0) {
            ros::Time now = ros::Time::now();
            
            if ((now - last_clear_time_).toSec() > cool_down_) {
                ROS_WARN("Empty global path detected! Clearing costmaps...");
                
                std_srvs::Empty srv;
                if (clear_costmaps_client_.call(srv)) {
                    ROS_INFO("Costmaps cleared successfully");
                    last_clear_time_ = now;
                    // 动态调整冷却时间（上限30秒）
                    cool_down_ = std::min(cool_down_ * 1.5, 30.0);
                } else {
                    ROS_ERROR("Failed to call clear_costmaps service");
                    cool_down_ = std::max(cool_down_ * 0.8, 0.5);
                }
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient clear_costmaps_client_;
    ros::Subscriber path_sub_;
    ros::Time last_clear_time_;
    double cool_down_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_costmap_cleaner");
    AutoCostmapClearer clearer;
    ros::spin();
    return 0;
}


