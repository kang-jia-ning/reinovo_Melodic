#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Pose2D.h"
#include <cmath>

PLUGINLIB_EXPORT_CLASS( my_planner::MyPlanner, nav_core::BaseLocalPlanner)

double beijing_x, shanghai_x, guangzhou_x, shenzheng_x, jilin_x, charge_x, goal6_x;
double beijing_y, shanghai_y, guangzhou_y, shenzheng_y, jilin_y, charge_y, goal6_y;
double beijing_z, shanghai_z, guangzhou_z, shenzheng_z, jilin_z, charge_z, goal6_z; 
double beijing_w, shanghai_w, guangzhou_w, shenzheng_w, jilin_w, charge_w, goal6_w;

double beijing_pre_x, shanghai_pre_x, guangzhou_pre_x, shenzheng_pre_x, jilin_pre_x;
double beijing_pre_y, shanghai_pre_y, guangzhou_pre_y, shenzheng_pre_y, jilin_pre_y;
double beijing_pre_z, shanghai_pre_z, guangzhou_pre_z, shenzheng_pre_z, jilin_pre_z;
double beijing_pre_w, shanghai_pre_w, guangzhou_pre_w, shenzheng_pre_w, jilin_pre_w;

bool adjust_ = false;
std::string rage_mode;

void load_param() {
    ros::param::get("/rage_mode", rage_mode);

    ros::param::get("/beijing_x", beijing_x);
    ros::param::get("/beijing_y", beijing_y);
    ros::param::get("/beijing_z", beijing_z);
    ros::param::get("/beijing_w", beijing_w);

    ros::param::get("/shanghai_x", shanghai_x);
    ros::param::get("/shanghai_y", shanghai_y);
    ros::param::get("/shanghai_z", shanghai_z);
    ros::param::get("/shanghai_w", shanghai_w);

    ros::param::get("/guangzhou_x", guangzhou_x);
    ros::param::get("/guangzhou_y", guangzhou_y);
    ros::param::get("/guangzhou_z", guangzhou_z);
    ros::param::get("/guangzhou_w", guangzhou_w);

    ros::param::get("/shenzhen_x", shenzheng_x);
    ros::param::get("/shenzhen_y", shenzheng_y);
    ros::param::get("/shenzhen_z", shenzheng_z);
    ros::param::get("/shenzhen_w", shenzheng_w);

    ros::param::get("/jilin_x", jilin_x);
    ros::param::get("/jilin_y", jilin_y);
    ros::param::get("/jilin_z", jilin_z);
    ros::param::get("/jilin_w", jilin_w);

    ros::param::get("/charge_x", charge_x);
    ros::param::get("/charge_y", charge_y);
    ros::param::get("/charge_z", charge_z);
    ros::param::get("/charge_w", charge_w);

    ros::param::get("/goal6_x", goal6_x);
    ros::param::get("/goal6_y", goal6_y);
    ros::param::get("/goal6_z", goal6_z);
    ros::param::get("/goal6_w", goal6_w);

    ros::param::get("/beijing_pre_x", beijing_pre_x);
    ros::param::get("/beijing_pre_y", beijing_pre_y);
    ros::param::get("/beijing_pre_z", beijing_pre_z);
    ros::param::get("/beijing_pre_w", beijing_pre_w);

    ros::param::get("/shanghai_pre_x", shanghai_pre_x);
    ros::param::get("/shanghai_pre_y", shanghai_pre_y);
    ros::param::get("/shanghai_pre_z", shanghai_pre_z);
    ros::param::get("/shanghai_pre_w", shanghai_pre_w);

    ros::param::get("/guangzhou_pre_x", guangzhou_pre_x);
    ros::param::get("/guangzhou_pre_y", guangzhou_pre_y);
    ros::param::get("/guangzhou_pre_z", guangzhou_pre_z);
    ros::param::get("/guangzhou_pre_w", guangzhou_pre_w);

    ros::param::get("/shenzhen_pre_x", shenzheng_pre_x);
    ros::param::get("/shenzhen_pre_y", shenzheng_pre_y);
    ros::param::get("/shenzhen_pre_z", shenzheng_pre_z);
    ros::param::get("/shenzhen_pre_w", shenzheng_pre_w);

    ros::param::get("/jilin_pre_x", jilin_pre_x);
    ros::param::get("/jilin_pre_y", jilin_pre_y);
    ros::param::get("/jilin_pre_z", jilin_pre_z);
    ros::param::get("/jilin_pre_w", jilin_pre_w);
}


namespace my_planner 
{
    MyPlanner::MyPlanner(): speed_multiplier_(1.0)
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {}

 void MyPlanner::adjustCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("in_adjustCallback");
    if (msg->data == "1" && !global_plan_.empty()) {
        try {
            geometry_msgs::PoseStamped raw_goal = global_plan_.back();
            
            // 四元数归一化处理
            tf::Quaternion q(
                raw_goal.pose.orientation.x,
                raw_goal.pose.orientation.y,
                raw_goal.pose.orientation.z,
                raw_goal.pose.orientation.w);
            q.normalize();
            raw_goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, tf::getYaw(q));
            
            // 等待坐标变换可用性
            tf_listener_->waitForTransform("map", raw_goal.header.frame_id, 
                                         ros::Time(0), ros::Duration(1.0));
            
            geometry_msgs::PoseStamped map_goal;
            tf_listener_->transformPose("map", raw_goal, map_goal);
            adjusted_goal_ = map_goal;

	tf::Quaternion map_q;
	tf::quaternionMsgToTF(map_goal.pose.orientation, map_q);
	if (map_q.length2() < 1e-6) { // 检查四元数是否接近零
	    ROS_WARN("目标点四元数为零，使用默认朝向");
	    map_q.setRPY(0, 0, 0); // 初始化为单位四元数
	} else {
	    map_q.normalize(); // 强制归一化
	}
	tf::quaternionTFToMsg(map_q, adjusted_goal_.pose.orientation);

            ROS_INFO("微调目标点: (%.2f, %.2f)", 
                   adjusted_goal_.pose.position.x,
                   adjusted_goal_.pose.position.y);
            adjusting_ = true;
        } catch (tf::TransformException &ex) {
            ROS_ERROR("微调目标点转换失败: %s", ex.what());
        }
    }
}
    geometry_msgs::Pose2D now_pose;
    // 回调函数处理"add_vel"话题
    void MyPlanner::speedCallback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == "1") {
            speed_multiplier_ = 2.0; // 超速2倍
            ROS_WARN("超速模式激活！");
        } else if (msg->data == "2") {
            speed_multiplier_ = 1.0; // 恢复正常速度
            ROS_WARN("恢复正常速度。");
        }
    }

    tf::TransformListener* tf_listener_;
    tf::TransformListener listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        tf_listener_ = new tf::TransformListener();
        costmap_ros_ = costmap_ros;

        // 初始化订阅者
        ros::NodeHandle nh;
        speed_sub_ = nh.subscribe("add_vel", 10, &MyPlanner::speedCallback, this);
        adjust_sub_ = nh.subscribe("small_adjust", 10, &MyPlanner::adjustCallback, this);
        load_param();
        
        target_index_ = 0;
        pose_adjusting_ = false;
        goal_reached_ = false;
        dist = 0.0;
    }

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index_ = 0;
        global_plan_ = plan;
        pose_adjusting_ = false;
        goal_reached_ = false;
        return true;
    }

    double dist = 0.0;
    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        // 获取代价地图的数据
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        unsigned char* map_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        // 使用 OpenCV 绘制代价地图
        cv::Mat map_image(size_y, size_x, CV_8UC3, cv::Scalar(128, 128, 128));
        for (unsigned int y = 0; y < size_y; y++)
        {
            for (unsigned int x = 0; x < size_x; x++)
            {
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];               // 从代价地图数据取值
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(map_index);  // 获取彩图对应像素地址
                
                if (cost == 0)          // 可通行区域
                    pixel = cv::Vec3b(128, 128, 128); // 灰色
                else if (cost == 254)   // 障碍物
                    pixel = cv::Vec3b(0, 0, 0);       // 黑色
                else if (cost == 253)   // 禁行区域 
                    pixel = cv::Vec3b(255, 255, 0);   // 浅蓝色
                else
                {
                    // 根据灰度值显示从红色到蓝色的渐变
                    unsigned char blue = 255 - cost;
                    unsigned char red = cost;
                    pixel = cv::Vec3b(blue, 0, red);
                }
            }
        }


       // 在代价地图上遍历导航路径点
       for(int i=0;i<global_plan_.size();i++)
       {
           geometry_msgs::PoseStamped pose_odom;
           global_plan_[i].header.stamp = ros::Time(0);
           tf_listener_->transformPose("odom",global_plan_[i],pose_odom);
           double odom_x = pose_odom.pose.position.x;
           double odom_y = pose_odom.pose.position.y;
           double origin_x = costmap->getOriginX();
           double origin_y = costmap->getOriginY();
           double local_x = odom_x - origin_x;
           double local_y = odom_y - origin_y;
           int x = local_x / costmap->getResolution();
           int y = local_y / costmap->getResolution();
           cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(255,0,255));    // 导航路径点

	 // 检测前方路径点是否在禁行区域或者障碍物里global_plan_.size()-1
	   int forward_look_point = (global_plan_.size()-1) / 4;
	 if(i >= target_index_ && i < target_index_ + forward_look_point)
	 {
	     cv::circle(map_image, cv::Point(x,y), 0, cv::Scalar(0,255,255));// 检测路径点
	     int map_index = y * size_x + x;
	     unsigned char cost = map_data[map_index];
	     if(cost >= 253)
		 return false;
	 }
       }

        map_image.at<cv::Vec3b>(size_y/2, size_x/2) = cv::Vec3b(0, 255, 0); // 机器人位置

        // 翻转地图
        cv::Mat flipped_image(size_x, size_y, CV_8UC3, cv::Scalar(128, 128, 128));
        for (unsigned int y = 0; y < size_y; ++y)
        {
            for (unsigned int x = 0; x < size_x; ++x)
            {
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y, x);
                flipped_image.at<cv::Vec3b>((size_x - 1 - x), (size_y - 1 - y)) = pixel;
            }
        }
        map_image = flipped_image;

        // 显示代价地图
        cv::namedWindow("Map");
        cv::resize(map_image, map_image, cv::Size(size_y*5, size_x*5), 0, 0, cv::INTER_NEAREST);
        cv::resizeWindow("Map", size_y*5, size_x*5);
        //cv::imshow("Map", map_image);

        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final, pose_final_goal;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link",global_plan_[final_index],pose_final);

	pose_final.header.stamp = ros::Time(0);
	tf_listener_->transformPose("map",pose_final,pose_final_goal);

        if(pose_adjusting_ == false)
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            dist = std::sqrt(dx*dx + dy*dy);
            if(dist < 0.06)
                pose_adjusting_ = true;
        }
        if(pose_adjusting_ == true)
        {
	    double final_yaw = tf::getYaw(pose_final.pose.orientation);
	    bool adjust_yaw = true;
            ROS_WARN("调整最终姿态，final_yaw = %.2f",final_yaw);
            cmd_vel.linear.x = pose_final.pose.position.x * 0.5;
	    cmd_vel.linear.y = pose_final.pose.position.y * 0.5;
            
	    if (adjust_yaw) {
            if(abs(final_yaw) < 0.10) {
	        cmd_vel.angular.z = final_yaw * 0.65;
            } else if(abs(final_yaw) < 0.20)
            {
	        cmd_vel.angular.z = final_yaw * 0.8;
            } else if(abs(final_yaw) < 0.30){
                cmd_vel.angular.z = final_yaw * 1.2;
            } else {
                cmd_vel.angular.z = final_yaw * 1.4;
	    }
             }
	    
	    double torle_yaw;

	    if ( (std::fabs(pose_final_goal.pose.position.x - charge_x)<0.15)  && (std::fabs(pose_final_goal.pose.position.y - charge_y)<0.15) )   {
            torle_yaw = 0.10;
            } else {torle_yaw = 0.045;}

            if(abs(final_yaw) < torle_yaw)//关键微调部分代码！！！！
            {
	        bool adjust_yaw = false;
		tf::StampedTransform transform;

	        geometry_msgs::PoseStamped pose_final_map;
	        pose_final.header.stamp = ros::Time(0);
	        tf_listener_->transformPose("map",pose_final,pose_final_map);
		double adjusted_goal_x = pose_final_map.pose.position.x;
		double adjusted_goal_y = pose_final_map.pose.position.y;

		listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
		tf::Quaternion q = transform.getRotation();
		double roll, pitch, yaw;
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		now_pose.x = transform.getOrigin().x();
		now_pose.y = transform.getOrigin().y();

	        double error_x = adjusted_goal_x - now_pose.x;
	        double error_y = adjusted_goal_y - now_pose.y;

		bool beijing_flag = (std::fabs(adjusted_goal_x - beijing_pre_x) < 0.01)&&(std::fabs(adjusted_goal_y - beijing_pre_y) < 0.01);
		bool shanghai_flag = (std::fabs(adjusted_goal_x - shanghai_pre_x) < 0.01)&&(std::fabs(adjusted_goal_y - shanghai_pre_y) < 0.01);
		bool guangzhou_flag = (std::fabs(adjusted_goal_x - guangzhou_pre_x) < 0.01)&&(std::fabs(adjusted_goal_y - guangzhou_pre_y) < 0.01);
		bool shenzheng_flag = (std::fabs(adjusted_goal_x - shenzheng_pre_x) < 0.01)&&(std::fabs(adjusted_goal_y - shenzheng_pre_y) < 0.01);
		bool jilin_flag = (std::fabs(adjusted_goal_x - jilin_pre_x) < 0.01)&&(std::fabs(adjusted_goal_y - jilin_pre_y) < 0.01);
                bool charge_flag = (std::fabs(adjusted_goal_x - charge_x) < 0.01)&&(std::fabs(adjusted_goal_y - charge_y) < 0.01);

		adjust_ = (beijing_flag || shanghai_flag || guangzhou_flag || shenzheng_flag || jilin_flag || charge_flag);
		bool zero_point = (adjusted_goal_x * adjusted_goal_x + adjusted_goal_y * adjusted_goal_y)<0.25;

	        if (!adjust_) {//不是pre点时才微调
		    try {
			start_adjust_time_ = ros::Time::now();
			ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!start_adjusting!!!!!!!!!!!!!!!!!!!!!!!");
		        // 计算位置误差

			ROS_INFO("adjusted_goal_.pose.position.x = %f,adjusted_goal_.pose.position.y = %f ", adjusted_goal_x, adjusted_goal_y);
			ROS_INFO("now_pose.x = %f, now_pose.y = %f", now_pose.x, now_pose.y);
			ROS_INFO("error_x = %f, error_y = %f", error_x, error_y);

		        // P控制（比例系数建议0.3-0.5）
                        if( (std::fabs(error_x) < 0.01) && (std::fabs(error_y) < 0.01)) {
                        cmd_vel.linear.x = 0.487 * error_x;
		        cmd_vel.linear.y = 0.487 * error_y;
                        }
			else if( (std::fabs(error_x) < 0.02) && (std::fabs(error_y) < 0.02)) {
                        cmd_vel.linear.x = 0.495 * error_x;
		        cmd_vel.linear.y = 0.495 * error_y;
                        }
			else if( (std::fabs(error_x) < 0.025) && (std::fabs(error_y) < 0.025)) {
                        cmd_vel.linear.x = 0.51 * error_x;
		        cmd_vel.linear.y = 0.51 * error_y;
                        }
			else if( (std::fabs(error_x) < 0.03) && (std::fabs(error_y) < 0.03)) {
                        cmd_vel.linear.x = 0.53 * error_x;
		        cmd_vel.linear.y = 0.53 * error_y;
                        }
			else {
		        cmd_vel.linear.x = 0.53 * error_x;
		        cmd_vel.linear.y = 0.53 * error_y;
                        }
			
			bool x_yes, y_yes;

			if (zero_point) {
			    x_yes = std::fabs(error_x) < 0.00123;
			    y_yes = std::fabs(error_y) < 0.00123;
			} else {
			    // x_yes = std::fabs(error_x) < 0.00435;
			    // y_yes = std::fabs(error_y) < 0.00435;
			    x_yes = std::fabs(error_x) < 0.0095;
			    y_yes = std::fabs(error_y) < 0.0095;
			}

		        if (x_yes && y_yes) { 
			    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!end_adjusting!!!!!!!!!!!!!!!!!!!!!!!");
		            adjusting_ = false;
		            goal_reached_ = true;
		            cmd_vel.linear.x = 0;
		            cmd_vel.linear.y = 0;
		            cmd_vel.angular.z = 0;
		            return true;
		        }
		        
		        // 超时处理
		        if ((ros::Time::now() - start_adjust_time_).toSec() > 8.0) {
			    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!end_adjusting!!!!!!!!!!!!!!!!!!!!!!!");
		            adjusting_ = false;
		            goal_reached_ = true;
		            cmd_vel.linear.x = 0;
		            cmd_vel.linear.y = 0;
		            cmd_vel.angular.z = 0;
		            ROS_WARN("微调超时");
                            return true;
		        }
		        
		        return true;
		    } catch (tf::TransformException &ex) {
		        ROS_ERROR("TF转换失败: %s", ex.what());
		        return false;
		    }
		}

		        goal_reached_ = true;
		        ROS_WARN("到达终点！");
		        cmd_vel.linear.x = 0;
		        cmd_vel.linear.y = 0;
		        cmd_vel.angular.z = 0;
		    }
		    return true;
		}

        geometry_msgs::PoseStamped target_pose;
        for(int i=target_index_;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

	    double tolr_dist = 0.38;

	    if (0) {
                tolr_dist = 0.43;}
            else {
                tolr_dist = 0.38;}

            if (dist > tolr_dist) 
            {
                target_pose = pose_base;
                target_index_ = i;
                ROS_WARN("选择第 %d 个路径点作为临时目标，距离=%.2f",target_index_,dist);
                break;
            }

            if(i == global_plan_.size()-1)
                target_pose = pose_base; 
        }

        if (adjust_) {
        cmd_vel.linear.x = target_pose.pose.position.x * 2.75;
        cmd_vel.linear.y = target_pose.pose.position.y * 2.75;
        } else {
        cmd_vel.linear.x = target_pose.pose.position.x * 3.9;
        cmd_vel.linear.y = target_pose.pose.position.y * 3.9;
        }

        cmd_vel.linear.x *= speed_multiplier_;
        cmd_vel.linear.y *= speed_multiplier_;

        cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));        
        for(int i=0;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
            int cv_x = 300 - pose_base.pose.position.y*100;
            int cv_y = 300 - pose_base.pose.position.x*100;
            cv::circle(plan_image, cv::Point(cv_x,cv_y), 1, cv::Scalar(255,0,255)); 
        }
        cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
        cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
        cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);

        // cv::namedWindow("Plan");
        // cv::imshow("Plan", plan_image);
        cv::waitKey(1);
        
        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
} // namespace my_planner
