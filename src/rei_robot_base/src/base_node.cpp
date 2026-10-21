#include <rei_robot_base/rei_robot_base.h>
#include <rei_robot_base/rei_print.h>
#include <rei_robot_base/CarData.h>
#include <rei_robot_base/MotorCmd.h>
#include <rei_robot_base/BumperCliff.h>
#include <rei_robot_base/kinematics.h>
#include <rei_robot_base/SetIO.h>
#include <rei_robot_base/Int8.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include <ros/ros.h>
#include <string>



std::shared_ptr<reinovo_base::BaseKinematics> kinematic_ptr;
reinovo_base::RobotBaseCom& base_com = reinovo_base::RobotBaseCom::getInstance();

bool publis_odom = false;
bool publis_odom_tf = false;
std::vector<int> output_io;
std::string robot_model;
bool odomPubCommandCallback(std_srvs::SetBool::Request &req,
                            std_srvs::SetBool::Response &res){
    if(req.data){
        publis_odom = true;
    }else publis_odom = false;
    res.success = true;
    res.message = "ok";
    return true;
}

bool odomTfPubCommandCallback(std_srvs::SetBool::Request &req,
                            std_srvs::SetBool::Response &res){
    if(req.data){
        publis_odom_tf = true;
    }else publis_odom_tf = false;
    res.success = true;
    res.message = "ok";
    return true;
}
double x;
double y;
double th;
bool odomClearCallback(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &res){
    x = y = th =0.0; 
    return true;
}

void velCallback(const geometry_msgs::Twist::ConstPtr &msg){
    int err;
    base_com.sendSpeed(kinematic_ptr->InverseKinematics(msg->linear.x, msg->linear.y, msg->angular.z), &err);
}

void motorSpeedCallback(const rei_robot_base::MotorCmd::ConstPtr &msg){
    int err;
    base_com.sendSpeed(msg->motor_expect_speed, &err);    
}

bool IOCallback(rei_robot_base::SetIO::Request &req,
                            rei_robot_base::SetIO::Response &res){
    uint16_t data[7];
    if((robot_model!="bobac2")&&(robot_model!="fox")){
        if(output_io.size()==7){
            for (size_t i = 0; i < 7; i++)
            {
                data[i] = output_io[i];
            }
            
            if(req.all_on&&req.all_off){
                res.success = false;
                res.message = "could not set all_off and all_on both as true";
            }else{
                if(req.all_on){
                    for(size_t i = 0; i<7; i++) data[i] = 1;
                }else if(req.all_off){
                    for(size_t i = 0; i<7; i++) data[i] = 0;
                }else{
                    for (size_t i = 0; i < req.io.size(); i++)
                    {
                        if((req.io[i]>7)||req.io[i]<0) continue;
                        if(req.state) data[req.io[i]] = 1;
                        else data[req.io[i]] = 0;
                    }
                }
                int err;
                base_com.setIO(data, &err);
                if(err==0){
                    res.success = true;
                    res.message = "ok";
                }else{
                    res.success = false;
                    res.message = "get output err";
                }
            }
            
        }else{
            res.success = false;
            res.message = "set failed"; 
        }
    }else{
        res.success = false;
        res.message = robot_model+" is not support"; 
    }
    return true;
}

bool relayCallback(std_srvs::SetBool::Request &req,
                            std_srvs::SetBool::Response &res){
    int err;
    base_com.setRelay(req.data, &err);
    if(err==0){
        res.message = "ok";
        res.success = true;
    }else{
        res.message = "failed";
        res.success = false;
    }
    return true;
}
bool buzzerCallback(rei_robot_base::Int8::Request &req,
                            rei_robot_base::Int8::Response &res){
    int err;
    base_com.setBuzzer(req.data, &err);
    if(err==0){
        res.message = "ok";
        res.success = true;
    }else{
        res.message = "failed";
        res.success = false;
    }
    return true;
}

bool extraMotorCallback(rei_robot_base::Int8::Request &req,
                            rei_robot_base::Int8::Response &res){
    int err;
    base_com.setExtraMotor(req.data, &err);
    if(err==0){
        res.message = "ok";
        res.success = true;
    }else{
        res.message = "failed";
        res.success = false;
    }
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_base_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    
    std::vector<std::string> existed_robots;
    existed_robots.push_back("bobac2");
    existed_robots.push_back("bobac3");
    existed_robots.push_back("oryxbot");
    existed_robots.push_back("fox");
    if(!p_nh.getParam("robot_model", robot_model)) {
        ROS_ERROR_STREAM("未设置机器人型号");
        return 0;
    }else{
        for (size_t i = 0; i < existed_robots.size(); i++)
        {
            if(robot_model == existed_robots[i]) break;
            if(i == existed_robots.size()-1){
                ROS_ERROR_STREAM("未知机器人型号: "<<robot_model);
                return 0;
            }
        }
        ROS_INFO_STREAM("机器人型号: "<<robot_model);
    }

    if(!p_nh.getParam("port", base_com.communicate_params_.port)) {
        ROS_ERROR_STREAM("未设置串口号");
        return 0;
    }
    base_com.communicate_params_.parity = 'N';
    p_nh.param("baud", base_com.communicate_params_.baud, 115200);
    p_nh.param("data_bit", base_com.communicate_params_.data_bit, 8);
    p_nh.param("stop_bit", base_com.communicate_params_.stop_bit, 1);
    p_nh.param("slave", base_com.communicate_params_.slave, 1);
    int err;
    base_com.connect(&err);
    if(err < 0){
        ROS_ERROR_STREAM("下位机连接失败，请检查通信参数及硬件");
        return 0;
    }else ROS_INFO_STREAM("下位机连接成功");

    kinematic_ptr = std::make_shared<reinovo_base::BaseKinematics>();
    std::string odom_frame_id = "odom";
    std::string robot_base_frame_id = "base_footprint";
    std::string range_1_frame_id = "ultrasound1_link";
    std::string range_2_frame_id = "ultrasound2_link";
    std::string range_3_frame_id = "ultrasound3_link";

    std::string BC_1_frame_id = "bumper_cliff1_link";
    std::string BC_2_frame_id = "bumper_cliff2_link";
    std::string BC_3_frame_id = "bumper_cliff3_link";

    ros::Publisher car_bumperAndcliff1_pub;
    ros::Publisher car_bumperAndcliff2_pub;
    ros::Publisher car_bumperAndcliff3_pub;

    std::vector<std::string> range_frame_ids;
    std::vector<ros::Publisher> range_pubs;
    std::vector<rei_robot_base::BumperCliff> BC_msgs;

    if(robot_model=="bobac2"||robot_model=="bobac3"){
        p_nh.param("range_1_frame_id", range_1_frame_id, range_1_frame_id);
        range_frame_ids.push_back(range_1_frame_id);
        p_nh.param("range_2_frame_id", range_2_frame_id, range_2_frame_id);
        range_frame_ids.push_back(range_2_frame_id);
        range_pubs.push_back(nh.advertise<sensor_msgs::Range>("range1", 1));
        range_pubs.push_back(nh.advertise<sensor_msgs::Range>("range2", 1));
        if(robot_model=="bobac2"){
            p_nh.param("range_3_frame_id", range_3_frame_id, range_3_frame_id);
            range_frame_ids.push_back(range_3_frame_id);
            range_pubs.push_back(nh.advertise<sensor_msgs::Range>("range3", 1));
            
            p_nh.param("BC_1_frame_id", BC_1_frame_id, BC_1_frame_id);
            p_nh.param("BC_2_frame_id", BC_2_frame_id, BC_2_frame_id);
            p_nh.param("BC_3_frame_id", BC_3_frame_id, BC_3_frame_id);

            rei_robot_base::BumperCliff BC_msg;
            BC_msgs.push_back(BC_msg);
            BC_msgs.push_back(BC_msg);
            BC_msgs.push_back(BC_msg);
            
            car_bumperAndcliff1_pub = nh.advertise<rei_robot_base::BumperCliff>("bumpercliff1", 1, true);
            car_bumperAndcliff2_pub = nh.advertise<rei_robot_base::BumperCliff>("bumpercliff2", 1, true);
            car_bumperAndcliff3_pub = nh.advertise<rei_robot_base::BumperCliff>("bumpercliff3", 1, true);
        }
    }

    p_nh.param("odom_frame_id", odom_frame_id, odom_frame_id);
    p_nh.param("robot_base_frame_id", robot_base_frame_id, robot_base_frame_id);
    p_nh.param("publis_odom", publis_odom, false);
    p_nh.param("publis_odom_tf", publis_odom_tf, false);
    //get kinematics_mode
    if(!p_nh.getParam("kinematics_mode", kinematic_ptr->car_data_.car_mode)) {
        kinematic_ptr->car_data_.car_mode = reinovo_base::MotionModel::kNone;
        ROS_INFO("no kinematics_mode set");
        return 0;
    } else {
        ROS_INFO("kinematics_mode = %d", kinematic_ptr->car_data_.car_mode);
    }
    //get wheel_diameter and wheel_separation
    if(!p_nh.getParam("wheel_radius", kinematic_ptr->car_data_.wheel_radius)) {
        kinematic_ptr->car_data_.wheel_radius = default_wheel_radius;
        ROS_INFO("use default param \"wheel_radius = %g\"",kinematic_ptr->car_data_.wheel_radius);
    } else {
        ROS_INFO("wheel_radius = %g", kinematic_ptr->car_data_.wheel_radius);
    }
    if(!p_nh.getParam("wheel_separation_x", kinematic_ptr->car_data_.wheel_separation_x)) {
        kinematic_ptr->car_data_.wheel_separation_x = 0.1;
        ROS_INFO("use default param \"wheel_separation_x = %g\"",kinematic_ptr->car_data_.wheel_separation_x);
    } else {
        ROS_INFO("wheel_separation_x = %g", kinematic_ptr->car_data_.wheel_separation_x);
    }
    if(!p_nh.getParam("wheel_separation_y", kinematic_ptr->car_data_.wheel_separation_y)) {
        kinematic_ptr->car_data_.wheel_separation_y = 0.1;
        ROS_INFO("use default param \"wheel_separation_y = %g\"",kinematic_ptr->car_data_.wheel_separation_y);
    } else {
        ROS_INFO("wheel_separation_y = %g", kinematic_ptr->car_data_.wheel_separation_y);
    }
    //get max velocity
    if(!p_nh.getParam("max_vx", kinematic_ptr->car_data_.max_x_vel)) {
        kinematic_ptr->car_data_.max_x_vel = default_vx;
        ROS_INFO("use default param \"max_vx = %g\"",kinematic_ptr->car_data_.max_x_vel);
    } else {
        ROS_INFO("max_vx = %g", kinematic_ptr->car_data_.max_x_vel);
    }

    if(!p_nh.getParam("max_vy", kinematic_ptr->car_data_.max_y_vel)) {
        kinematic_ptr->car_data_.max_y_vel = default_vy;
        ROS_INFO("use default param \"max_vy = %g\"",kinematic_ptr->car_data_.max_y_vel);
    } else {
        ROS_INFO("max_vy = %g", kinematic_ptr->car_data_.max_y_vel);
    }

    if(!p_nh.getParam("max_vth", kinematic_ptr->car_data_.max_theta_vel)) {
        kinematic_ptr->car_data_.max_theta_vel = default_vth;
        ROS_INFO("use default param \"max_vth = %g\"",kinematic_ptr->car_data_.max_theta_vel);
    } else {
        ROS_INFO("max_vth = %g", kinematic_ptr->car_data_.max_theta_vel);
    }
    kinematic_ptr->init();
    ros::Publisher car_data_pub = nh.advertise<rei_robot_base::CarData>("car_data", 1, true);
    ros::Publisher car_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Publisher bumper_vel_pub = nh.advertise<geometry_msgs::Twist>("bumper_stop", 1);

    ros::Subscriber twist_sub = nh.subscribe("cmd_vel", 1, velCallback);
    ros::Subscriber motor_cmd_sub = nh.subscribe("motor_cmd", 1, motorSpeedCallback);

    ros::ServiceServer pub_odom_server = nh.advertiseService("pub_odom", odomPubCommandCallback);
    ros::ServiceServer pub_odom_tf_server = nh.advertiseService("pub_odom_tf", odomTfPubCommandCallback);
    ros::ServiceServer odom_clear_server = nh.advertiseService("clear_odom", odomClearCallback);
    ros::ServiceServer io_server = nh.advertiseService("set_io", IOCallback);
    ros::ServiceServer relay_server = nh.advertiseService("set_relay", relayCallback);
    ros::ServiceServer buzzer_server = nh.advertiseService("set_buzzer", buzzerCallback);
    ros::ServiceServer extra_motor_server = nh.advertiseService("set_extra_motor", extraMotorCallback);

    ros::Rate loop(20);
    std::vector<double> d_data;
    std::vector<float> f_data;
    std::vector<int> i_data;
    float fdata;
    int idata;
    bool bdata;

    sensor_msgs::Range range_msg;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.max_range = 3.0;
    range_msg.min_range = 0.2;
    range_msg.field_of_view = 0.523;
    
    sensor_msgs::BatteryState battery_msg;
    ros::Time current_time, last_time;
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.child_frame_id = "base_footprint";
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::Quaternion odom_quat;
    while (ros::ok())
    {
        int err;
        rei_robot_base::CarData car_data;
        base_com.readData(&err);
        if(!base_com.is_connected_) break;
        if(err==0){
            current_time = ros::Time::now();
            car_data.header.stamp = current_time;
            //read motor
            base_com.getMotorSpeed(&d_data, &err);
            if(err==0){
                car_data.motor_speed.assign(d_data.begin(), d_data.end());
                std::vector<double> real_vel_temp;
                real_vel_temp  = kinematic_ptr->ForwardKinematics(car_data.motor_speed);
                
                double dt = (current_time - last_time).toSec();
                double delta_x = (real_vel_temp[0] * cos(th) - real_vel_temp[1] * sin(th)) * dt;
                double delta_y = (real_vel_temp[0] * sin(th) + real_vel_temp[1] * cos(th)) * dt;
                double delta_th = real_vel_temp[2] * dt;

                x += delta_x;
                y += delta_y;
                th += delta_th;
                last_time = current_time;
                if(publis_odom_tf){
                    odom_trans.header.stamp = current_time;
                    odom_trans.header.frame_id = "odom";
                    odom_trans.child_frame_id = "base_footprint";
                    odom_trans.transform.translation.x = x;
                    odom_trans.transform.translation.y = y;
                    odom_quat = tf::createQuaternionMsgFromYaw(th);
                    odom_trans.transform.rotation = odom_quat;
                    odom_broadcaster.sendTransform(odom_trans);
                }
                if(publis_odom){
                    odom_msg.header.stamp = current_time;
                    odom_msg.pose.pose.position.x = x;
                    odom_msg.pose.pose.position.y = y;
                    odom_msg.pose.pose.position.z = 0.0;
                    odom_msg.pose.pose.orientation = odom_quat;
                    //set the velocity
                    odom_msg.twist.twist.linear.x = real_vel_temp[0];
                    odom_msg.twist.twist.linear.y = real_vel_temp[1];
                    odom_msg.twist.twist.angular.z = real_vel_temp[2];
                    //publish the message
                    car_odom_pub.publish(odom_msg);
                }

            } 

            //read ultrasound
            if(range_frame_ids.size()>0){
                base_com.getUltraSound(&f_data, &err);
                if(err==0){
                    for(size_t i=0; i<range_frame_ids.size(); i++){
                        car_data.ultrasound.push_back(f_data[i]);
                        if(f_data[i]<0) continue;
                        range_msg.header.stamp = current_time;
                        range_msg.header.frame_id = range_frame_ids[i];
                        range_msg.range = f_data[i]/1000.0f;
                        range_pubs[i].publish(range_msg);
                    }
                }
            }
            //read battery
            base_com.getPower(&fdata, &err);
            if(err==0){
                car_data.power_voltage = fdata;
            }
            base_com.getCharge(&idata, &err);
            if(err==0){
                car_data.is_charge = idata;
            }

            //read input io
            base_com.getInputIO(&i_data, &err);
            if(err==0){
                car_data.input_io[0] = i_data[0];
                car_data.input_io[1] = i_data[1];
                car_data.input_io[2] = i_data[2];
                car_data.input_io[3] = i_data[3];
            }

            //read smoke
            base_com.getSmoke(&idata, &err);
            if(err==0){
                car_data.smoke = idata;
            }

            //read Temperature
            base_com.getTemperature(&fdata, &err);
            if(err==0){
                car_data.tempareture = fdata;
            }

            //read relativehumidity
            base_com.getRelativeHumidity(&fdata, &err);
            if(err==0){
                car_data.relative_humidity = fdata;
            }

            //read bumper cliff
            if(robot_model=="bobac2"){
                BC_msgs[0].header.frame_id = BC_1_frame_id;
                BC_msgs[1].header.frame_id = BC_2_frame_id;
                BC_msgs[2].header.frame_id = BC_3_frame_id;

                BC_msgs[0].header.stamp = current_time;
                BC_msgs[1].header.stamp = current_time;
                BC_msgs[2].header.stamp = current_time;

                //get bumper
                bool bc_flag = false;

                base_com.getBumper(&i_data, &err);
                if(err == 0){
                    for(size_t i=0; i<i_data.size(); i++){
                        car_data.crash.push_back(i_data[i]);
                        if(i_data[i]==1) bc_flag = true;
                    }
                    
                }
                
                //get cliff
                base_com.getCliff(&i_data, &err);
                if(err == 0){
                    for(size_t i=0; i<i_data.size(); i++){
                        car_data.cliff.push_back(i_data[i]);
                        if(i_data[i]==1) bc_flag = true;
                    }
                }
                if(bc_flag){
                    geometry_msgs::Twist stop_vel;
                    bumper_vel_pub.publish(stop_vel);
                }
                for (size_t i = 0; i < car_data.crash.size(); i++)
                {
                    BC_msgs[i].bumper = car_data.crash[i];
                }
                for (size_t i = 0; i < car_data.cliff.size(); i++)
                {
                    BC_msgs[i].cliff = car_data.cliff[i];
                }
                car_bumperAndcliff1_pub.publish(BC_msgs[0]);
                car_bumperAndcliff2_pub.publish(BC_msgs[1]);
                car_bumperAndcliff3_pub.publish(BC_msgs[2]);
            }

            //read Relay
            base_com.getRelay(&bdata, &err);
            if(err==0) car_data.relay_status = bdata;

            //read output io
            base_com.getOutputIO(&output_io, &err);
            if(err==0){
                for (size_t i = 0; i < output_io.size(); i++)
                {
                    car_data.output_io[i] = output_io[i];
                }
            }
            car_data_pub.publish(car_data);
        }
        ros::spinOnce();
        loop.sleep();
    }
    
    return 0;
}
