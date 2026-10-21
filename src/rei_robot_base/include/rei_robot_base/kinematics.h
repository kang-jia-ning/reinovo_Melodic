/*
 * @Author: Zhaoq 1327153842@qq.com
 * @Date: 2022-09-08 11:30:39
 * @LastEditors: Zhaoq 1327153842@qq.com
 * @LastEditTime: 2022-09-13 19:00:55
 * @Gitee: https://gitee.com/REINOVO
 * Copyright (c) 2022 by 深圳市元创兴科技, All Rights Reserved. 
 * @Description: 机器人运动学解算
 */
#ifndef _REINOVO_KINEMATICS_H
#define _REINOVO_KINEMATICS_H

#include <iostream>
#include <string>
#include <vector>
#include <complex>
const double PI = std::acos(-1);

#define default_kinematics_mode (2)

#define default_wheel_radius (0.25)
#define default_wheel_separation (0.5)
#define default_vx (0.5)
#define default_vy (0.5)
#define default_vth (1.0)


namespace reinovo_base
{
/**
 * @brief 枚举 底盘运动模型
 * @param kNone 无 1
 * @param kDiff 两轮差动底盘 2
 * @param kThreeWheeledOmni 三轮全向轮底盘 3
 * @param kFourWheeledOmni 四轮全向轮底盘 4
 * @param kFourMecanumOmni 四轮麦克纳姆轮底盘(长方形) 5
 * @param kFourWheelSlip 四轮滑移底盘 6
 * @param kAckermannSteering 阿克曼底盘 7
 */
enum MotionModel{
    kNone = 1, 
    kDiff,
    kThreeWheeledOmni,
    kFourWheeledOmni,
    kFourMecanumOmni,
    kFourWheelSlip,
    kAckermannSteering
};

/**
 * @brief 结构体 底盘运动参数
 * @param car_mode 底盘运动模型 
 * @param wheel_separation_x x方向轮间距 
 * @param wheel_separation_y y方向轮间距
 * @param wheel_radius 主动轮半径
 * @param max_x_vel 允许最大x方向线速度
 * @param max_y_vel 允许最大y方向线速度
 * @param max_theta_vel 允许最大绕z轴角速度
 * 
 */
struct CarData
{
    int car_mode;
    double wheel_separation_x;//x方向轮间距
    double wheel_separation_y;//y方向轮间距
    double wheel_radius;
    double max_x_vel;
    double max_y_vel;
    double max_theta_vel;
    /* data */
};

/**
 * @brief 运动学解算类 调用流程：实例化类 --> 设置运动底盘参数car_data_ --> init() --> 正解或逆解
 * 
 */
class BaseKinematics
{
public:
    /**
     * @brief 构造函数
     * @return {*}
     */
    BaseKinematics();

    /**
     * @brief 初始化 并检查是否正确设置运动底盘参数
     * @return {bool}设置正确返回true 设置错误或未设置返回false
     */
    bool init();

    /**
     * @brief 运动学正解 各个电机转速解算为底盘速度 速度单位(m/s、rad/s) 转速单位 转/min
     * @param {vector<double>} motor_speed 电机转速
     * @return {vector<double>} 运动底盘速度 [运动底盘x方向线速度，运动底盘y方向线速度，运动底盘绕z轴角速度]
     */
    std::vector<double> ForwardKinematics(std::vector<double> motor_speed);
    /**
     * @brief: 运动学逆解 速度转换为各个电机转速 速度单位(m/s、rad/s) 转速单位 转/min
     * @param {double} vx 运动底盘x方向线速度
     * @param {double} vy 运动底盘y方向线速度
     * @param {double} vth 运动底盘绕z轴角速度
     * @return {vector<double>} 各个电机转速
     */
    std::vector<double> InverseKinematics(double vx, double vy, double vth);
    CarData car_data_;//运动底盘参数
private: 
    bool init_ok;//初始化完成标志
};
}

#endif
