/*
 * @Author: Zhaoq 1327153842@qq.com
 * @Date: 2022-09-06 13:44:44
 * @LastEditors: Zhaoq 1327153842@qq.com
 * @LastEditTime: 2022-12-01 17:36:10
 * @Gitee: https://gitee.com/REINOVO
 * Copyright (c) 2022 by 深圳市元创兴科技, All Rights Reserved. 
 * @Description: reinovo Standardized robot base communication tools，applies to :
 * bobac2 bobac3 oryxbot fox
 */
#ifndef __REINOVO_ROBOT_BASE_H_
#define __REINOVO_ROBOT_BASE_H_

#include <modbus/modbus-rtu.h>
#include <memory>
#include <iostream>
#include <vector>

#define VERSION_COMPUTE(MAJ, MIN, PATCH) (MAJ*100+MIN*10+PATCH)
#define MODBUS_VERION_COMPARE(OP, MAJ, MIN, PATCH) \
    (VERSION_COMPUTE(LIBMODBUS_VERSION_MAJOR, LIBMODBUS_VERSION_MINOR, LIBMODBUS_VERSION_MICRO) OP VERSION_COMPUTE(MAJ, MIN, PATCH))

namespace reinovo_base
{
/**
 * @brief 枚举 各数据起始地址
 * @param 输入寄存器：
 * @param MotorSpeed 电机当前速度 0
 * @param Charge 充电状态 16
 * @param Power 电池电压 17
 * @param InputIO 输入io状态 18
 * @param UltruaSound 超声波数据 19
 * @param Smoke 烟雾传感器数据 22
 * @param Temperature 温度传感器数据 23
 * @param RelativeHumidity 湿度传感器数据 25
 * 
 * @param 保存寄存器：
 * @param MotorCmd 下发给电机的目标转速 0
 * @param Buzzer 蜂鸣器状态 16 
 * @param Relay 继电器状态 17
 * @param OutputIO 输出io 18
 * @param ExtraMotor 扩展电机状态 19
 * 
 */
enum DataAdress{
  MotorSpeed = 0,
  Charge = 16, 
  Power = 17,
  InputIO = 18,
  UltruaSound = 19,
  Smoke = 22,
  Temperature = 23,
  RelativeHumidity = 25,

  MotorCmd = 0,
  Buzzer = 16,
  Relay = 17,
  OutputIO = 18,
  ExtraMotor = 19
};

/**
 * @brief 结构体 下位机通信参数
 * @param {string} port 串口号
 * @param {int} baud 波特率
 * @param {char} parity 标志位
 * @param {int} data_bit 数据位
 * @param {int} stop_bit 停止位
 * @param {int} slave 从机地址
 */
struct BaseComParams
{
    std::string port;
    int baud;
    char parity;
    int data_bit;
    int stop_bit;
    int slave;
};

/**
 * @brief 下位机通信类，RobotBaseCom调用流程：实例化类->设置连接参数communicate_params_->connect()
 * |
 * -->获取下位机各数据时前需要readData(每帧只用调一次)
 * |
 * -->下发控制指令
 */
class RobotBaseCom{
public:
    /**
     * @brief 连接下位机 注：需设置好连接参数后再调用 
     * @param {int*} err 返回错误码 0表示成功 -1表示连接失败
     * @return {*}
     */
    void connect(int* err);

    /**
     * @brief 读取一帧下位机数据
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void readData(int* err);

    /**
     * @brief 发送电机控制指令到下位机
     * @param {vector<double>} motors_speed 各个电机的目标转速 单位 转/min
     * @param {int*} err err 返回错误码 0表示成功
     * @return {*}
     */
    void sendSpeed(std::vector<double> motors_speed, int* err);

    /**
     * @brief 设置输出io
     * @param {uint16_t} io 长度为7的io数组 1表示开 0表示关
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void setIO(uint16_t io[7], int* err);

    /**
     * @brief 继电器开关
     * @param {bool} state true 闭合、false 断开
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void setRelay(bool state, int* err);

    /**
     * @brief 控制拓展电机
     * @param {int} state 目前仅支持三个状态 0 电机停止 1 电机正传 3 电机反转
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void setExtraMotor(int state, int* err);

    /**
     * @brief 设置蜂鸣器动作
     * @param {int} time 鸣叫的时长 0 停止 取值范围[0, 10]
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void setBuzzer(int time, int* err);

    /**
     * @brief 电机当前转速 必须在readData(后)
     * @param {std::vector<double>*} data_out 输出四个电机转速 单位 转/min
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getMotorSpeed(std::vector<double>* data_out, int* err);

    /**
     * @brief 获取碰撞传感器数据 当前仅bobac2有意义
     * @param {std::vector<int>*} data_out 输出各个碰撞传感器状态
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getBumper(std::vector<int>* data_out, int* err);

    /**
     * @brief 获取防跌落传感器数据 当前仅bobac2有意义
     * @param {std::vector<int>*} data_out 输出各个防跌落传感器状态
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getCliff(std::vector<int>* data_out, int* err);

    /**
     * @brief 获取超声波数据 当前仅bobac系列有意义
     * @param {std::vector<float>*} data_out 输出各个超声波测量距离 单位 毫米
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getUltraSound(std::vector<float>* data_out, int* err);

    /**
     * @brief 获取相对湿度 当前仅bobac系列有意义
     * @param {float*} data_out 输出相对湿度值 单位 rh
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getRelativeHumidity(float* data_out, int* err);

    /**
     * @brief 获取当前环境温度 当前仅bobac系列有意义
     * @param {float*} data_out 输出当前环境温度 单位 摄氏度
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getTemperature(float* data_out, int* err);

    /**
     * @brief 获取当前烟雾传感器值 当前仅bobac系列有意义 仅可检查可燃气体
     * @param {int*} data_out 输出是否检测到可燃气体 1 检测到气体 0 未检测到气体
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getSmoke(int* data_out, int* err);

    /**
     * @brief 获取输入io值 当前bobac2机器人无意义
     * @param {std::vector<int>*} data_out 6个输入io的值
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getInputIO(std::vector<int>* data_out, int* err);

    /**
     * @brief 获取当前电池电压值
     * @param {float*} data_out 当前电池电压值 单位 v
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getPower(float* data_out, int* err);
    
    /**
     * @brief 获取当前是否充电 目前仅可检测充电桩充电 bobac2机器人无意义 
     * @param {int*} data_out 充电状态 0 未充电 1 正在充电
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getCharge(int* data_out, int* err);

    /**
     * @brief 获取当前输出io值
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getOutputIO(std::vector<int>* data_out, int* err);

    /**
     * @brief 获取当前继电器状态
     * @param {bool*} data_out true 闭合 false 断开
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getRelay(bool* data_out, int* err);


    /**
     * @brief 获取当前拓展电机状态
     * @param {int*} state 输出当前拓展电机状态 0 电机停止 1 电机正传 3 电机反转
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void getExtraMotor(int* state, int*err);

    /**
     * @brief 关闭连接
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void close(int* err);

    /**
     * @brief 修改从机地址，函数内会自动连接，无需再调用connect()
     * @param {int} slave 从机地址
     * @param {int*} err 返回错误码 0表示成功
     * @return {*}
     */
    void changeSlave(int slave, int* err);

    static RobotBaseCom& getInstance(){
        static RobotBaseCom instance;
        return instance;
    }
    
    int err_time;//通信出错次数

    BaseComParams communicate_params_;//通信参数

    bool is_connected_;//当前与下位机连接状态

private:
    RobotBaseCom();
    ~RobotBaseCom();
    RobotBaseCom(const RobotBaseCom& other) = delete;
    RobotBaseCom& operator=(const RobotBaseCom& other) = delete;

private:

    union {
        float fd;
        uint16_t id[2];
    } f_to_uint_;//用于float型与uint16_t型互转

    union {
        double dd;
        uint16_t id[4];
    } d_to_uint_;//用于double型与uint16_t型互转

    uint16_t buff_[35];//用于存储读取的一帧下位机数据
    uint16_t registerbuff_[3];//用于存储保持寄存器数据
    modbus_t* mb_=NULL; //modbus句柄
};
}
#endif