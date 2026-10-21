#include <rei_robot_base/kinematics.h>
template<typename T>
std::ostream& operator<< (std::ostream& os, std::vector<T> vec)
{
    os << '[';
    for(int i=0; i<vec.size(); i++) {
        os << vec[i] <<" ";
    }
    return os << ']';
}
namespace reinovo_base
{
BaseKinematics::BaseKinematics():init_ok(false){}

bool BaseKinematics::init(){
    switch(car_data_.car_mode){
        case MotionModel::kDiff:
            if(car_data_.wheel_radius*car_data_.wheel_separation_y==0){
                init_ok = false;
                return false;
            }
            else{
                init_ok = true;
            }
            break;
        case MotionModel::kThreeWheeledOmni:
            if(car_data_.wheel_radius*car_data_.wheel_separation_x==0){
                init_ok = false;
                return false;
            }
            else{
                init_ok = true;
            }
            break;
        case MotionModel::kFourMecanumOmni:
            if(car_data_.wheel_radius*car_data_.wheel_separation_x*car_data_.wheel_separation_y==0){
                init_ok = false;
                return false;
            }
            else{
                init_ok = true;
            }
            break;
        default:
            std::cerr<<"This motion model is unrecognized or unimplemented, motion model: "<< car_data_.car_mode <<std::endl;
            init_ok = false;
            return false;
            break;
    }
    return true;
}

std::vector<double> BaseKinematics::ForwardKinematics(std::vector<double> motor_speed){
    std::vector<double> velocity(3, 0);	// vx, vy, vth
    if(init_ok){
        switch(car_data_.car_mode) {
        case (MotionModel::kDiff):
            //两轮差速
            /******************
                2--↑--1
            ******************/
        
            velocity[0] = -(motor_speed[0] - motor_speed[1])*PI*car_data_.wheel_radius/60.0; //m/s
            velocity[1] = 0;
            velocity[2] = (motor_speed[0] + motor_speed[1])*2*PI*car_data_.wheel_radius/car_data_.wheel_separation_y/60.0;
            break;
        case (MotionModel::kThreeWheeledOmni):
            //三轮全向
            /******************
                2-----1
                \ ↑ /
                 \↑/
                  3
            ******************/

            velocity[0] = (motor_speed[0] - motor_speed[1])*PI/30.f*car_data_.wheel_radius/sqrt(3);
            velocity[1] = (motor_speed[0] + motor_speed[1] - 2*motor_speed[2])*PI*car_data_.wheel_radius/30.f/3.f;
            velocity[2] = -(motor_speed[0] + motor_speed[1] + motor_speed[2])*PI*car_data_.wheel_radius/30.f/3.f/car_data_.wheel_separation_x;
            break;
        case (MotionModel::kFourMecanumOmni):
            //麦克纳姆轮(辊子接触地面为方形)
            /******************
                2-----1
                |  ↑  |
                |  ↑  |
                3-----4
            ******************/
            velocity[0] = (motor_speed[0] - motor_speed[1])*car_data_.wheel_radius/2*PI/30.f;
            velocity[1] = (motor_speed[0] - motor_speed[3])*car_data_.wheel_radius/2*PI/30.f;
            velocity[2] = (motor_speed[1] + motor_speed[3])*car_data_.wheel_radius/(car_data_.wheel_separation_x+car_data_.wheel_separation_y)*PI/30.f;
            break;
        default:
            std::cerr<<"This motion model is unrecognized or unimplemented, motion model: "<< car_data_.car_mode <<std::endl;
        }
        #if 1//KINEMATICS_DEBUG
            //std::cout << "kinematics: "<< velocity << std::endl;
        #endif
    }
    return  velocity;
}
std::vector<double> BaseKinematics::InverseKinematics(double vx, double vy, double vth)
{
    std::vector<double> motor_speed(4, 0);
    if(init_ok){
        switch(car_data_.car_mode) {
        case (MotionModel::kDiff):
            //两轮差速
            /******************
                2--↑--1
            ******************/
            motor_speed[0] = -(vx - vth*car_data_.wheel_separation_y/2)/car_data_.wheel_radius*30/PI;
            motor_speed[1] = (vx + vth*car_data_.wheel_separation_y/2)/car_data_.wheel_radius*30/PI;
            break;
        case (MotionModel::kThreeWheeledOmni):
            //三轮全向
            /******************
                2-----1
                \ ↑ /
                \↑/
                3
            ******************/
            //ROS_INFO("do nothing on kinematics_mode=3, waiting ro update");

            motor_speed[0] = (vy*0.5 + vx*sqrt(3.0)/2.0 - car_data_.wheel_separation_x*vth)/car_data_.wheel_radius*30/PI;
            motor_speed[1] = (vy*0.5 - vx*sqrt(3.0)/2.0  - car_data_.wheel_separation_x*vth)/car_data_.wheel_radius*30/PI;
            motor_speed[2] = (-vy - car_data_.wheel_separation_x*vth)/car_data_.wheel_radius*30/PI;
            break;
        case (MotionModel::kFourMecanumOmni):
            //麦克纳姆轮(辊子接触地面为方形)
            /******************
                2-----1
                |  ↑  |
                |  ↑  |
                3-----4
            ******************/
            motor_speed[0] = (vx + vy + 0.5*vth*(car_data_.wheel_separation_y + car_data_.wheel_separation_x))/car_data_.wheel_radius*30/PI;
            motor_speed[1] = -(vx - vy - 0.5*vth*(car_data_.wheel_separation_y + car_data_.wheel_separation_x))/car_data_.wheel_radius*30/PI;
            motor_speed[2] = -(vx + vy - 0.5*vth*(car_data_.wheel_separation_y + car_data_.wheel_separation_x))/car_data_.wheel_radius*30/PI;
            motor_speed[3] = (vx - vy + 0.5*vth*(car_data_.wheel_separation_y + car_data_.wheel_separation_x))/car_data_.wheel_radius*30/PI;
            break;
        default:
            std::cerr<<"This motion model is unrecognized or unimplemented, motion model: "<< car_data_.car_mode <<std::endl;
        }
        #if 1//INVERS_KINEMATICS_DEBUG
            //std::cout << "kinematics motor speed: " << motor_speed << std::endl;
        #endif
    }

    return motor_speed;
}
} // namespace reinovo_base

