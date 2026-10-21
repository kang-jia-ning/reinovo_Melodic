#include <rei_robot_base/rei_robot_base.h>
#include <rei_robot_base/rei_print.h>
#include <string.h>
namespace reinovo_base
{
RobotBaseCom::RobotBaseCom():is_connected_(false), err_time(0){
    memset(buff_, 0, 35);
}
RobotBaseCom::~RobotBaseCom(){
    if(is_connected_){
        modbus_close(mb_);
    }
    modbus_free(mb_);
}

void RobotBaseCom::connect(int* err){
    if(!is_connected_){
        mb_ = modbus_new_rtu(communicate_params_.port.c_str(),
            communicate_params_.baud,communicate_params_.parity,communicate_params_.data_bit,
            communicate_params_.stop_bit);
        if(mb_!=NULL){
            if(modbus_set_slave(mb_, communicate_params_.slave)==0){
                if(modbus_connect(mb_)==0){
                    is_connected_ = true;
                    #if MODBUS_VERION_COMPARE(>=, 3, 1, 2)
                        modbus_set_response_timeout(mb_,0, 200000);
                    #else
                        struct timeval timeout;
                        timeout.tv_sec = 0;
                        timeout.tv_usec = 200000;
                        modbus_set_response_timeout(mb_, &timeout);
                    #endif
                    
                    if(modbus_read_registers(mb_, 16, 3, registerbuff_) == 3){
                        is_connected_ = true;
                        err = 0;
                        return;
                    }
                    is_connected_ = false;
                    modbus_close(mb_);
                }
            }
            modbus_free(mb_);
        }
        *err = -1;
    }else *err = -2;
}

void RobotBaseCom::close(int* err){
    if(is_connected_){
        modbus_close(mb_);
        modbus_free(mb_);
        is_connected_ = false;
        *err=0;
    }*err = -2;
}

void RobotBaseCom::readData(int* err){
    if(is_connected_){
        int size = modbus_read_input_registers(mb_,0,27,buff_);
        
        if(size == 27){
            err_time = 0;
            *err = 0;

        }else{
            *err = -1;
            err_time++;
            if(err_time>3) is_connected_ = false;
            PCOUT_DEBUG<<err_time<<std::endl;
        }
    }else{
        *err = -1;
    }
    
}
void RobotBaseCom::getMotorSpeed(std::vector<double>* data_out, int* err){
    if(is_connected_){
        data_out->clear();
        for (size_t i = DataAdress::MotorSpeed; i < (DataAdress::MotorSpeed+16); i=i+4)
        {
            d_to_uint_.id[3] = buff_[i];
            d_to_uint_.id[2] = buff_[i+1];
            d_to_uint_.id[1] = buff_[i+2];
            d_to_uint_.id[0] = buff_[i+3];
            data_out->push_back(d_to_uint_.dd);
        }
        *err = 0;
    }else *err = -1;
}
void RobotBaseCom::getUltraSound(std::vector<float>* data_out, int* err){
  if(is_connected_){
    data_out->clear();
    int dist = buff_[DataAdress::UltruaSound];
    if(dist==65535) dist = -99900.0f;
    data_out->push_back((float)dist);

    dist = buff_[DataAdress::UltruaSound+1];
    if(dist==65535) dist = -99900.0f;
    data_out->push_back((float)dist);

    dist = buff_[DataAdress::UltruaSound+2];
    if(dist==65535) dist = -99900.0f;
    data_out->push_back((float)dist);
    *err = 0;
  }else *err = -1;
}
void RobotBaseCom::getPower(float* data_out, int* err){
  if(is_connected_){
    int power = buff_[DataAdress::Power];
    *data_out = (float)power/1000.0;
    *err = 0;
  }else *err = -1;
}
void RobotBaseCom::getCharge(int* data_out, int* err){
  if(is_connected_){
    *data_out = 1 >> buff_[DataAdress::Charge];//0则取1,1则取0
    *err = 0;
  }else *err = -1;
}
void RobotBaseCom::getInputIO(std::vector<int>* data_out, int* err){
    if(is_connected_){
        data_out->clear();
        uint16_t check_bit = 1;
        uint16_t io_data = buff_[DataAdress::InputIO];
        //std::cout<<io_data<<std::endl;
        for (size_t i = 0; i <4; i++)
        {
            data_out->push_back((io_data>>i)&check_bit);
        }
        *err = 0;
    }
    else *err = -1;
}
void RobotBaseCom::getSmoke(int* data_out, int* err){
    if(is_connected_){
        *data_out = 1 >> buff_[DataAdress::Smoke];//0则取1,1则取0
        *err = 0;
    }else *err = -1;
}
void RobotBaseCom::getTemperature(float* data_out, int* err){
    if(is_connected_){
        f_to_uint_.id[1] = buff_[DataAdress::Temperature];
        f_to_uint_.id[0] = buff_[DataAdress::Temperature+1];
        *data_out = f_to_uint_.fd;
        *err = 0;
    }else *err = -1;
}
void RobotBaseCom::getRelativeHumidity(float* data_out, int* err){
    if(is_connected_){
        f_to_uint_.id[1] = buff_[DataAdress::RelativeHumidity];
        f_to_uint_.id[0] = buff_[DataAdress::RelativeHumidity+1];
        *data_out = f_to_uint_.fd;
        *err = 0;
    }else *err = -1;
}
void RobotBaseCom::getBumper(std::vector<int>* data_out, int* err){
    
    if(is_connected_){
        std::vector<int> temp;
        getInputIO(&temp, err);
        if(err==0){
            data_out->clear();
            data_out->push_back(temp[5]);
            data_out->push_back(temp[1]);
            data_out->push_back(temp[3]);
            *err = 0;
        }else *err = -1;
    }else *err = -1;
}
void RobotBaseCom::getCliff(std::vector<int>* data_out, int* err){
    if(is_connected_){
        std::vector<int> temp;
        getInputIO(&temp, err);
        if(err==0){
            data_out->clear();
            data_out->push_back(temp[5]);
            data_out->push_back(temp[1]);
            data_out->push_back(temp[3]);
            *err = 0;
        }else *err = -1;
    }else *err = -1;
}

void RobotBaseCom::changeSlave(int slave, int* err){
    if((mb_!=NULL)&&is_connected_){
        modbus_close(mb_);
        if(modbus_set_slave(mb_, slave)==0){
            if(modbus_connect(mb_)==0){
                is_connected_ = true;
                #if MODBUS_VERION_COMPARE(>=, 3, 1, 2)
                    modbus_set_response_timeout(mb_,0, 200000);
                #else
                    struct timeval timeout;
                    timeout.tv_sec = 0;
                    timeout.tv_usec = 200000;
                    modbus_set_response_timeout(mb_, &timeout);
                #endif
                int err;
                readData(&err);
                if(err == 0){
                    is_connected_ = true;
                    err = 0;
                    return;
                }
                is_connected_ = false;
                modbus_close(mb_);
            }
        }
    }else *err = -1;
}

void RobotBaseCom::sendSpeed(std::vector<double> motors_speed, int* err){
    if(is_connected_){
        uint16_t send_data[16];
        uint16_t *p=send_data;
        memset(send_data, 0, 16);
        for (size_t i = 0; i < motors_speed.size(); i++)
        {
            d_to_uint_.dd = motors_speed[i];
            *p++=d_to_uint_.id[3];
            *p++=d_to_uint_.id[2];
            *p++=d_to_uint_.id[1];
            *p++=f_to_uint_.id[0];
        }
        if(modbus_write_registers(mb_,0,16,send_data) == 16)
            *err = 0;
        else *err = -1;
    }else *err = -1;

}

void RobotBaseCom::setRelay(bool state, int* err){
    if(is_connected_){
        uint16_t data;
        if(state) data = 1;
        else data = 0;
        if(modbus_write_registers(mb_, DataAdress::Relay, 1, &data)==1){
            registerbuff_[1] = data;
            *err = 0;
        }else *err = -1;
    }else *err = -1;
}   

void RobotBaseCom::setIO(uint16_t io[7], int* err){
    uint16_t data = 0;
    for (size_t i = 0; i < 7; i++)
    {
        data = data+(io[i]<<i);
    }
    uint16_t tmp;
    if(modbus_write_registers(mb_, DataAdress::OutputIO, 1, &data)==1){
        registerbuff_[2] = data;
        *err = 0;
    }
    else *err = -1;
}

void RobotBaseCom::setExtraMotor(int state, int* err){
    if(is_connected_){
        uint16_t command;
        if(state<0) command = 1; //反转
        else if(state>0) command = 3; //正转
        else command = 0; //停止
    }else *err = -1;
}

void RobotBaseCom::setBuzzer(int time, int* err){
    if(is_connected_){
        if((time>10)||(time<0)) *err = -2;
        else{
            uint16_t data;
            if(modbus_write_registers(mb_, DataAdress::Buzzer, 1, &data)==1){
                registerbuff_[0] = (bool)time;
            }else *err = -1;
        }
    }else *err = -1;
}

void RobotBaseCom::getRelay(bool* data_out, int* err){
    if(is_connected_){
        if(registerbuff_[1]==1) *data_out = true;
        else *data_out = false;
        *err = 0;
    }else *err = -1;
}

void RobotBaseCom::getOutputIO(std::vector<int>* data_out, int* err){
    if(is_connected_){
        data_out->clear();
        for(size_t i=0; i<7; i++){
            if((registerbuff_[2]>>i)&1) data_out->push_back(1);
            else data_out->push_back(0);
        }
        *err = 0;
    }else *err = -1;
}
} // namespace reinovo_base
