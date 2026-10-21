#include "ros/ros.h"
#include "robot_audio/Collect.h"
#include "robot_audio/robot_iat.h"
#include "robot_audio/robot_tts.h"
#include "face_rec/recognition_results.h"

class Interaction {
private:
    ros::NodeHandle n;
    ros::ServiceClient collect_client;
    ros::ServiceClient dictation_client;
    ros::ServiceClient tts_client;
    ros::ServiceClient face_rec_client;

public:
    Interaction() : collect_client(n.serviceClient<robot_audio::Collect>("voice_collect")),
                    dictation_client(n.serviceClient<robot_audio::robot_iat>("voice_iat")),
                    tts_client(n.serviceClient<robot_audio::robot_tts>("voice_tts")),
                    face_rec_client(n.serviceClient<face_rec::recognition_results>("face_recognition_results")) {}

    std::string voice_collect() {
        ros::service::waitForService("voice_collect");
        robot_audio::Collect srv;
        srv.request.collect_flag = 1;
        if (collect_client.call(srv)) {
            ROS_INFO("Voice collected successfully");
        } else {
            ROS_ERROR("Failed to collect voice");
        }
        return srv.response.voice_filename;
    }

    std::string voice_dictation(const std::string& filename) {
        ros::service::waitForService("voice_iat");
        robot_audio::robot_iat srv;
        srv.request.audiopath = filename;
        dictation_client.call(srv);
        return srv.response.text;
    }

    void voice_tts(const std::string& text) {
        ros::service::waitForService("voice_tts");
        robot_audio::robot_tts srv;
        srv.request.text = text;
        tts_client.call(srv);
        std::string cmd = "play " + srv.response.audiopath;
        system(cmd.c_str());
        sleep(1);
    }

    std::string face_recognition_results() {
        ros::service::waitForService("face_recognition_results");
        face_rec::recognition_results face_rec_data;
        face_rec_data.request.mode = 1;
        face_rec_client.call(face_rec_data);
        std::string rec_name;
        if (face_rec_data.response.success) {
            int face_num = face_rec_data.response.result.num;
            std::cout << "我看到了" << face_num << "个人脸,他们分别是：" << std::endl;
            for (int i = 0; i < face_num; i++) {
                rec_name = face_rec_data.response.result.face_data[i].name;
                std::cout << rec_name << " ";
            }
            std::cout << std::endl;
        }
        return rec_name;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "interaction");
    Interaction audio;

    while (ros::ok()) {
        std::string dir = audio.voice_collect();
        std::string text = audio.voice_dictation(dir);
       

        if (text.find("是谁") != std::string::npos) {
            audio.voice_tts("好的，让我看一看");
            std::string name = audio.face_recognition_results();
            audio.voice_tts("我看到了" + name);
        }
    }

    return 0;
}
