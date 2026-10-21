#include"ros/ros.h"//包含ROS头文件  
#include <move_base_msgs/MoveBaseAction.h>//包含move_base action头文件  
#include <actionlib/client/simple_action_client.h>//包含simple_aciton客户端头文件  
#include <geometry_msgs/Twist.h>
#include <robot_audio/robot_iat.h>
#include <robot_audio/Collect.h>
#include <robot_audio/robot_tts.h>

#include <iostream>
#include <string>


using namespace std;

# define goal1 beijing
# define goal2 shanghai 
# define goal3 guangzhou
# define goal4 shengzheng
# define goal5 jilin

# define goal1_pre beijing_pre
# define goal2_pre shanghai_pre
# define goal3_pre guangzhou_pre
# define goal4_pre shengzheng_pre
# define goal5_pre jilin_pre

string voice[10];

double beijing_x, shanghai_x, guangzhou_x, shenzheng_x, jilin_x, charge_x, goal6_x, goal7_x;
double beijing_y, shanghai_y, guangzhou_y, shenzheng_y, jilin_y, charge_y, goal6_y, goal7_y;
double beijing_z, shanghai_z, guangzhou_z, shenzheng_z, jilin_z, charge_z, goal6_z, goal7_z; 
double beijing_w, shanghai_w, guangzhou_w, shenzheng_w, jilin_w, charge_w, goal6_w, goal7_w;

double beijing_pre_x, shanghai_pre_x, guangzhou_pre_x, shenzheng_pre_x, jilin_pre_x;
double beijing_pre_y, shanghai_pre_y, guangzhou_pre_y, shenzheng_pre_y, jilin_pre_y;
double beijing_pre_z, shanghai_pre_z, guangzhou_pre_z, shenzheng_pre_z, jilin_pre_z;
double beijing_pre_w, shanghai_pre_w, guangzhou_pre_w, shenzheng_pre_w, jilin_pre_w;

void load_param();

void voice_fun() {
voice[1]="/home/bobac3/bobac3_ws/src/nav_goal/voices/北京是中国的首都政治中心.wav";
voice[2]="/home/bobac3/bobac3_ws/src/nav_goal/voices/上海是中国的经济中心.wav";
voice[3]="/home/bobac3/bobac3_ws/src/nav_goal/voices/广州自古都是我国的商业之都.wav";
voice[4]="/home/bobac3/bobac3_ws/src/nav_goal/voices/深圳是中国的科创中心.wav";
voice[5]="/home/bobac3/bobac3_ws/src/nav_goal/voices/吉林位于我国东北是人参之都.wav";
}

class nav_goal{
    public:
        nav_goal();
        string voice_collect(); 
        string voice_dictation(const char* filename); 
        string voice_tts(const std::string& text);
        void goto_nav(struct Point* point); 

    private:
        ros::NodeHandle n;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac; 
        ros::ServiceClient collect_client,dictation_client,tts_client; 
};
nav_goal::nav_goal(){
    collect_client = n.serviceClient<robot_audio::Collect>("voice_collect"); 
    dictation_client = n.serviceClient<robot_audio::robot_iat>("voice_iat"); 
    tts_client = n.serviceClient<robot_audio::robot_tts>("voice_tts"); 
}
string nav_goal::voice_collect(){
    ros::service::waitForService("voice_collect");
    robot_audio::Collect srv;
    srv.request.collect_flag = 1;
    collect_client.call(srv);
    return srv.response.voice_filename;
}
string nav_goal::voice_dictation(const char* filename){
    ros::service::waitForService("voice_iat");
    robot_audio::robot_iat srv;
    srv.request.audiopath = filename;
    dictation_client.call(srv);
    return srv.response.text;
}
string nav_goal::voice_tts(const std::string& text){
    ros::service::waitForService("voice_tts");
    robot_audio::robot_tts srv;
    srv.request.text = text;
    tts_client.call(srv);
    string cmd= "play "+srv.response.audiopath;
    system(cmd.c_str());
    sleep(1);
    return srv.response.audiopath;
}

move_base_msgs::MoveBaseGoal beijing;
move_base_msgs::MoveBaseGoal guangzhou;
move_base_msgs::MoveBaseGoal jilin;
move_base_msgs::MoveBaseGoal shengzheng;
move_base_msgs::MoveBaseGoal shanghai;
move_base_msgs::MoveBaseGoal goal6;
move_base_msgs::MoveBaseGoal charge;

move_base_msgs::MoveBaseGoal beijing_pre;
move_base_msgs::MoveBaseGoal guangzhou_pre;
move_base_msgs::MoveBaseGoal jilin_pre;
move_base_msgs::MoveBaseGoal shengzheng_pre;
move_base_msgs::MoveBaseGoal shanghai_pre;
move_base_msgs::MoveBaseGoal beitai;
move_base_msgs::MoveBaseGoal goal7;

void coordinate()
{
  beijing.target_pose.header.frame_id = "map";  
  beijing.target_pose.header.stamp = ros::Time::now();  
  beijing.target_pose.pose.position.x = beijing_x;
  beijing.target_pose.pose.position.y = beijing_y;
  beijing.target_pose.pose.orientation.z = beijing_z; 
  beijing.target_pose.pose.orientation.w = beijing_w;

  beijing_pre.target_pose.header.frame_id = "map";  
  beijing_pre.target_pose.header.stamp = ros::Time::now();  
  beijing_pre.target_pose.pose.position.x = beijing_pre_x;
  beijing_pre.target_pose.pose.position.y = beijing_pre_y;
  beijing_pre.target_pose.pose.orientation.z = beijing_pre_z;
  beijing_pre.target_pose.pose.orientation.w = beijing_pre_w;

  guangzhou.target_pose.header.frame_id = "map";  
  guangzhou.target_pose.header.stamp = ros::Time::now();  
  guangzhou.target_pose.pose.position.x = guangzhou_x;
  guangzhou.target_pose.pose.position.y = guangzhou_y; 
  guangzhou.target_pose.pose.orientation.z = guangzhou_z;  
  guangzhou.target_pose.pose.orientation.w = guangzhou_w;

  guangzhou_pre.target_pose.header.frame_id = "map";  
  guangzhou_pre.target_pose.header.stamp = ros::Time::now();  
  guangzhou_pre.target_pose.pose.position.x = guangzhou_pre_x;
  guangzhou_pre.target_pose.pose.position.y = guangzhou_pre_y; 
  guangzhou_pre.target_pose.pose.orientation.z = guangzhou_pre_z;  
  guangzhou_pre.target_pose.pose.orientation.w = guangzhou_pre_w;

  jilin.target_pose.header.frame_id = "map";  
  jilin.target_pose.header.stamp = ros::Time::now();  
  jilin.target_pose.pose.position.x = jilin_x;
  jilin.target_pose.pose.position.y = jilin_y;   
  jilin.target_pose.pose.orientation.z = jilin_z;  
  jilin.target_pose.pose.orientation.w = jilin_w;

  jilin_pre.target_pose.header.frame_id = "map";  
  jilin_pre.target_pose.header.stamp = ros::Time::now();  
  jilin_pre.target_pose.pose.position.x = jilin_pre_x;
  jilin_pre.target_pose.pose.position.y = jilin_pre_y;   
  jilin_pre.target_pose.pose.orientation.z = jilin_pre_z;  
  jilin_pre.target_pose.pose.orientation.w = jilin_pre_w;

  shengzheng.target_pose.header.frame_id = "map";  
  shengzheng.target_pose.header.stamp = ros::Time::now();  
  shengzheng.target_pose.pose.position.x =shenzheng_x;  
  shengzheng.target_pose.pose.position.y = shenzheng_y;     
  shengzheng.target_pose.pose.orientation.z = shenzheng_z;  
  shengzheng.target_pose.pose.orientation.w = shenzheng_w;

  shengzheng_pre.target_pose.header.frame_id = "map";  
  shengzheng_pre.target_pose.header.stamp = ros::Time::now();  
  shengzheng_pre.target_pose.pose.position.x =shenzheng_pre_x;  
  shengzheng_pre.target_pose.pose.position.y = shenzheng_pre_y;   
  shengzheng_pre.target_pose.pose.orientation.z = shenzheng_pre_z;  
  shengzheng_pre.target_pose.pose.orientation.w = shenzheng_pre_w;

  shanghai.target_pose.header.frame_id = "map";  
  shanghai.target_pose.header.stamp = ros::Time::now();  
  shanghai.target_pose.pose.position.x =shanghai_x;
  shanghai.target_pose.pose.position.y = shanghai_y;   
  shanghai.target_pose.pose.orientation.z =shanghai_z;  
  shanghai.target_pose.pose.orientation.w = shanghai_w;

  shanghai_pre.target_pose.header.frame_id = "map";  
  shanghai_pre.target_pose.header.stamp = ros::Time::now();  
  shanghai_pre.target_pose.pose.position.x =shanghai_pre_x;
  shanghai_pre.target_pose.pose.position.y = shanghai_pre_y;   
  shanghai_pre.target_pose.pose.orientation.z =shanghai_pre_z;  
  shanghai_pre.target_pose.pose.orientation.w = shanghai_pre_w;

  goal6.target_pose.header.frame_id = "map";  
  goal6.target_pose.header.stamp = ros::Time::now();  
  goal6.target_pose.pose.position.x =goal6_x;
  goal6.target_pose.pose.position.y = goal6_y;    
  goal6.target_pose.pose.orientation.z =goal6_z;  
  goal6.target_pose.pose.orientation.w = goal6_w;

  goal7.target_pose.header.frame_id = "map";  
  goal7.target_pose.header.stamp = ros::Time::now();  
  goal7.target_pose.pose.position.x =goal7_x;
  goal7.target_pose.pose.position.y = goal7_y;    
  goal7.target_pose.pose.orientation.z =goal7_z;  
  goal7.target_pose.pose.orientation.w = goal7_w;

  charge.target_pose.header.frame_id = "map";  
  charge.target_pose.header.stamp = ros::Time::now();  
  charge.target_pose.pose.position.x =charge_x;//
  charge.target_pose.pose.position.y = charge_y;    //
  charge.target_pose.pose.orientation.z =charge_z;  
  charge.target_pose.pose.orientation.w = charge_w;//
}

int main(int argc,char** argv)  
{   
  ros::init(argc,argv,"nav_goal");  

   nav_goal audio; 
   string dir,text,path;

  load_param();
  
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
  ac.waitForServer();
  coordinate();
  voice_fun();
  bool task1=true;
  bool task2=false;

  while(task1)
  {
	   dir = audio.voice_collect(); 
	   text = audio.voice_dictation(dir.c_str()).c_str(); 
           if(text.find("元宝") != string::npos) {
	       char command0[256];
               snprintf(command0, sizeof(command0), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/什么事呀.wav");
	       system(command0);
          }
	   if(text.find("一圈") != string::npos)
	  {
	       char command[256];
               snprintf(command, sizeof(command), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/这就带你参观.wav");
	       system(command);
	       usleep(1000*6);

		  ac.sendGoal(goal1_pre);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
		  ac.sendGoal(goal1);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
	       char command1[256];
               snprintf(command1, sizeof(command1), "play %s", voice[1].c_str());
	       system(command1);
	       usleep(1000*6);
		    
		//   ac.sendGoal(goal1_pre);  
		//   while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		//        usleep(1000*20);  
		//     }  

		   ac.sendGoal(goal2_pre);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }
		   ac.sendGoal(goal2);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
	       char command2[256];
               snprintf(command2, sizeof(command2), "play %s", voice[2].c_str());
	       system(command2);
	       usleep(1000*6);
			
		   ac.sendGoal(goal3_pre);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
		  ac.sendGoal(goal3);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
	       char command3[256];
               snprintf(command3, sizeof(command3), "play %s", voice[3].c_str());
	       system(command3);
	       usleep(1000*6);

		   ac.sendGoal(goal4_pre);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
		 ac.sendGoal(goal4);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
	       char command4[256];
               snprintf(command4, sizeof(command4), "play %s", voice[4].c_str());
	       system(command4);
	       usleep(1000*6);

		//    ac.sendGoal(goal4_pre);  
		//   while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		//     {  
		//        usleep(1000*20);  
		//     }  

		   ac.sendGoal(goal5_pre);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
		 ac.sendGoal(goal5);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
	       char command5[256];
               snprintf(command5, sizeof(command5), "play %s", voice[5].c_str());
	       system(command5);
	       usleep(1000*6);

		// ac.sendGoal(goal5_pre);  
		//   while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		//     {  
		//        usleep(1000*20);  
		//     }  

		 ac.sendGoal(goal6);  
		  while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
		    {  
		       usleep(1000*20);  
		    }  
		 task1=false; 
	   }
	task2=true;
   }
  while(task2)
//  while(1)
  {
	   dir = audio.voice_collect(); //采集语音
	   text = audio.voice_dictation(dir.c_str()).c_str(); //语音听写
           if(text.find("元宝") != string::npos) {
	       char command0[256];
               snprintf(command0, sizeof(command0), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/什么事呀.wav");
	       system(command0);
           }
	   if(text.find("北京") != string::npos)
	   {
	     char command1[256];
             snprintf(command1, sizeof(command1), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/去北京馆.wav");
	     system(command1);
    	     usleep(1000 * 3);

	      ac.sendGoal(beijing_pre);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {
		 usleep(1000*10); 
	      } 
	      ac.sendGoal(beijing);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*10);
	      }

             char command2[256];
             snprintf(command2, sizeof(command2), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/北京是中国的首都政治中心.wav");
	     system(command2);
    	     usleep(1000 * 3);
 
	       ac.sendGoal(charge);
		   while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  //必须有，未到达前一直在循环里
	       {  
		  usleep(1000*10);
	       }
		  char command[256];
		   snprintf(command, sizeof(command), "roslaunch %s %s", "secondary_localization", "track.launch &");
		   system(command);//进程阻塞。有&不会

    	   usleep(1000 * 12000);//2s1000*2000？

		   system(" rosnode kill /relative_move");
		 //   system(" rosnode kill /base_pose_adjust");
		 //   system(" rosnode kill /bottom_camera/ar_track_alvar");
	      
	       char command3[256];
               snprintf(command3, sizeof(command3), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/充电成功.wav");
	       system(command3);
	       usleep(1000*6);

	      ac.sendGoal(goal7);
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*20);  
	      }  
	   }
	   else if(text.find("广州") != string::npos)
	   {
	      //a=0;
	     char command1[256];
             snprintf(command1, sizeof(command1), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/去广州馆.wav");
	     system(command1);
    	     usleep(1000 * 3);

	      ac.sendGoal(guangzhou_pre);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {
		 usleep(1000*10);  //每次检查 ac.getState() 之后暂停执行 10 毫秒,避免程序过度占用 CPU 资源
	      } 
	      ac.sendGoal(guangzhou);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*10);  
	      }  

	     char command2[256];
             snprintf(command2, sizeof(command2), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/广州自古都是我国的商业之都.wav");
	     system(command2);
    	     usleep(1000 * 3);

		   	      ac.sendGoal(charge);
		   while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	       {  
		  usleep(1000*10);
	       }
		  char command[256];
		   snprintf(command, sizeof(command), "roslaunch %s %s", "secondary_localization", "track.launch &");
		   system(command);//进程阻塞。有&不会

    	   usleep(1000 * 12000);//2s1000*2000？

		   system(" rosnode kill /relative_move");
		 //   system(" rosnode kill /base_pose_adjust");
		 //   system(" rosnode kill /bottom_camera/ar_track_alvar");

	       char command3[256];
               snprintf(command3, sizeof(command3), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/oices/充电成功.wav");
	       system(command3);
	       usleep(1000*6);

	      ac.sendGoal(goal7);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*20);  
	      }  
	   }
	   else if(text.find("吉林") != string::npos)
	   {
	      //a=0;
	     char command1[256];
             snprintf(command1, sizeof(command1), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/去吉林馆.wav");
	     system(command1);
    	     usleep(1000 * 3);

	      ac.sendGoal(jilin_pre);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {
		 usleep(1000*10);  //每次检查 ac.getState() 之后暂停执行 10 毫秒,避免程序过度占用 CPU 资源
	      } 

	      ac.sendGoal(jilin);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*10);  
	      }  
	     char command2[256];
             snprintf(command2, sizeof(command2), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/吉林位于我国东北是人参之都.wav");
	     system(command2);
    	     usleep(1000 * 3);

	     ac.sendGoal(charge);
		   while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	       {  
		  usleep(1000*10);
	       }
		  char command[256];
		   snprintf(command, sizeof(command), "roslaunch %s %s", "secondary_localization", "track.launch &");
		   system(command);//进程阻塞。有&不会

    	   usleep(1000 * 12000);//2s1000*2000？

		   system(" rosnode kill /relative_move");
		 //   system(" rosnode kill /base_pose_adjust");
		 //   system(" rosnode kill /bottom_camera/ar_track_alvar");

	       char command3[256];
               snprintf(command3, sizeof(command3), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/充电成功.wav");
	       system(command3);
	       usleep(1000*6);

	      ac.sendGoal(goal7);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*20);  
	      }  
	   }
	      else if(text.find("深圳") != string::npos)
	   {
	     // a=0;

	     char command1[256];
             snprintf(command1, sizeof(command1), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/去深圳馆.wav");
	     system(command1);
    	     usleep(1000 * 3);

	      ac.sendGoal(shengzheng_pre);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {
		 usleep(1000*10);  //每次检查 ac.getState() 之后暂停执行 10 毫秒,避免程序过度占用 CPU 资源
	      } 

	      ac.sendGoal(shengzheng);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*10);  
	      }  
	     char command2[256];
             snprintf(command2, sizeof(command2), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voice/深圳是中国的科创中心.wav");
	     system(command2);
    	     usleep(1000 * 3);
		   	      ac.sendGoal(charge);
		   while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	       {  
		  usleep(1000*10);
	       }
		   		 char command[256];
		   snprintf(command, sizeof(command), "roslaunch %s %s", "secondary_localization", "track.launch &");
		   system(command);//进程阻塞。有&不会

    	   usleep(1000 * 12000);//2s1000*2000？

		   system(" rosnode kill /relative_move");
		 //   system(" rosnode kill /base_pose_adjust");
		 //   system(" rosnode kill /bottom_camera/ar_track_alvar");

	       char command3[256];
               snprintf(command3, sizeof(command3), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/充电成功.wav");
	       system(command3);
	       usleep(1000*6);

	      ac.sendGoal(goal7);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*20);  
	      }  
	   }
	      else if(text.find("上海") != string::npos)
	   {
	      //a=0;
	     char command1[256];
             snprintf(command1, sizeof(command1), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/去上海馆.wav");
	     system(command1);
    	     usleep(1000 * 3);
	      ac.sendGoal(shanghai_pre);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {
		 usleep(1000*10);  //每次检查 ac.getState() 之后暂停执行 10 毫秒,避免程序过度占用 CPU 资源
	      } 

	      ac.sendGoal(shanghai);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*10);  
	      }  
	     char command2[256];
             snprintf(command2, sizeof(command2), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/上海是中国的经济中心.wav");
	     system(command2);
    	     usleep(1000 * 3);
		   	      ac.sendGoal(charge);
		   while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	       {  
		  usleep(1000*10);
	       }
		   		 char command[256];
		   snprintf(command, sizeof(command), "roslaunch %s %s", "secondary_localization", "track.launch &");
		   system(command);//进程阻塞。有&不会

    	  usleep(1000 * 12000);//2s1000*2000？

		   system(" rosnode kill /relative_move");
		 //   system(" rosnode kill /base_pose_adjust");
		 //   system(" rosnode kill /bottom_camera/ar_track_alvar");

	       char command3[256];
               snprintf(command3, sizeof(command3), "play %s", "/home/bobac3/bobac3_ws/src/nav_goal/voices/充电成功.wav");
	       system(command3);
	       usleep(1000*6);

	      ac.sendGoal(goal7);  
	      while(!(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED))  
	      {  
		 usleep(1000*20);  
	      }  
	   }
  }
  
  return 0;  
}  

void load_param() {
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

    ros::param::get("/goal7_x", goal7_x);
    ros::param::get("/goal7_y", goal7_y);
    ros::param::get("/goal7_z", goal7_z);
    ros::param::get("/goal7_w", goal7_w);

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




