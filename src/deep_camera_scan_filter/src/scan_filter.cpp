#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
using  namespace std;

class scan_filter
{
     private:
    ros::NodeHandle nh;
     ros::Subscriber scan_in;
     ros::Publisher scan_out;
     string  scan_in_name;
     string scan_out_name;
     public:
     scan_filter();
     void scan_filter_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in);
};
 
scan_filter::scan_filter()
{
     nh.getParam("/scan_filter/scan_in_name",scan_in_name);
     nh.getParam("/scan_filter/scan_out_name",scan_out_name);
     scan_in = nh.subscribe(scan_in_name,1,&scan_filter::scan_filter_cb,this);
     scan_out = nh.advertise<sensor_msgs::LaserScan>(scan_out_name,1);
}
void scan_filter::scan_filter_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    //scan_in->ranges.size();
    sensor_msgs::LaserScan m_scan_in;
    m_scan_in = *scan_in;
    for (int i=0;i<m_scan_in.ranges.size();i++)
         {
           if(m_scan_in.ranges[i] == INFINITY)
           {
             m_scan_in.ranges[i] = 0;

           }
         }
     scan_out.publish(m_scan_in);
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"scan_filter");
    scan_filter m_scan_filter;
    ros::spin();
    return 0;
}
