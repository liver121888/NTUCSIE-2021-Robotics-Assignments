#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

int main(int argc, char **argv){
    ros::init(argc,argv,"Node_name");
    ros::NodeHandle nh;

    ros::Publisher topic_pub = nh.advertise<std_msgs::String>("name_of_topic",1000);
    //(1000 is buffer, if larger than 1000 msg throw)
		ros::Rate loop_rate(1);//Hz

    while(ros::ok()){
        std_msgs::String msg;

        topic_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}