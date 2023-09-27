// Author: Ryan Gupta
//
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

#include "ros/ros.h"

// bool received = false;

class Listener{
  public:
    double x, y, yaw, rx, ry, rz, rw;
    geometry_msgs::Pose pose;

    void global_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose = msg->pose;
        // received =true;


        //geometry_msgs::PoseWithCovarianceStamped pose_msg;
    }
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "hsr_localization_converter");
    ros::NodeHandle n;
    //ros::Rate loop_rate(100);
    ros::Rate loop_rate(10);

    Listener listener;

    ros::Subscriber global_pose_sub = n.subscribe("/localization_ros", 1000, &Listener::global_pose_cb, &listener);
    ros::Publisher localization_ros_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/global_pose", 1000);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;


    ros::Time current_time;
    current_time = ros::Time::now();

    while(ros::ok()){
        ros::spinOnce();

	pose_msg.header.frame_id = "map";
	pose_msg.pose.pose = listener.pose;
    pose_msg.header.stamp = ros::Time::now();
        // if(received)
        // {
    localization_ros_pub.publish(pose_msg);
            // received=false;

        // }
        loop_rate.sleep();
    }


}
