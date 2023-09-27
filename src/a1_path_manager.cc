// Author: Ryan Gupta
// A node to intake /search_plan (global search paths)
//   and publish them as the robot approaches them.
// Additionally, this node intakes /priority_wps (curiosity
//   based waypoints) from the curiosity_waypoint_node for 
//   priority inspection


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include <stdlib.h>

bool path_received = false;
bool curiosity_received = false;
int k = 0;
int p = 0;

class Listener{
  public:
    double x, y, yaw;
    geometry_msgs::PoseStamped waypoints[44440];
    geometry_msgs::Pose pose;

    geometry_msgs::Pose priority_wp, prev_priority;

    void path_cb(const nav_msgs::Path::ConstPtr& msg)
    {
        for(int j=0; j<msg->poses.size(); ++j){
            waypoints[j] = msg->poses[j];
        }
        // std::cout << "Path Received" << std::endl;
        path_received = true;
        ++k;
    }

    void curiosity_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        priority_wp = msg->pose;
        // std::cout << "priority wp received" << std::endl;i
        std::cout << "priority_wp.position.x = " << priority_wp.position.x << std::endl;
        std::cout << "prev_priority.position.x = " << prev_priority.position.x << std::endl;
        if( (abs(prev_priority.position.x - priority_wp.position.x) > 3.0)
                || (abs(prev_priority.position.x - priority_wp.position.y) > 3.0) ){
          curiosity_received = true;
          prev_priority = priority_wp;
        }
                        // std::cout << "priority wp received ++" << std::endl;
    }

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose = msg->pose;
    }
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "a1_path_manager");
    ros::NodeHandle n;
    //ros::Rate loop_rate(100);
    ros::Rate loop_rate(10);

    Listener listener;

    ros::Subscriber search_path_sub = n.subscribe("/search_plan", 1000, &Listener::path_cb, &listener);
    ros::Subscriber curiosity_sub = n.subscribe("/priority_wps", 1000, &Listener::curiosity_cb, &listener);
    ros::Subscriber pose_sub = n.subscribe("/localization_ros", 1000, &Listener::pose_cb, &listener);
    ros::Publisher next_waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

    geometry_msgs::PoseStamped goal_msg;
    geometry_msgs::Pose prev_goal;
    geometry_msgs::Pose pose;

    bool manage = false;
    bool done_with_prev = true;
    bool given_priority_wp = false;
    bool given_global_wp = false;;

    double time_gap = 13.;

    ros::Time current_time, prev_time;
    current_time = ros::Time::now();
    prev_time = ros::Time::now();

    int i = 1;

    while(ros::ok()){
        ros::spinOnce();
        pose = listener.pose;

        // std::cout << "Main loop" << std::endl;

        if(path_received && k<=1)
        {
            // std::cout << "Path received and sending first goal" << std::endl;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = "map";
            goal_msg.pose = listener.waypoints[0].pose;
            prev_goal = goal_msg.pose;

            next_waypoint_pub.publish(goal_msg);
            path_received = false;
            given_global_wp = true;
            // Global Paths received, start managing
            manage = true;
            prev_time = ros::Time::now();
            done_with_prev = true;

        }

        current_time = ros::Time::now();


        if(manage)
        {
          // std::cout << "manage = true" << std::endl;
          // Now we have our global paths.
          ros::Duration diff = current_time - prev_time;
          // Time b/w waypoints at least 5 seconds and done with the previous one
          std::cout << "diff.toSec() = " << diff.toSec() << std::endl;
          std::cout << "done_with_prev = " << done_with_prev << std::endl;
          std::cout << "curiosity_received = " << curiosity_received << std::endl;
          if(p>=1) time_gap = 20.;

          if(diff.toSec() >= time_gap && done_with_prev && curiosity_received){
            std::cout << "ready for new priority wp" << std::endl;
            done_with_prev = false; // reset prev_time after we finish
            goal_msg.pose = listener.priority_wp;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = "map";
            next_waypoint_pub.publish(goal_msg);
            given_priority_wp = true;
            given_global_wp = false;
            curiosity_received = false;
            p++;
          }

          if(given_priority_wp){
            std::cout << "given priority wp" << std::endl;
            double dist, ang;
            dist = sqrt(pow((pose.position.x - goal_msg.pose.position.x),2.0) + pow((pose.position.y - goal_msg.pose.position.y), 2.0));
            // Angular Error from goal point
            tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            tf::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            tf::Quaternion q2(goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, goal_msg.pose.orientation.z, goal_msg.pose.orientation.w);
            tf::Matrix3x3 m2(q2);
            double r2, p2, y2;
            m2.getRPY(r2, p2, y2);
            ang = abs(y2 - y);
            // If we are near point and correct angle, publish next waypoint
            if( (dist <= 1.5) && (ang <= 0.3) )
            {
              goal_msg.pose = listener.waypoints[i-1].pose;
              goal_msg.header.stamp = ros::Time::now();
              goal_msg.header.frame_id = "map";
              next_waypoint_pub.publish(goal_msg);
              given_global_wp = true;
              given_priority_wp = false;
              done_with_prev = true;
              prev_time = ros::Time::now();
            }
          }

          if(given_global_wp){
            // std::cout << "given_global wp" << std::endl;
            double dist, ang;
            // Distance from goal point
            dist = sqrt(pow((pose.position.x - goal_msg.pose.position.x),2.0) + pow((pose.position.y - goal_msg.pose.position.y), 2.0));
            // Angular Error from goal point
            tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            tf::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            tf::Quaternion q2(goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, goal_msg.pose.orientation.z, goal_msg.pose.orientation.w);
            tf::Matrix3x3 m2(q2);
            double r2, p2, y2;
            m2.getRPY(r2, p2, y2);
            ang = abs(y2 - y);
            // If we are near point and correct angle, publish next waypoint
            if( (dist <= 0.9) && (ang <= 0.5) ) // 10.14
            {
              goal_msg.pose = listener.waypoints[i].pose;
              goal_msg.header.stamp = ros::Time::now();
              goal_msg.header.frame_id = "map";
              next_waypoint_pub.publish(goal_msg);
              ++i;
            }
          }

          if((listener.waypoints[i].pose.position.x <= 0.0001) && (listener.waypoints[i].pose.position.y <= 0.0001))
          {
            manage = false;
          }

        }

        loop_rate.sleep();
    }


}
