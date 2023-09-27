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

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose = msg->pose;
    }
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "go1_path_manager");
    ros::NodeHandle n;
    //ros::Rate loop_rate(100);
    ros::Rate loop_rate(10);

    Listener listener;

    ros::Subscriber search_path_sub = n.subscribe("/go1_path", 1000, &Listener::path_cb, &listener);
    ros::Subscriber pose_sub = n.subscribe("/localization_ros", 1000, &Listener::pose_cb, &listener);
    ros::Publisher next_waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

    geometry_msgs::PoseStamped goal_msg;
    geometry_msgs::Pose prev_goal;
    geometry_msgs::Pose pose;

    bool manage = false;

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
          // Global Paths received, start managing
          manage = true;
          prev_time = ros::Time::now();
      }

      current_time = ros::Time::now();


      if(manage)
      {
        // std::cout << "manage = true" << std::endl;
        // Now we have our global paths.
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

        if((listener.waypoints[i].pose.position.x <= 0.0001) && (listener.waypoints[i].pose.position.y <= 0.0001))
        {
          manage = false;
        }

      }

      loop_rate.sleep();
    }


}
