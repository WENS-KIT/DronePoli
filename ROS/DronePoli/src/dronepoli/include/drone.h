#pragma once

#include <vector>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <sensor_msgs/NavSatFix.h>

#include <geographic_msgs/GeoPoseStamped.h>
// #include <mavros_msgs/GlobalPositionTarget.h> -> GeoPoseStamped.h
// sensor_msgs::NavSatFix global_position; -> GeoPoseStamped

bool global_position_received = false;

// Route
std::vector<std::vector<double>> route;

struct POSITION{
    double latitude, longitude;
    double altitude;
};
struct QUAT{
  double w, x, y, z;
};

class PX5{
    public:
        double base_altitude = -1;

        int command_receive = 0;
        bool mission_enable = false;

        mavros_msgs::State current_state;

        mavros_msgs::SetMode cmd_mode;

        POSITION cur;
        POSITION goal;
        QUAT pose;

        geographic_msgs::GeoPoseStamped CurrMsg();
        geographic_msgs::GeoPoseStamped GoalMsg();

        bool ReachOnGoal(double threshold);
        int GoTo(double goal_latitude, double goal_longitude);

}px5;

bool PX5::ReachOnGoal(double threshold){
    if(abs(px5.goal.latitude -px5.cur.latitude ) < threshold
    && abs(px5.goal.longitude-px5.cur.longitude) < threshold) return true;

    else false;
}

geographic_msgs::GeoPoseStamped PX5::CurrMsg(){
    geographic_msgs::GeoPoseStamped goal_position;

    goal_position.header.stamp = ros::Time::now();
    goal_position.pose.position.latitude = px5.cur.latitude;
    goal_position.pose.position.longitude = px5.cur.longitude;
    goal_position.pose.position.altitude = px5.cur.altitude;
    goal_position.pose.orientation.x = px5.pose.x;
    goal_position.pose.orientation.y = px5.pose.y;
    goal_position.pose.orientation.z = px5.pose.z;
    goal_position.pose.orientation.w = px5.pose.w;
    return goal_position;
}
geographic_msgs::GeoPoseStamped PX5::GoalMsg(){
    geographic_msgs::GeoPoseStamped goal_position;

    goal_position.header.stamp = ros::Time::now();
    goal_position.pose.position.latitude = px5.goal.latitude;
    goal_position.pose.position.longitude = px5.goal.longitude;
    goal_position.pose.position.altitude = px5.goal.altitude;
    goal_position.pose.orientation.x = px5.pose.x;
    goal_position.pose.orientation.y = px5.pose.y;
    goal_position.pose.orientation.z = px5.pose.z;
    goal_position.pose.orientation.w = px5.pose.w;
    return goal_position;
}


int PX5::GoTo(double goal_latitude, double goal_longitude){

    PX5::goal.latitude = goal_latitude;
    PX5::goal.longitude = goal_longitude;

    return 1;
}