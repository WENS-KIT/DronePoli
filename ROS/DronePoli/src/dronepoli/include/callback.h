#pragma once

#include <drone.h>

// callback functions
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {

    px5.cur.latitude = msg->latitude;
    px5.cur.longitude = msg->longitude;
    px5.cur.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    if(px5.base_altitude == -1) px5.base_altitude = px5.cur.altitude;

    global_position_received = true;
    /*
    global_position.pose.position.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    global_position.header.stamp = msg->header.stamp;
    global_position.pose.position.latitude = msg->latitude;
    global_position.pose.position.longitude = msg->longitude;
    global_position_received = true;
    */
    ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", px5.cur.latitude, px5.cur.longitude, px5.cur.altitude);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    px5.current_state = *msg;
}

void command_cb(const std_msgs::Int16::ConstPtr& msg) {
    px5.command_receive = msg->data;
}
void route_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if(msg->latitude == 500.0 || msg->longitude == 500.0){
        route.clear();
    }
    else if(msg->latitude == 600.0 || msg->longitude == 600.0){
        for(int i=0; i<route.size(); i++){
        ROS_INFO("%d) %.11f, %.11f", i, route[i][0], route[i][1]);
        }
        ROS_INFO("Route Received");
    }
    else{
        route.push_back({msg->latitude, msg->longitude});
    }
}