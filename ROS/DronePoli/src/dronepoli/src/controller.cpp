
#include <drone.h>
#include <_convert.h>
#include <_keyboard.h>
#include <callback.h>



// main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "dronepoli_controller");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool > ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode > ("mavros/set_mode");
    ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State > ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe < sensor_msgs::NavSatFix > ("mavros/global_position/global", 1, globalPosition_cb);
    ros::Publisher goal_pos_pub = nh.advertise < geographic_msgs::GeoPoseStamped > ("mavros/setpoint_position/global", 10);
    // ros::Publisher goal_pos_pub = nh.advertise < mavros_msgs::GlobalPositionTarget > ("mavros/setpoint_position/global", 10);
    ros::Subscriber command_sub = nh.subscribe < std_msgs::Int16 > ("Hololens/command", 1, command_cb);
    ros::Subscriber route_sub = nh.subscribe < sensor_msgs::NavSatFix > ("Hololens/route", 1, route_cb);

    route.push_back({36.14424294, 128.39389622});
    route.push_back({36.14424294, 128.39375});
    route.push_back({36.14415, 128.39375});
    route.push_back({36.14415, 128.39389622});
    route.push_back({36.14424294, 128.39389622});


    while(px5.command_receive != 9){ // wait until pause command
        ROS_INFO_ONCE("Press [p] to connect FCU");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("[p] Pressed");
    

    for(int i=0; i<route.size(); i++){
        ROS_INFO("%.8f / %.8f", route[i][0],route[i][1]);
    }

    // wait for fcu connection
    while (ros::ok() && !px5.current_state.connected) {
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    // wait for position information
    while (ros::ok() && !global_position_received) {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");

    // while(px5.command_receive != 1){ // wait until take off command
    //     ROS_INFO_ONCE("Press [t] to take off");
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // ROS_INFO("[t] Pressed");

    // 1) send a few setpoints before starting
    geographic_msgs::GeoPoseStamped goal_position;
    goal_position.pose.position.latitude = px5.cur.latitude;
    goal_position.pose.position.longitude = px5.cur.longitude;
    goal_position.pose.position.altitude = px5.cur.altitude;
    for (int i=0; i<20; ++i) {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        ros::spinOnce();
        rate.sleep();
    }

    // 2) arm
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    } else {
        ROS_ERROR("Arming failed");
    }

    // 3) set mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.base_mode = 0;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD enabled");
    } else {
        ROS_ERROR("Failed to set OFFBOARD");
    }

    // 4) take off to 3m above ground
    double takeoff_altitude = 3.0;
    //px5.cur.altitude += takeoff_altitude;
    px5.goal.altitude = px5.goal.altitude + takeoff_altitude;

    while (ros::ok()) {

        ROS_INFO_THROTTLE(1, "[%d] Points remained", route.size());
        /*
        switch(px5.command_receive){
            case 1: // take off

                break;
            case 2: // land
                px5.cmd_mode.request.base_mode = 0;
                px5.cmd_mode.request.custom_mode = "AUTO.LAND";
                set_mode_client.call(px5.cmd_mode);

                px5.command_receive = 0; // Set to default
                ROS_INFO("[g] Land");
                break;

            case 3: // mission
                px5.mission_enable = true;

                px5.command_receive = 0; // Set to default
                ROS_INFO("[m] Mission Started");
                break;

            case 9: // pause
                px5.mission_enable = false;

                px5.command_receive = 0; // Set to default
                ROS_INFO("[p] Mission Paused");
                break;
        }
        */
        
        if(px5.ReachOnGoal(0.00000001)) // 경유 지점에 도착하면 다음 경유 지점 선택
        {
            route.erase(route.begin());
            ROS_INFO("Reached, %d Points remained", route.size());
        }

        if(route.empty()){
            px5.mission_enable = false;
            ROS_INFO_THROTTLE(1, "Mission Disabled (route empty)");

            px5.cmd_mode.request.base_mode = 0;
            px5.cmd_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(px5.cmd_mode);
        }
        else {
            if(abs(goal_position.pose.position.altitude-px5.cur.altitude)<0.5){
            px5.mission_enable = true;

            double degree = compute_bearing(px5.cur.latitude, px5.cur.longitude,route[0][0],route[0][1]);
            double radian = degree * PI / 180;
            if      (radian >= PI * 2) radian -= PI * 2;
            else if (radian <= 0)      radian += PI * 2;
            QUAT direction_quat = convert_euler_to_quaternion(radian);
            //px5.pose = direction_quat;

            ROS_INFO_THROTTLE(5, "Go to %.11f, %.11f / deg: %.6f, rad: %.6f", route[0][0], route[0][1], degree, radian);
            
            //px5.goal.latitude = route[0][0];
            //px5.goal.longitude = route[0][1];
            goal_position.pose.position.latitude = route[0][0];
            goal_position.pose.position.longitude = route[0][1];
            }
            else {goal_position.pose.position.latitude = px5.cur.latitude; goal_position.pose.position.longitude = px5.cur.longitude;}

        }
        
        ROS_INFO_THROTTLE(1, "Alt curr: %.5f goal: %.5f", px5.cur.altitude, goal_position.pose.position.altitude);

        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);

        //if(px5.mission_enable && abs(px5.goal.altitude-px5.cur.altitude)<0.5) goal_pos_pub.publish(px5.GoalMsg());
        //else goal_pos_pub.publish(px5.CurrMsg());
        

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("DronePoli_node OFF");
    return 0;
}