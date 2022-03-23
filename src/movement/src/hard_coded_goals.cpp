#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <actionlib_msgs/GoalStatusArray.h>

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0.05;
geometry_msgs::TransformStamped map_transform;

ros::Publisher goal_pub;
ros::Subscriber map_sub;
ros::Subscriber status_sub;
uint8_t statusOfRoute;

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    ROS_INFO("Callback triggered");
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

    map_resolution = msg_map->info.resolution;
    map_transform.transform.translation.x = msg_map->info.origin.position.x;
    map_transform.transform.translation.y = msg_map->info.origin.position.y;
    map_transform.transform.translation.z = msg_map->info.origin.position.z;

    map_transform.transform.rotation = msg_map->info.origin.orientation;

    //tf2::poseMsgToTF(msg_map->info.origin, map_transform);

    const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }

}

void statusCallback(const actionlib_msgs::GoalStatusArrayPtr& status) {
    if(status->status_list.size() > 0){
        statusOfRoute = status->status_list[0].status;
    }
    // ROS_INFO("Shitshow %d", 0);
}

void moveTo(int x, int y) {

    int v = (int)cv_map.at<unsigned char>(y, x);

	if (v != 255) {
		ROS_WARN("Unable to move to (x: %d, y: %d), not reachable", x, y);
		return;
	}

    ROS_INFO("resolution is %f", map_resolution);
    geometry_msgs::Point pt;
    geometry_msgs::Point transformed_pt;

    pt.x = (float)x * map_resolution;
    pt.y = (float)(cv_map.rows - y) * map_resolution;
    pt.z = 0.0;
    
    tf2::doTransform(pt, transformed_pt, map_transform);
    
    //geometry_msgs::Point transformed = map_transform * pt;

    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1;
    goal.pose.position.x = transformed_pt.x;
    goal.pose.position.y = transformed_pt.y;
    goal.header.stamp = ros::Time::now();

    ROS_INFO("Moving to (x: %f, y: %f)", transformed_pt.x, transformed_pt.y);

    goal_pub.publish(goal);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "hard_goals");
    ros::NodeHandle n;

	ros::Rate rate(10);

    map_sub = n.subscribe("map", 10, &mapCallback);
    status_sub = n.subscribe("/move_base/status", 10, &statusCallback);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    int goals[8][2] = {
        {286, 268},
        {246, 232},
        {242, 200},
        {288, 190},
        {299, 230},
        {313, 208},
        {326, 258},
        {283, 262}
    };
    //int goals[5][2] = {{250, 200}, {258, 230}, {284, 263}, {324, 250}, {291, 225}};

    int i = 0;
    int Moving = 0;
    while(ros::ok()) {
        ros::spinOnce();
        
        // ta Moving je zato ker ko mu das ukaz da se premakne ne zacne takoj
        // in mores pol cakat da se zacne premikat da lahko spet cakas da je 
        // status 3
        ROS_INFO("Status : %d", statusOfRoute);
        if (statusOfRoute == 3 && !Moving && i < 8) {
            Moving = 1;
            moveTo(goals[i][0], goals[i][1]);
            i++;
        }
        
        if (Moving && statusOfRoute == 1) {
            Moving = 0;
        }

        rate.sleep();
    }
    return 0;

}