#include "markers/Service.h"
#include "ros/ros.h"

#include <string>
#include <iostream>
#include <algorithm>

ros::Publisher markers_pub;

bool placeMarker(markers::Service::Request &req,
                 markers::Service::Response &res)
{
  res.status = true;
  ROS_INFO("placing: %d %d", req.x, req.y);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_placer_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("marker_placer_node/placer", placeMarker);
  ROS_INFO("Marker placer service up");
  ros::spin();

  return 0;
}
