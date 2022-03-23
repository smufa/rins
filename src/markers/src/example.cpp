#include "ros/ros.h"
#include "markers/Service.h"

#include <sstream>

#define SERVICE "marker_placer_node/placer"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_placer_example");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<markers::Service>(SERVICE);

  markers::Service srv;

  srv.request.x = 10;
  srv.request.y = 5;

  ros::service::waitForService(SERVICE, 1000);

  ROS_INFO("Sending...");

  if (client.call(srv))
  {
    ROS_INFO("The service RESPONDED");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
