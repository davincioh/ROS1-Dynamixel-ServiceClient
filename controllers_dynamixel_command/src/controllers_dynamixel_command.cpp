#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controllers_dynamixel_command");
  if (argc != 3)
  {
    ROS_INFO("usage : controllers_dynamixel_command ID PositionValue");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

  dynamixel_workbench_msgs::DynamixelCommand srv;

  /* dynamixel_workbench_msgs/DynamixelCommand.srv
  string command

  uint8 id
  string addr_name
  int32 value
  ---
  bool comm_result
*/

  std::string item_command = "";
  std::string item_addr = "Goal_Position";

  srv.request.command = item_command;
  srv.request.id = atoll(argv[1]);
  srv.request.addr_name = item_addr;
  srv.request.value = atoll(argv[2]);

  if (client.call(srv))
  {
    ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
    ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
  }
  else
  {
    ROS_ERROR("Failed to call dynamixel_command");
    return 1;
  }
  return 0;
}