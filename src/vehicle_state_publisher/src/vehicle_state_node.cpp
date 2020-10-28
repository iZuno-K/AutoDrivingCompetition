// #include "invehicle_state_node.hpp"
#include "vehicle_state.hpp"
#define __APP_NAME__ "vehicle_state_publiser"

// using namespace vehicle_state_publisher;

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);
  VehicleStatePublisher vsp;
  vsp.run();
  return 0;
}