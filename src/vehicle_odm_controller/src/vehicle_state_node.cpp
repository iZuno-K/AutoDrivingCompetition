// #include "invehicle_state_node.hpp"
#include "vehicle_state.hpp"
#define __APP_NAME__ "vehicle_state_publiser"

// using namespace vehicle_odm_controller;

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);
  VehicleOdmController vsp;
  vsp.run();
  return 0;
}