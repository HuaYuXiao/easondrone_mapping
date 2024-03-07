#include <ros/ros.h>
#include "global_planner.h"


using namespace Global_Planning;


int main(int argc, char** argv){
  ros::init(argc, argv, "global_planner");

  ros::NodeHandle nodehandle("~");

  Global_Planner global_planner;
  global_planner.init(nodehandle);

  ros::spin();

  return 0;
}

