#include "global_planner.h"


using namespace Global_Planning;


int main(int argc, char** argv){
  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle nodehandle("~");

  Global_Planner global_planner;
  global_planner.init(nodehandle);

  ros::spin();

  return 0;
}

