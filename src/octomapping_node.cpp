#include "octomapping.h"


using namespace octomapping;


int main(int argc, char** argv){
  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle nodehandle("~");

  Global_Planner global_planner;
  global_planner.init(nodehandle);

  ros::spin();

  return 0;
}

