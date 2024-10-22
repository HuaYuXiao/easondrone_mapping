#include "octomap/octomapping.h"

using namespace octomapping;

int main(int argc, char** argv){
  ros::init(argc, argv, "octomapping");

  ros::NodeHandle nodehandle("~");

  OctoMapping MyOctoMapping;
  MyOctoMapping.init(nodehandle);

  ros::spin();

  return 0;
}
