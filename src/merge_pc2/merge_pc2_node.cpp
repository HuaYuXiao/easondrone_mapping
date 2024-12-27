#include "merge_pc2/merge_pc2.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_pc2");
    ros::NodeHandle nh("~");
    
    mergePC2 merge_pc2(nh);

    std::thread process_thread(&mergePC2::sync_process, &merge_pc2);

    ros::spin();

    return 0;
}
