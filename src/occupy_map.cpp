#include <occupy_map.h>


namespace Global_Planning{
// 初始化函数
void Occupy_map::init(ros::NodeHandle& nodehandle){
    // 地图原点
    nodehandle.param("map/origin_x", origin_(0), 0.0);
    nodehandle.param("map/origin_y", origin_(1), 0.0);
    nodehandle.param("map/origin_z", origin_(2), 0.0);

    // 地图实际尺寸，单位：米
    nodehandle.param("map/map_size_x", map_size_3d_(0), 6.0);
    nodehandle.param("map/map_size_y", map_size_3d_(1), 6.0);
    nodehandle.param("map/map_size_z", map_size_3d_(2), 3.0);

    min_range_ = origin_;
    max_range_ = origin_ + map_size_3d_;

    // 地图分辨率，单位：米
    nodehandle.param("map/resolution", resolution,  0.05);

    for (int i = 0; i < 3; ++i){
        // 占据图尺寸 = 地图尺寸 / 分辨率
        grid_size_(i) = ceil(map_size_3d_(i) / resolution);
    }

    // 占据容器的大小 = 占据图尺寸 x*y*z
    occupancy_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);

    // 发布地图rviz显示
    // 发布二维占据图？
    // TODO 似乎只发送给了rviz，而不是全局消息？消息类型是否正确？
    global_pcl_pub = nodehandle.advertise<sensor_msgs::PointCloud2>("/prometheus/planning/global_pcl",  10);
}


// 地图更新函数 - 输入：全局点云
void Occupy_map::map_update_gpcl(const sensor_msgs::PointCloud2ConstPtr & global_point){
    global_env_ptr = global_point;

     // 发布未膨胀点云
    global_pcl_pub.publish(*global_env_ptr);
}


// 地图更新函数 - 输入：laser
void Occupy_map::map_update_laser(const sensor_msgs::LaserScanConstPtr & local_point, const nav_msgs::Odometry & odom){
    // TODO 将传递过来的数据转为全局点云

    // 发布未膨胀点云
    global_pcl_pub.publish(*global_env_ptr);
}


bool Occupy_map::isInMap(Eigen::Vector3d pos){
    // min_range就是原点，max_range就是原点+地图尺寸
    // 比如设置0,0,0为原点，[0,0,0]点会被判断为不在地图里
    // TODO 开环建图可以不管这个问题吧？

    if (pos(0) < min_range_(0) + 1e-4 ||
        pos(1) < min_range_(1) + 1e-4 ||
        pos(2) < min_range_(2) + 1e-4 ||
        pos(0) > max_range_(0) - 1e-4 ||
        pos(1) > max_range_(1) - 1e-4 ||
        pos(2) > max_range_(2) - 1e-4){
        return false;
    }

    return true;
}


void Occupy_map::setOccupancy(Eigen::Vector3d pos, int occ){
    if (occ != 1 && occ != 0){
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "occ value error!\n");
        return;
    }

    if (!isInMap(pos)){
        return;
    }

    Eigen::Vector3i id;
    posToIndex(pos, id);

    // 设置为占据/不占据 索引是如何索引的？ [三维地图 变 二维数组]
    // 假设10*10*10米，分辨率1米，buffer大小为 1000 （即每一个占格都对应一个buffer索引）
    // [0.1,0.1,0.1] 对应索引为[0,0,0]， buffer索引为 0  
    // [9.9,9.9,9.9] 对应索引为[9,9,9]， buffer索引为 900+90+9 = 999
    occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)] = occ;
}


void Occupy_map::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i &id){
    for (int i = 0; i < 3; ++i){
        id(i) = floor((pos(i) - origin_(i)) / resolution);
    }
}


void Occupy_map::indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos){
    for (int i = 0; i < 3; ++i){
        pos(i) = (id(i) + 0.5) * resolution + origin_(i);
    }
}


int Occupy_map::getOccupancy(Eigen::Vector3d pos){
    if (!isInMap(pos)){
        return -1;
    }
        
    Eigen::Vector3i id;
    posToIndex(pos, id);

    return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}


int Occupy_map::getOccupancy(Eigen::Vector3i id){
    if (id(0) < 0 ||
        id(0) >= grid_size_(0) ||
        id(1) < 0 ||
        id(1) >= grid_size_(1) ||
        id(2) < 0 ||
        id(2) >= grid_size_(2)){
        return -1;
    }
        
    return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}
}
