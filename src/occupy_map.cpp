#include <occupy_map.h>

namespace Global_Planning
{
// 初始化函数
void Occupy_map::init(ros::NodeHandle& nh)
{
    // 地图原点
    nh.param("map/origin_x", origin_(0), 0.0);
    nh.param("map/origin_y", origin_(1), 0.0);
    nh.param("map/origin_z", origin_(2), 0.0);
    // 地图实际尺寸，单位：米
    nh.param("map/map_size_x", map_size_3d_(0), 6.0);
    nh.param("map/map_size_y", map_size_3d_(1), 6.0);
    nh.param("map/map_size_z", map_size_3d_(2), 3.0);
    // 地图分辨率，单位：米
    nh.param("map/resolution", resolution_,  0.05);


    // 发布 地图rviz显示
    global_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/prometheus/planning/global_pcl",  10);
 
    // 发布二维占据图？

    this->inv_resolution_ = 1.0 / resolution_;
    for (int i = 0; i < 3; ++i)
    {
        // 占据图尺寸 = 地图尺寸 / 分辨率
        grid_size_(i) = ceil(map_size_3d_(i) / resolution_);
    }
        
    // 占据容器的大小 = 占据图尺寸 x*y*z
    occupancy_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);

    min_range_ = origin_;
    max_range_ = origin_ + map_size_3d_;   
}

// 地图更新函数 - 输入：全局点云
void Occupy_map::map_update_gpcl(const sensor_msgs::PointCloud2ConstPtr & global_point)
{
    has_global_point = true;
    global_env_ = global_point;
}

// 地图更新函数 - 输入：局部点云
void Occupy_map::map_update_lpcl(const sensor_msgs::PointCloud2ConstPtr & local_point, const nav_msgs::Odometry & odom)
{
    has_global_point = true;
// 待江涛更新
// 将传递过来的局部点云转为全局点云
}

// 地图更新函数 - 输入：laser
void Occupy_map::map_update_laser(const sensor_msgs::LaserScanConstPtr & local_point, const nav_msgs::Odometry & odom)
{
    has_global_point = true;
// 待更新
// 将传递过来的数据转为全局点云
}

// 当global_planning节点接收到点云消息更新时，进行设置点云指针并膨胀
// Astar规划路径时，采用的是此处膨胀后的点云（setOccupancy只在本函数中使用）
void Occupy_map::inflate_point_cloud(void)
{
    if(!has_global_point)
    { 
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Occupy_map [inflate point cloud]: don't have global point, can't inflate!\n");
        return;
    }

    // 发布未膨胀点云
    global_pcl_pub.publish(*global_env_);

    // 转化为PCL的格式进行处理
    pcl::PointCloud<pcl::PointXYZ> latest_global_cloud_;
    pcl::fromROSMsg(*global_env_, latest_global_cloud_);

    if ((int)latest_global_cloud_.points.size() == 0)  
    {return;}

    Eigen::Vector3d p3d;

    // 遍历全局点云中的所有点
    for (size_t i = 0; i < latest_global_cloud_.points.size(); ++i) 
    {
        // 取出第i个点
        p3d(0) = latest_global_cloud_.points[i].x;
        p3d(1) = latest_global_cloud_.points[i].y;
        p3d(2) = latest_global_cloud_.points[i].z;

        // 若取出的点不在地图内（膨胀时只考虑地图范围内的点）
        if(!isInMap(p3d))
        {
            continue;
        }
    }

}

void Occupy_map::setOccupancy(Eigen::Vector3d pos, int occ) 
{
    if (occ != 1 && occ != 0) 
    {
        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "occ value error!\n");
        return;
    }

    if (!isInMap(pos))
    {
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


void Occupy_map::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i &id) 
{
    for (int i = 0; i < 3; ++i)
    {
        id(i) = floor((pos(i) - origin_(i)) * inv_resolution_);
    }
       
}

void Occupy_map::indexToPos(Eigen::Vector3i id, Eigen::Vector3d &pos) 
{
    for (int i = 0; i < 3; ++i)
    {
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
    }
}

int Occupy_map::getOccupancy(Eigen::Vector3d pos) 
{
    if (!isInMap(pos))
    {
        return -1;
    }
        
    Eigen::Vector3i id;
    posToIndex(pos, id);

    return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}

int Occupy_map::getOccupancy(Eigen::Vector3i id) 
{
    if (id(0) < 0 || id(0) >= grid_size_(0) || id(1) < 0 || id(1) >= grid_size_(1) || id(2) < 0 ||
        id(2) >= grid_size_(2))
    {
        return -1;
    }
        
    return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}
}
