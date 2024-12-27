#ifndef ICP_UTILS_H
#define ICP_UTILS_H

#include <thread>
#include <ros/ros.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class icpUtils{
public:
    bool icp_enable;

    icpUtils(ros::NodeHandle nh){
        // The Iterative Closest Point algorithm
        nh.param<bool>("icp_enable", icp_enable, false);

        nh.param<int>("icp_max_iter", icp_max_iter, 1);
        this->icp.setMaximumIterations(icp_max_iter);
        nh.param<double>("icp_tf_epsilon", icp_tf_epsilon, 1e-8);
        this->icp.setTransformationEpsilon(icp_tf_epsilon);
        nh.param<double>("icp_euclidean_fit_epsilon", icp_euclidean_fit_epsilo, 1e-5);
        this->icp.setEuclideanFitnessEpsilon(icp_euclidean_fit_epsilo);
        nh.param<double>("icp_max_corr_d", icp_max_coor_d, 0.05);
        this->icp.setMaxCorrespondenceDistance(icp_max_coor_d);
        // TODO: Multi-thread not working
        // this->icp.setNumberOfThreads(std::thread::hardware_concurrency());
    }
    ~icpUtils() = default;

    void icpAlign(PointCloudT::Ptr& pcT_out, const PointCloudT::Ptr& pcT) {
        this->icp.setInputSource(pcT);
        this->icp.setInputTarget(pcT_out);

        PointCloudT::Ptr pcT_align(new PointCloudT());
        this->icp.align(*pcT_align);

        if (this->icp.hasConverged()) {
            PCL_INFO("ICP converged with score: %f\n", this->icp.getFitnessScore());
            *pcT_out += *pcT_align;
        }
        else {
            PCL_ERROR("ICP failed to converge within MaximumIterations!\n");
        }
    }

private:
    // The Iterative Closest Point algorithm
    int icp_max_iter;
    double icp_tf_epsilon;
    double icp_euclidean_fit_epsilo;
    double icp_max_coor_d;

    pcl::IterativeClosestPoint<PointT, PointT> icp;
};

#endif // ICP_UTILS_H
