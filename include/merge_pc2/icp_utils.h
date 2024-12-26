#ifndef ICP_UTILS_H
#define ICP_UTILS_H

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
        nh.param<double>("icp_tf_epsilon", icp_tf_epsilon, 1e-8);
        nh.param<double>("icp_euclidean_fit_epsilon", icp_euclidean_fit_epsilo, 1e-5);
        nh.param<double>("icp_max_corr_d", icp_max_coor_d, 0.05);
    }
    ~icpUtils() = default;

    void icpAlign(PointCloudT::Ptr& pcT_out, const PointCloudT::Ptr& pcT) {
        pcl::IterativeClosestPoint<PointT, PointT> icp;

        icp.setInputSource(pcT);
        icp.setInputTarget(pcT_out);
        icp.setMaximumIterations(icp_max_iter);
        icp.setTransformationEpsilon(icp_tf_epsilon);
        icp.setEuclideanFitnessEpsilon(icp_euclidean_fit_epsilo);
        icp.setMaxCorrespondenceDistance(icp_max_coor_d);
        // icp.setNumberOfThreads(hardware_concurrency);

        PointCloudT::Ptr pcT_align(new PointCloudT());
        icp.align(*pcT_align);

        if (icp.hasConverged()) {
            PCL_INFO("ICP converged with score: %f\n", icp.getFitnessScore());
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
};

#endif // ICP_UTILS_H
