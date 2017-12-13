#ifndef RECONSTRUCT_OBJECT_HPP
#define RECONSTRUCT_OBJECT_HPP

#include <robot_helpers/robot.hpp>
#include <robot_helpers/geometry.hpp>
#include <robot_helpers/util.hpp>
#include <camera_helpers/gphoto2_capture.hpp>
#include <camera_helpers/openni_capture.hpp>
#include <cvx/util/camera/camera.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include <gpd_interface/getPointCloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <camera_helpers/openni_capture.hpp>

using namespace Eigen;
using namespace robot_helpers;
using namespace camera_helpers;
using namespace std;
using namespace cvx::util;

void getPlane(const image_geometry::PinholeCameraModel &cam, cv::Mat &mask);
static cv::Mat maskFromCorners(const cv::Size &sz, const cv::Point2d &p1, const cv::Point2d &p2, const cv::Point2d &p3, const cv::Point2d &p4);
void depthToCloud(const cv::Mat &depth, const cv::Mat &mask, const cvx::util::PinholeCamera &model_, pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &frame_id);
void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output);

class PointCloudClass
{
private:
    ros::Publisher pub_point_cloud, pub_point;
    ros::Subscriber sub;
    ros::NodeHandle n;

    ros::ServiceServer get_point_cloud_server;

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
    sensor_msgs::PointCloud2ConstPtr cloud_msg;
    pcl::PCLPointCloud2::Ptr cloud_filtered;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;

    OpenNICaptureRGBD grabber_;

    double height_offset; //

public:
    PointCloudClass():
    grabber_("xtion2")
    ,height_offset(0.0)
    {
        grabber_.connect();
        cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>() );
        pub_point_cloud = n.advertise<pcl::PCLPointCloud2>("/basic_next_best_view/occupancy", 1);
        pub_point = n.advertise<geometry_msgs::Point>("/basic_next_best_view/poi", 100);
        get_point_cloud_server = n.advertiseService("/point_cloud_class/get_point_cloud", &PointCloudClass::calcPointCloud, this);
    }

    bool estimatePointCloud();
    void pubPointCloud();
    void pubPoi();
    void depthToPointCloud(const PinholeCamera &model_, const cv::Mat &depth, const cv::Mat &mask, pcl::PointCloud<pcl::PointXYZI> &cloud);

    bool calcPointCloud(gpd_interface::getPointCloud::Request &req, gpd_interface::getPointCloud::Response &res);

};
#endif // RECONSTRUCT_OBJECT_HPP
