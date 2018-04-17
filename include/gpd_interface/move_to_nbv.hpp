#ifndef MOVE_ROBOT_HPP
#define MOVE_ROBOT_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <robot_helpers/robot.hpp>
#include <robot_helpers/geometry.hpp>
#include <robot_helpers/util.hpp>
#include <gpd_interface/construct_point_cloud.hpp>

#include "pcl_ros/point_cloud.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <Eigen/Geometry>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <basic_next_best_view/ExecuteAction.h>
#include <basic_next_best_view/BasicRequest.h>

#include <gpd/CloudIndexed.h>

using namespace Eigen;
using namespace robot_helpers;
using namespace std;

typedef typename actionlib::SimpleActionClient<basic_next_best_view::ExecuteAction> ExecuteActionClient;
typedef typename boost::shared_ptr<ExecuteActionClient> ExecuteActionClientPtr;

class MoveRobot
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber views_sub;
    ros::Subscriber poi_sub;
    ros::Publisher pub_final_point_cloud;
    ros::ServiceClient service_client;
    ros::Publisher cloud_sources_pub;

    gpd::CloudIndexed msg;

    gpd_interface::getPointCloud srv;

    Vector3d poi;
    std::vector<Vector3d> views;
    std::vector<Quaterniond> quaternions;
    std::vector<Vector3d> views_spherical;

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud_sampled;
    sensor_msgs::PointCloud2ConstPtr cloud_msg;

    uint number_of_views;
    double angle_offset;
    double *azimuth;

public:
    MoveRobot():number_of_views(3), angle_offset(M_PI/2)
    {
        azimuth = new double[number_of_views - 1];
        service_client = nh_.serviceClient<gpd_interface::getPointCloud>("/point_cloud_class/get_point_cloud");
	//ros::service::waitForService("/point_cloud_class/get_point_cloud", -1);
        views_sub = nh_.subscribe<geometry_msgs::PoseArray>("/cameras",1,&MoveRobot::viewsCallback,this);
        poi_sub = nh_.subscribe<geometry_msgs::Point>("/basic_next_best_view/poi",1,&MoveRobot::poiCallback,this);
        cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/basic_next_best_view/occupancy");

        pub_final_point_cloud = nh_.advertise<pcl::PCLPointCloud2>("move_robot/final_point_cloud", 1);

        cloud_sources_pub = nh_.advertise<gpd::CloudIndexed>("gpd/cloud_indexed", 1);


        //only for testing
//        final_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::io::loadPCDFile("/tmp/point_cloud/point_cloud2/final_pc.pcd", *final_cloud);
//        final_cloud->header.frame_id = "base_link";
//        removePlane();

//        Affine3d base_to_rgb = getTransform("xtion2_rgb_optical_frame", "base_link");
//        Matrix4d base_to_xtion = base_to_rgb.matrix();
//        pcl::transformPointCloud(*final_cloud, *final_cloud, base_to_xtion);
//        final_cloud->header.frame_id = "xtion2_rgb_optical_frame";

    }

    void viewsCallback(const geometry_msgs::PoseArray msg);
    void poiCallback(const geometry_msgs::Point msg);

    void publishFinalPointCloud();
    void multipleViewPointCloud();
    void publishMultipleViewPointCloud();

    bool moveToNextBestView();
    bool initializePointCloud();
    bool reconstructScene(uint i);

    void sendGoal();
    bool callGetPointCloudService();

    void viewsFromCartesianToSpherical();
    bool checkAzimuth(const Vector3d &views_spherical, uint &pc);

    void removePlane();
};

#endif // MOVE_ROBOT_HPP
