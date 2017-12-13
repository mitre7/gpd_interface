#include <gpd_interface/construct_point_cloud.hpp>
#include <ros/ros.h>

using namespace Eigen;
using namespace robot_helpers;
using namespace camera_helpers;
using namespace std;
using namespace cvx::util;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "posting") ;

    PointCloudClass pcc;

//    if (!pcc.estimatePointCloud()){
//        cout << "Cannot estimate the point cloud" << endl;
//        return 0;
//    }

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        pcc.pubPointCloud();
        pcc.pubPoi();

        ros::spinOnce();
    }
    return 0;

}

