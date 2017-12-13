#include <gpd_interface/move_to_nbv.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_views");

    ros::NodeHandle nh;

    MoveRobot mvr;

    ros::AsyncSpinner spinner(4) ;
    spinner.start() ;

    if (mvr.moveToNextBestView())
    {
        ROS_INFO("Robot moved");
    }

//    mvr.multipleViewPointCloud();
    ros::Rate r(0.5);
    while (ros::ok())
    {
        mvr.publishFinalPointCloud();
//        mvr.publishMultipleViewPointCloud();
        r.sleep();
    }

    ros::waitForShutdown();
    spinner.stop();

    return 0;
}
