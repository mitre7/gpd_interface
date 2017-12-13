#include <ros/ros.h>

#include <gpd_interface/grasp_unknown_object.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_unknown_object");

    ros::NodeHandle nh;

    GraspUnknownObject guo;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (guo.isMessageEmpty()){
        ros::Duration(0.5).sleep();
    }

    if (guo.planTipToObject()) {
        ROS_INFO("Object was successfully picked up");
    }
    else {
        ROS_INFO("Could not pick up the object");
    }

    ros::waitForShutdown();
    spinner.stop();

    return 0;
}
