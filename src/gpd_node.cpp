#include <ros/ros.h>
#include <gpd_interface/move_to_nbv.hpp>
#include <gpd_interface/grasp_unknown_object.hpp>
#include <certh_pickup/PickUp.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "nbv");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4) ;
    spinner.start() ;

    while(true)
    {
        MoveRobot mvr;
        if (mvr.moveToNextBestView())
            ROS_INFO("Robot moved");
        else
            ROS_INFO("Next best view planning failed");

        mvr.publishFinalPointCloud();

        GraspUnknownObject guo;
        while (guo.isMessageEmpty()){
            ros::Duration(0.5).sleep();
        }

        if (guo.planTipToObject())
            ROS_INFO("Object was successfully picked up. Move to next object");
        else
            ROS_INFO("Could not pick up the object");

        int abort;
        std::cout << "Press 0 or 1" << std::endl;
        std::cout << "0:Abort" << std::endl;
        std::cout << "1:Pick up the next object" << std::endl;
        std::cin >> abort;
        if (abort == 0)
            break;
        else
            std::cout << "repeat the procedure" << std::endl;

    }

    ros::ServiceClient pickup_client = nh.serviceClient<certh_pickup::PickUp>("/pick_cloth_from_table_service/pickup");

    certh_pickup::PickUp srv;

    std::cout << "Picking up clothes..." << std::endl;
    srv.request.time_out = 1000;

    if(pickup_client.call(srv))
        std::cout << "Status:" << srv.response.status << std::endl;
    else
        std::cout << "service not called" << std::endl;

    
    ros::waitForShutdown();
    spinner.stop();



    return 0;
}
