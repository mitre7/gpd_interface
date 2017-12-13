#include <gpd_interface/move_to_nbv.hpp>


void MoveRobot::poiCallback(const geometry_msgs::Point msg)
{
    poi.x() = msg.x;
    poi.y() = msg.y;
    poi.z() = msg.z;
}


void MoveRobot::viewsCallback(const geometry_msgs::PoseArray msg)
{
    for (uint i = 0; i < msg.poses.size(); i++)
    {
        Vector3d temp;
        temp.x() = msg.poses[i].position.x;
        temp.y() = msg.poses[i].position.y;
        temp.z() = msg.poses[i].position.z;
        views.push_back(temp);

        Quaterniond temp_q;
        temp_q.x() = msg.poses[i].orientation.x;
        temp_q.y() = msg.poses[i].orientation.y;
        temp_q.z() = msg.poses[i].orientation.z;
        temp_q.w() = msg.poses[i].orientation.w;
        quaternions.push_back(temp_q);
    }
}


bool MoveRobot::initializePointCloud()
{
    final_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    final_cloud_sampled.reset(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromROSMsg(*cloud_msg,*final_cloud);

    image_geometry::PinholeCameraModel cam ;
    ros::Time ts;
    OpenNICaptureRGBD grabber_("xtion2");

    grabber_.connect();

    cv::Mat rgb, depth, mask;
    if ( !grabber_.grab(rgb, depth, ts, cam) ) return false ;

    getPlane(cam, mask);

    PinholeCamera camera(525, 525, 640/2.0, 480/2, cv::Size(640, 480)) ;

    pcl::PointCloud<pcl::PointXYZ> initial_point_cloud;
    depthToCloud(depth, mask, camera, initial_point_cloud, "xtion2_rgb_optical_frame");

    pcl::copyPointCloud(initial_point_cloud, *final_cloud);

    //tranform points to robot coordinate system
    Affine3d rgb_to_base = getTransform("base_link", "xtion2_rgb_optical_frame");
    Matrix4d xtion_to_base = rgb_to_base.matrix();
    pcl::transformPointCloud(*final_cloud, *final_cloud, xtion_to_base);
    final_cloud->header.frame_id = "base_link";

    //downsample pointcloud
    std::vector< int > index;
    pcl::removeNaNFromPointCloud(*final_cloud, *final_cloud, index);
    downsample(final_cloud, final_cloud_sampled);

    pcl::io::savePCDFile("/tmp/pc0.pcd", *final_cloud_sampled);

    grabber_.disconnect();

    return true;

}

bool MoveRobot::reconstructScene(uint i)
{
    image_geometry::PinholeCameraModel cam ;
    ros::Time ts;
    OpenNICaptureRGBD grabber_("xtion2");

    grabber_.connect();

    cv::Mat rgb, depth, mask;
    if ( !grabber_.grab(rgb, depth, ts, cam) ) return false ;

    getPlane(cam, mask);

    PinholeCamera camera(525, 525, 640/2.0, 480/2, cv::Size(640, 480)) ;

    pcl::PointCloud<pcl::PointXYZ> next_view_cloud;
    depthToCloud(depth, mask, camera, next_view_cloud, "xtion2_rgb_optical_frame");

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::copyPointCloud(next_view_cloud, *tmp_cloud);

    /*
    //-------------------------------//
    //save point cloud having as reference the camera coordinate system
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_(new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::copyPointCloud(*tmp_cloud, *tmp_cloud_);
    tmp_cloud_->header.frame_id = "xtion2_rgb_optical_frame";
    std::vector< int > index_;
    pcl::removeNaNFromPointCloud(*tmp_cloud_, *tmp_cloud_, index_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_sampled_(new pcl::PointCloud<pcl::PointXYZ>() );
    downsample(tmp_cloud_, tmp_cloud_sampled_);
    pcl::io::savePCDFile(("/tmp/pc_temp"+std::to_string(i)+".pcd"), *tmp_cloud_sampled_);
    //-------------------------------//
    */

    //Transform point cloud from xtion optical frame to base link in order to add the point clouds.
    Affine3d rgb_to_base = getTransform("base_link", "xtion2_rgb_optical_frame");
    Matrix4d xtion_to_base = rgb_to_base.matrix();
    pcl::transformPointCloud(*tmp_cloud, *tmp_cloud, xtion_to_base);
    tmp_cloud->header.frame_id = "base_link";

    //downsample pointcloud
    std::vector< int > index;
    pcl::removeNaNFromPointCloud(*tmp_cloud, *tmp_cloud, index);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>() );
    downsample(tmp_cloud, tmp_cloud_sampled);

    pcl::io::savePCDFile(("/tmp/pc"+std::to_string(i)+".pcd"), *tmp_cloud_sampled);

    //align two point clouds
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(0.02);
    icp.setInputTarget(final_cloud_sampled);
    icp.setInputSource(tmp_cloud_sampled);

    icp.setMaximumIterations(10);
    icp.align(*tmp_cloud_sampled);

    *final_cloud_sampled = *final_cloud_sampled + *tmp_cloud_sampled;

    pcl::io::savePCDFile("/tmp/final_pc.pcd", *final_cloud_sampled);

//    cout << "Point cloud saved" << endl;

    grabber_.disconnect();

    return true;

}


bool MoveRobot::moveToNextBestView()
{
    initializePointCloud();
    uint pc = 1; //counts the number of multiple camera views deployed

    string arm_name = "r2";
    RobotArm arm(arm_name);

    arm.setRobotSpeed(0.6);

    RobotArm::Plan plan;

    while( pc <= number_of_views)
    {
        //construct and post /basic_next_view/occupancy
        callGetPointCloudService();

        //create a goal for the next view actionlib
        sendGoal();

        //convert views from cartesian to spherical
        viewsFromCartesianToSpherical();

//        double diff = 0.4;

        for (uint i = 0; i < views.size(); i++)
        {
//            Vector3d temp = getTransform("base_link", "xtion2_rgb_optical_frame").translation();
//            Vector3d temp(getTransform("base_link", "xtion2_rgb_optical_frame").translation().x(),getTransform("base_link", "xtion2_rgb_optical_frame").translation().y(),getTransform("base_link", "xtion2_rgb_optical_frame").translation().z());
//            if ( abs(views[i].x() - temp.x()) < diff && abs(views[i].y() - temp.y()) < diff && abs(views[i].y() - temp.y()) < diff)
//                continue;

            if (checkAzimuth(views_spherical[i], pc))
                continue;

            if ( !arm.planXtionIK(views[i] , quaternions[i], plan) )
            {
                cerr << "can't plan to location:" << views[i] << endl;
                continue;
            }
            else
            {
                if ( arm.execute(plan) )
                {
                    cout << "xtion at: " << getTransform("base_link", "xtion2_rgb_optical_frame").translation().adjoint() << endl;
                    if (reconstructScene(pc))\
                    {
                        azimuth[pc - 1] = views_spherical[i].y();
                        cout << "Phi = " << views_spherical[i].y() << endl;
                        cout << "Base link cs: " << endl << views[i] << endl;
                        cout << "Tray cs: " << endl << getTransform("tray_bottom", "base_link")* views[i] << endl;
                        pc++;
                        break;
                    }
                }
                continue;
            }
        }
    }

    callGetPointCloudService(); //only for vizualization in rviz

    final_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    downsample(final_cloud_sampled, final_cloud);
    pcl::io::savePCDFile("/tmp/final_pc.pcd", *final_cloud);

    arm.moveHome();

//    removePlane();

//    Affine3d base_to_rgb = getTransform("xtion2_rgb_optical_frame", "base_link");
//    Matrix4d base_to_xtion = base_to_rgb.matrix();
//    pcl::transformPointCloud(*final_cloud, *final_cloud, base_to_xtion);
//    final_cloud->header.frame_id = "xtion2_rgb_optical_frame";

    if (pc == (number_of_views + 1))
        return true;
    else
        return false;
}

void MoveRobot::publishFinalPointCloud()
{
    cout << "Publishing Point Cloud..." << endl;
    pub_final_point_cloud.publish(*final_cloud);
}

void MoveRobot::sendGoal()
{
    views.clear();
    quaternions.clear();

    ExecuteActionClientPtr ac(new ExecuteActionClient("/basic_next_best_view/execute", true));

    ROS_INFO("Waiting for action server to start.");
    ac->waitForServer(); //will wait for infinite time

    basic_next_best_view::ExecuteGoal goal;

    goal.request.type = 3;
    goal.request.has_index_range = false;

    ac->sendGoal(goal);

    bool finished_before_timeout = ac->waitForResult(ros::Duration(10.0));

    if (finished_before_timeout)
    {
        for (uint i = 0; i < ac->getResult()->response.best_poses.poses.size()  ; i++)
        {
            Eigen::Vector3d temp;
            temp.x() = ac->getResult()->response.best_poses.poses[i].position.x;
            temp.y() = ac->getResult()->response.best_poses.poses[i].position.y;
            temp.z() = ac->getResult()->response.best_poses.poses[i].position.z;
            views.push_back(temp);

            Eigen::Quaterniond temp_d;
            temp_d.x() = ac->getResult()->response.best_poses.poses[i].orientation.x;
            temp_d.y() = ac->getResult()->response.best_poses.poses[i].orientation.y;
            temp_d.z() = ac->getResult()->response.best_poses.poses[i].orientation.z;
            temp_d.w() = ac->getResult()->response.best_poses.poses[i].orientation.w;
            quaternions.push_back(temp_d);
        }
    }

}

bool MoveRobot::callGetPointCloudService()
{
    if(service_client.call(srv))
        return true;
    else
        return false;
}

void MoveRobot::viewsFromCartesianToSpherical()
{
    views_spherical.clear();

    Affine3d tray_frame = getTransform("tray_bottom", "base_link");
    for (uint i = 0; i < views.size(); i++)
    {
        Vector3d temp;
        temp = tray_frame * views[i];
        Vector3d spherical;
        spherical.x() = temp.norm();
        spherical.y() = atan2(temp.y(), temp.x()); //phi
        if (spherical.y() < 0)
            spherical.y() += 2*M_PI;
        spherical.z() = atan2(temp.z(),temp.norm()); //theta
//        cout << "Spherical:" << endl << spherical << endl;
        views_spherical.push_back(spherical);
    }
}

bool MoveRobot::checkAzimuth(const Vector3d &views_spherical, uint &pc)
{
    for (uint i = 1; i < pc; i++)
    {
        if ( abs(views_spherical.y() - azimuth[i -1]) < angle_offset )
            return true;
        else
            continue;
    }
    return false;
}

void MoveRobot::removePlane()
{
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices());

    float x, y, z;

    for(unsigned int i = 0; i < final_cloud->points.size(); i++)
    {
        x = final_cloud->points[i].x;
        y = final_cloud->points[i].y;
        z = final_cloud->points[i].z;

        //if(dist > r || z < offset)
        if(z < 0.77 /*|| x > xmax || x < xmin || y < ymax || y > ymin*/)
            outliers->indices.push_back(i);
    }

    //!< Extract the outliers from the point cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(final_cloud);
    extract.setIndices(outliers);
    extract.setNegative(true);
    extract.filter(*final_cloud);
}


//only for testing!!!
void MoveRobot::multipleViewPointCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::io::loadPCDFile("/tmp/pc0.pcd", *cloud1);
    cloud1->header.frame_id = "base_link";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::io::loadPCDFile("/tmp/pc1.pcd", *cloud2);
    cloud1->header.frame_id = "base_link";

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(0.02);
    icp.setInputTarget(cloud1);
    icp.setInputSource(cloud2);

    icp.setMaximumIterations(10);
    icp.align(*cloud2);

    uint size1 = cloud1->size();
    cout << "Point cloud1 has " << size1 << " points" << endl;

    *cloud1 = *cloud1 + *cloud2;
    pcl::io::savePCDFile("/tmp/two_views_final.pcd", *cloud1);

    uint size1_2 = cloud1->size();
    cout << "Point cloud1_2 has " << size1_2 << " points" << endl;

    pcl::toROSMsg(*cloud1, msg.cloud_sources.cloud);

    cout << "Cloud message ok" << endl;

    for (uint i = 0; i < size1_2; i++)
    {
        std_msgs::Int64 camera_index;
        if (i < size1)
            camera_index.data = 0;
        else
            camera_index.data = 1;

        msg.cloud_sources.camera_source.push_back(camera_index);
    }

    geometry_msgs::Point p;

    //first view
    p.x = 0.543;
    p.y = -0.906;
    p.z = 1.729;
    msg.cloud_sources.view_points.push_back(p);
\
    //second view
    p.x = 0.807;
    p.y = -0.98;
    p.z = 1.44;
    msg.cloud_sources.view_points.push_back(p);

}

void MoveRobot::publishMultipleViewPointCloud()
{
    cout << "Publishing indexed point cloud" << endl;
    cloud_sources_pub.publish(msg);
}
