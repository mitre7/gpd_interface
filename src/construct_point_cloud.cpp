#include <gpd_interface/construct_point_cloud.hpp>

static cv::Mat maskFromCorners(const cv::Size &sz, const cv::Point2d &p1, const cv::Point2d &p2, const cv::Point2d &p3, const cv::Point2d &p4) {
    cv::Mat mask = cv::Mat::zeros(sz, CV_8UC1) ;

    vector<vector<cv::Point>> pts(1) ;
    pts[0].push_back(cv::Point(p1.x, p1.y)) ;
    pts[0].push_back(cv::Point(p2.x, p2.y)) ;
    pts[0].push_back(cv::Point(p3.x, p3.y)) ;
    pts[0].push_back(cv::Point(p4.x, p4.y)) ;

    cv::fillPoly(mask, pts, cv::Scalar(255));

    return mask ;
}

void getPlane(const image_geometry::PinholeCameraModel &cam, cv::Mat &mask)
{
    Affine3d cam_frame = getTransform("xtion2_rgb_optical_frame", "base_link") ;

    Vector3d c1 = cam_frame * getTransform("base_link", "tray_wall_1").translation();
    Vector3d c2 = cam_frame * getTransform("base_link", "tray_wall_2").translation();
    Vector3d c3 = cam_frame * getTransform("base_link", "tray_wall_3").translation();
    Vector3d c4 = cam_frame * getTransform("base_link", "tray_wall_4").translation();

    cv::Point2d p1 = cam.project3dToPixel(cv::Point3d(c1.x(), c1.y(), c1.z())) ;
    cv::Point2d p2 = cam.project3dToPixel(cv::Point3d(c2.x(), c2.y(), c2.z())) ;
    cv::Point2d p3 = cam.project3dToPixel(cv::Point3d(c3.x(), c3.y(), c3.z())) ;
    cv::Point2d p4 = cam.project3dToPixel(cv::Point3d(c4.x(), c4.y(), c4.z())) ;

    mask = maskFromCorners(cam.fullResolution(), p1, p2, p3, p4) ;
}


void depthToCloud(const cv::Mat &depth, const cv::Mat &mask, const PinholeCamera &model_, pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &frame_id)
{
    cloud.width = depth.cols ;
    cloud.height = depth.rows ;
    cloud.is_dense = false ;
    cloud.points.resize(cloud.height * cloud.width) ;
    cloud.header.frame_id = frame_id;

    float center_x = model_.cx();
    float center_y = model_.cy();

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud.begin();

    double unit_scaling = 0.001 ;
    float constant_x = unit_scaling / model_.fx();
    float constant_y = unit_scaling / model_.fy();

    cv::Mat_<ushort> depth_(depth) ;
    cv::Mat_<uchar> src_mask(mask) ;

    for(int i=0 ; i<depth.rows ; i++)
        for(int j=0 ; j<depth.cols ; j++)
        {

            if ( src_mask[i][j] == 0 ) continue ;

            pcl::PointXYZ & pt = *pt_iter++;
            ushort val = depth_[i][j] ;

            if ( val == 0 ) {
                pt.x = pt.y = pt.z = bad_point;
                continue;
            }

            pt.x = (j - center_x) * val * constant_x;
            pt.y = (i - center_y) * val * constant_y;
            pt.z = val * unit_scaling ;
        }
}

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.005f, 0.005f, 0.005f);
    sor.filter(*output);
}


bool PointCloudClass::estimatePointCloud()
{
    image_geometry::PinholeCameraModel cam ;
    ros::Time ts;

    cv::Mat rgb, depth, mask;

    pcl::PointCloud<pcl::PointXYZI> cloud;

    if ( !grabber_.grab(rgb, depth, ts, cam) ) return false ;

    cv::imwrite("/tmp/rgd_xtion.png", rgb);
    cv::imwrite("/tmp/depth_xtion.png", depth);

    getPlane(cam, mask);

    PinholeCamera camera(525, 525, 640/2.0, 480/2, cv::Size(640, 480)) ;
    depthToPointCloud(camera, depth, mask, cloud);

//        pcl::io::savePCDFile("/tmp/plane.pcd", cloud);
    cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>() );
    pcl::copyPointCloud(cloud, *cloud_ptr);

    return true;
}

void PointCloudClass::pubPointCloud()
{
    pub_point_cloud.publish(*cloud_ptr);
}

void PointCloudClass::pubPoi()
{

    geometry_msgs::Point poi;

    Vector3d p = getTransform("base_link", "tray_bottom").translation();

    poi.x = p.x();
    poi.y = p.y();
    poi.z = p.z() + height_offset;

    pub_point.publish(poi);
}

void PointCloudClass::depthToPointCloud(const PinholeCamera &model_, const cv::Mat &depth, const cv::Mat &mask, pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    cloud.width = depth.cols ;
    cloud.height = depth.rows ;
    cloud.is_dense = false ;
    cloud.points.resize(cloud.height * cloud.width) ;
    cloud.header.frame_id = "xtion2_rgb_optical_frame";

    float center_x = model_.cx();
    float center_y = model_.cy();

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZI>::iterator pt_iter = cloud.begin();

    double unit_scaling = 0.001 ;
    float constant_x = unit_scaling / model_.fx();
    float constant_y = unit_scaling / model_.fy();

    cv::Mat_<ushort> depth_(depth) ;
    cv::Mat_<uchar> src_mask(mask) ;

    for(int i=0 ; i<depth.rows ; i++)
        for(int j=0 ; j<depth.cols ; j++)
        {
            if ( src_mask[i][j] == 0 ) continue ;

            pcl::PointXYZI & pt = *pt_iter++;
            ushort val = depth_[i][j] ;

            if ( val == 0 ) {
                pt.x = pt.y = pt.z = bad_point;
                continue;
            }

            pt.x = (j - center_x) * val * constant_x;
            pt.y = (i - center_y) * val * constant_y;
            pt.z = val *  unit_scaling;
            pt.intensity = depth.at<unsigned short>(i, j);

//                cout << "x = " << pt.x << ", y = " << pt.y << ", z = " << pt.z << endl;
        }
}

bool PointCloudClass::calcPointCloud(gpd_interface::getPointCloud::Request &req, gpd_interface::getPointCloud::Response & res)
{
//    if(!grabber_.connect()) return false;

    if (!estimatePointCloud())
    {
        cout << "Cannot estimate the Point Cloud" << endl;
        res.point_cloud_estimated = false;
//        grabber_.disconnect();
        return false;
    }
    else
    {
        res.point_cloud_estimated = true;
//        grabber_.disconnect();
        return true;
    }
}



