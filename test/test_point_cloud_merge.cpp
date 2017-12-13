#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <robot_helpers/util.hpp>

using namespace robot_helpers;
using namespace Eigen;
using namespace std;

//void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
//{
//    pcl::VoxelGrid<pcl::PointXYZ> sor;
//    sor.setInputCloud(cloud);
//    sor.setLeafSize(0.005f, 0.005f, 0.005f);
//    sor.filter(*output);
//}

void test1()
{
    std::vector< int > index;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output2(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::PointXYZ>::Ptr pc1(new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::io::loadPCDFile("/home/mitre/pc0.pcd", *output1);
    pcl::removeNaNFromPointCloud(*output1, *output1, index);
//    downsample(pc1, output1);
//    pcl::io::savePCDFile("/tmp/pc1_downsampled.pcd", *output1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2(new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::io::loadPCDFile("/home/mitre/pc1.pcd", *output2);
    pcl::removeNaNFromPointCloud(*output2, *output2, index);
//    downsample(pc2, output2);
//    pcl::io::savePCDFile("/tmp/pc2_downsampled.pcd", *output2);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(0.001);
    icp.setInputTarget(output1);
    icp.setInputSource(output2);

    icp.setMaximumIterations(5);
    icp.align(*output2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>() );

    *final = *output1 + *output2;

    pcl::io::savePCDFile("/home/mitre/pc1_2.pcd", *final);
}

void test2()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/tmp/point_cloud/point_cloud5/final_pc.pcd", *cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

      int i = 0, nr_points = (int) cloud_filtered->points.size ();
      // While 30% of the original cloud is still there
      while (cloud_filtered->points.size () > 0.3 * nr_points)
      {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        std::stringstream ss;
        ss << "/tmp/final_pc" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
      }

      pcl::io::savePCDFile("/tmp/final_pc_f.pcd", *cloud_f);
}

void test3()
{
    double xmin = -0.2;
    double xmax = 0.7;
    double ymax = -0.6;
    double ymin = -1.4;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width  = (int)abs((xmax-xmin)/0.001) + 1;
    cloud->height = (int)abs((ymax-ymin)/0.001) + 1;
    cloud->points.resize (cloud->width * cloud->height);

    uint i = 0;
    for (double x = xmin; x < xmax; x+=0.005 )
    {
        for (double y = ymin; y < ymax; y+=0.005 )
        {
            cloud->points[i].x = x;
            cloud->points[i].y = y;
            cloud->points[i].z = 0.77;
            i++;
        }
    }

    pcl::io::savePCDFile("/tmp/tray_plane.pcd", *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/tmp/point_cloud/point_cloud6/final_pc.pcd", *cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);

    *cloud_final = *cloud + *cloud_filtered;

    pcl::io::savePCDFile("/tmp/tray_plane_with_object.pcd", *cloud_final);
}


int main(int argc, char *argv[])
{
    test2();

    return 0;
}
