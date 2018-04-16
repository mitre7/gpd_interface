#ifndef GRASP_UNKNOWN_OBJECT_HPP
#define GRASP_UNKNOWN_OBJECT_HPP

#include <robot_helpers/robot.hpp>
#include <robot_helpers/geometry.hpp>
#include <robot_helpers/util.hpp>
#include <gpd/GraspConfigList.h>
#include <ros/ros.h>

using namespace robot_helpers;
using namespace Eigen;
using namespace std;

class GraspUnknownObject{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;

    std::vector<Matrix4d> object_to_base;
    std::vector<Vector3d> approach_vector;
    std::vector<Vector3d> binormal;
    std::vector<Vector3d> axis;
    std::vector<Vector4d> t;
    std::vector<Vector4d> surface;
    std::vector<Vector4d> bottom;

    float direction_offset;

public:
    GraspUnknownObject():
    direction_offset(0.025)
    {
        clearVectors();
        sub = nh.subscribe("/detect_grasps/clustered_grasps", 1, &GraspUnknownObject::graspListCallback, this);
    }

    void graspListCallback(const gpd::GraspConfigList &msg);
    bool planTipToObject();

    double estimateAngle(const Matrix4d &mat_);

    void clearVectors();
    bool isMessageEmpty();
};

#endif // GRASP_UNKNOWN_OBJECT_HPP
