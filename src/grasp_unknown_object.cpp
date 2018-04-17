#include <gpd_interface/grasp_unknown_object.hpp>

#include <radioroso_gripper_control/GripperClose.h>

void closeGripperServiceCall()
{
    ros::NodeHandle n;
    ros::ServiceClient close_gripper_client = n.serviceClient<radioroso_gripper_control::GripperClose>("/radioroso_gripper_control/GripperClose");

    radioroso_gripper_control::GripperClose srv;

    //srv.request.xd = 1100;

    std::cout << "Call service" << std::endl;

    if(close_gripper_client.call(srv))
        std::cout << srv.response.resp << std::endl;
    else
        std::cout << "service not called" << std::endl;
}


void GraspUnknownObject::graspListCallback(const gpd::GraspConfigList &msg)
{
//    Affine3d rgb_to_base = getTransform("base_link", "xtion2_rgb_optical_frame");
//    Matrix4d xtion_to_base = rgb_to_base.matrix();

    for (int id=0; id<msg.grasps.size(); id++)
    {
        Matrix4d temp_matrix;

        temp_matrix << msg.grasps[id].approach.x, msg.grasps[id].binormal.x, msg.grasps[id].axis.x, msg.grasps[id].bottom.x,
                       msg.grasps[id].approach.y, msg.grasps[id].binormal.y, msg.grasps[id].axis.y, msg.grasps[id].bottom.y,
                       msg.grasps[id].approach.z, msg.grasps[id].binormal.z, msg.grasps[id].axis.z, msg.grasps[id].bottom.z,
                                               0,                         0,                     0,                       1;

//        Matrix4d object_to_base_temp =  xtion_to_base * temp_matrix;
//        object_to_base.push_back(object_to_base_temp);

        object_to_base.push_back(temp_matrix);

        Vector3d temp_approach_vector;
        temp_approach_vector.x() = msg.grasps[id].approach.x;
        temp_approach_vector.y() = msg.grasps[id].approach.y;
        temp_approach_vector.z() = msg.grasps[id].approach.z;
//        temp_approach_vector.x() = object_to_base_temp(0,0);
//        temp_approach_vector.y() = object_to_base_temp(1,0);
//        temp_approach_vector.z() = object_to_base_temp(2,0);
        approach_vector.push_back(temp_approach_vector);
//        cout << "approach" << temp_approach_vector << endl;

        Vector3d temp_binormal;
        temp_binormal.x() = msg.grasps[id].binormal.x;
        temp_binormal.y() = msg.grasps[id].binormal.y;
        temp_binormal.z() = msg.grasps[id].binormal.z;
//        temp_binormal.x() = object_to_base_temp(0,1);
//        temp_binormal.y() = object_to_base_temp(1,1);
//        temp_binormal.z() = object_to_base_temp(2,1);
        binormal.push_back(temp_binormal);

        Vector3d temp_axis;
        temp_axis.x() = msg.grasps[id].axis.x;
        temp_axis.y() = msg.grasps[id].axis.y;
        temp_axis.z() = msg.grasps[id].axis.z;
//        temp_axis.x() = object_to_base_temp(0,2);
//        temp_axis.y() = object_to_base_temp(1,2);
//        temp_axis.z() = object_to_base_temp(2,2);
        axis.push_back(temp_axis);

        Vector4d temp_t(msg.grasps[id].bottom.x, msg.grasps[id].bottom.y, msg.grasps[id].bottom.z, 1);
//        Vector4d temp_t(object_to_base_temp(0,3), object_to_base_temp(1,3), object_to_base_temp(2,3), 1);
        t.push_back(temp_t);

        bottom.push_back(temp_t);

        Vector4d temp_surface(msg.grasps[id].surface.x, msg.grasps[id].surface.y, msg.grasps[id].surface.z, 1);
//        Vector4d temp_t(object_to_base_temp(0,3), object_to_base_temp(1,3), object_to_base_temp(2,3), 1);
        surface.push_back(temp_surface);
    }
}


bool GraspUnknownObject::planTipToObject()
{
    std::string arm_name = "r1";
    RobotArm arm(arm_name);
    arm.openGripper();

    arm.setRobotSpeed(0.3);

    cout << "Move robot?" << endl;
    //cin.get();

    for (uint i = 0; i < object_to_base.size(); i++)
    {
        double angle = estimateAngle(object_to_base[i]);

        Quaterniond q;
        q = robot_helpers::lookAt(Vector3d(approach_vector[i].x(), approach_vector[i].y(), approach_vector[i].z()), angle);
//        cout << "[ " << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << " ]" << endl;

        Vector3d new_position;
        Vector3d approach_offset = direction_offset * approach_vector[i];

        new_position.x() = t[i].x() + approach_offset.x();
        new_position.y() = t[i].y() + approach_offset.y();
        new_position.z() = t[i].z() + approach_offset.z();

        RobotArm::Plan plan;

        //cout << "Hand bottom: x = " << bottom[i].x() << ", y = " << bottom[i].y() << ", z = " << bottom[i].z() << endl;
        //cout << "Surface bottom: x = " << surface[i].x() << ", y = " << surface[i].y() << ", z = " << surface[i].z() << endl;

        //approach unknown object
        if ( !arm.planTipIK(t[i].head<3>(), q, plan) ) {
            cerr << "can't plan to location:" << t[i] << endl;
            continue;
        }
        else {
            if ( arm.execute(plan) ) {
                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }
        ros::Duration(1).sleep();
        cout << "Move down" << endl;
        //cin.get();


        //move to the grasping position
        if ( !arm.planTipIK(new_position, q, plan) ) {
            cerr << "can't plan to location:" << new_position << endl;
            continue;
        }
        else {
            if ( arm.execute(plan) ) {
                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }
        //cout << "Close gripper?" << endl;
        //cin.get();
	//closeGripperServiceCall();
	ros::Duration(2).sleep();
       // arm.closeGripper();

	arm.moveHome();

/*
        //picking up the object
        if ( !arm.planTipIK(Vector3d(new_position.x(), new_position.y(), new_position.z() + 0.5), q, plan) ) {
            cerr << "can't plan to location:" << new_position << endl;
        }
        else {
            if ( arm.execute(plan) ) {
                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
            }
        }
*/

        //moving to drop position
//        Quaterniond q_final = robot_helpers::lookAt(Vector3d(0, 0, -1), 0);
//        if ( !arm.planTipIK(Vector3d(-0.08, -0.84, 1.10), q_final, plan) ) {
//            cerr << "can't plan to location:" << endl;
//        }
//        else {
//            if ( arm.execute(plan) )
//            {
//                cout << "tip at: " << arm.getTipPose().translation().adjoint() <<endl  ;
//            }
//        }
//        ros::Duration(1).sleep();
        //arm.openGripper();

        return true;
    }

    return false;


}

double GraspUnknownObject::estimateAngle(const Matrix4d &mat_)
{
    Matrix3d temp_rotation = mat_.block(0, 0, 2, 2);

    Vector3d euler_angles = temp_rotation.eulerAngles(2, 1, 0);   //ZYX rotation
    std::cout << "Angles :" << endl << euler_angles * 180 / M_PI << std::endl;

    double angle = euler_angles[2];
    if (angle > M_PI/2 && angle < 3*M_PI/2)
        angle = angle - M_PI;
    else if (angle < -M_PI/2 && angle > -3*M_PI/2)
        angle = angle + M_PI;

    std::cout << "Angle = " << angle * 180 / M_PI << std::endl;
    return angle;
}

bool GraspUnknownObject::isMessageEmpty()
{
    return object_to_base.empty();
}

void GraspUnknownObject::clearVectors()
{
    object_to_base.clear();
    approach_vector.clear();
    binormal.clear();
    axis.clear();
    t.clear();
}


