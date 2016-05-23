/**
 * @class JustinaManip
 * @brief Contains static methods to perform robot manipulation actions.
 *
 * This library is based on the original JustinaManip library of the 
 * justina_tools package created by Marco Negrete and it is implemented in this
 * package in order to allow compatibility with users of the original 
 * justina_tools package.
 *
 * @author Marco Negrete (the_magnificent)
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/

#ifndef _JUSTINAMANIP_H_
#define _JUSTINAMANIP_H_
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/InverseKinematicsPath.h"
#include "manip_msgs/InverseKinematicsPose.h"
#include "manip_msgs/DirectKinematics.h"
#include "justina_tools/transformations.h"

class JustinaManip
{
private:
    static Transformations m_transf;
    static bool is_node_set;

public:
    static bool setNodeHandle(ros::NodeHandle* nh);

    static bool inverseKinematics(std::vector<float>& cartesian, std::vector<float>& articular);
    static bool inverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::vector<float>& articular);
    static bool inverseKinematics(float x, float y, float z, std::vector<float>& articular);
    static bool inverseKinematics(std::vector<float>& cartesian, std::string frame_id, std::vector<float>& articular);
    static bool inverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::string frame_id, std::vector<float>& articular);
    static bool inverseKinematics(float x, float y, float z, std::string frame_id, std::vector<float>& articular);
    //static bool inverseKinematics(geometry_msgs::Pose& cartesian, std::vector<float>& articular);
    //static bool inverseKinematics(nav_msgs::Path& cartesianPath, std::vector<std::vector<float> >& articularPath);
    //static bool inverseKinematics(nav_msgs::Path& cartesianPath, std::vector<Float32MultiArray>& articularPath);
    static bool directKinematics(std::vector<float>& cartesian, std::vector<float>& articular);
};
#endif
