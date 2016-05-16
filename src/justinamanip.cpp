#include "justina_tools/JustinaManip.h"

bool JustinaManip::is_node_set = false;
Transformations JustinaManip::m_transf;

bool JustinaManip::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaManip::is_node_set)
        return true;
    if(nh == 0)
        return false;

    m_transf.initRosConnection(nh);
    JustinaManip::is_node_set = true;

    return true;
}

bool JustinaManip::inverseKinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
    std::cout << "JustinaManip.->Calling service for inverse kinematics..." << std::endl;
    return m_transf.inverseKinematics(cartesian, articular);
}

bool JustinaManip::inverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::vector<float>& articular)
{
    return false;
}

bool JustinaManip::inverseKinematics(float x, float y, float z, std::vector<float>& articular)
{
    return false;
}

bool JustinaManip::inverseKinematics(std::vector<float>& cartesian, std::string frame_id, std::vector<float>& articular)
{
    return false;
}

bool JustinaManip::inverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::string frame_id, std::vector<float>& articular)
{
    return false;
}

bool JustinaManip::inverseKinematics(float x, float y, float z, std::string frame_id, std::vector<float>& articular)
{
    return false;
}

// bool JustinaManip::inverseKinematics(geometry_msgs::Pose& cartesian, std::vector<float>& articular);
// bool JustinaManip::inverseKinematics(nav_msgs::Path& cartesianPath, std::vector<std::vector<float> >& articularPath);
// bool JustinaManip::inverseKinematics(nav_msgs::Path& cartesianPath, std::vector<Float32MultiArray>& articularPath);
bool JustinaManip::directKinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
    std::cout << "JustinaManip.->Calling service for direct kinematics..." << std::endl;
    return m_transf.directKinematics(cartesian, articular);
}
