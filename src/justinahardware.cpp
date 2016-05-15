#include "justina_tools/justinahardware.h"

HeadStatus JustinaHardware::m_headStatus;
MobileBaseStatus JustinaHardware::m_mobileBase;
RobotArmStatus JustinaHardware::m_rightArmStatus;
RobotArmStatus JustinaHardware::m_leftArmStatus(0, 7, 
        "/hardware/left_arm/current_pose",
        "/hardware/left_arm/current_gripper",
        "/hardware/left_arm/goal_gripper",
        "/hardware/left_arm/goal_pose",
        "/hardware/left_arm/torque_gripper",
        "/hardware/left_arm/goal_torque",
        "/hardware/robot_state/left_arm_battery");
RobotStatus JustinaHardware::m_robotStatus;
SensorsTasks JustinaHardware::m_sensorsTasks;

bool JustinaHardware::is_node_set = false;

bool JustinaHardware::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaHardware::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaHardware.->Setting ros node..." << std::endl;

    m_headStatus.initRosConnection(nh);
    m_mobileBase.initRosConnection(nh);
    m_rightArmStatus.initRosConnection(nh);
    m_leftArmStatus.initRosConnection(nh);
    m_robotStatus.initRosConnection(nh);
    m_sensorsTasks.initRosConnection(nh);
    JustinaHardware::is_node_set = true;

    return true;
}

//Methods for operating head
void JustinaHardware::getHeadCurrentPose(float& pan, float& tilt)
{
    m_headStatus.getHeadPose(pan, tilt);
}

float JustinaHardware::getHeadCurrentPan()
{
    return m_headStatus.getHeadPan();
}

float JustinaHardware::getHeadCurrentTilt()
{
    return m_headStatus.getHeadTilt();
}

void JustinaHardware::setHeadGoalPose(float pan, float tilt)
{
    m_headStatus.setHeadPose(pan, tilt);
}

//Methods for operating the left arm
float JustinaHardware::getLeftArmCurrentGripper()
{
    return m_leftArmStatus.getArmCurrentGripper();
}

void JustinaHardware::getLeftArmCurrentPose(std::vector<float>& currentPose)
{
    currentPose = m_leftArmStatus.getArmCurrentPose();
}

void JustinaHardware::setLeftArmGoalGripper(float goalGripper)
{
    m_leftArmStatus.setGoalGripper(goalGripper);
}

void JustinaHardware::setLeftArmGoalPose(std::vector<float>& goalAngles)
{
    m_leftArmStatus.setGoalPose(goalAngles);
}

void JustinaHardware::setLeftArmGoalPose(float theta0, float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{
    std::vector<float> msg;
    msg.push_back(theta0);
    msg.push_back(theta1);
    msg.push_back(theta2);
    msg.push_back(theta3);
    msg.push_back(theta4);
    msg.push_back(theta5);
    msg.push_back(theta6);
    m_leftArmStatus.setGoalPose(msg);
}

void JustinaHardware::setLeftArmGoalTorqueGrip(float torqueGripper)
{
    m_leftArmStatus.setTorqueGrip(torqueGripper);
}

void JustinaHardware::setLeftArmGoalTorque(std::vector<float>& goalTorques)
{
    m_leftArmStatus.setArmGoalTorque(goalTorques);
}

void JustinaHardware::setLeftArmGoalTorque(float t0, float t1, float t2, float t3, float t4, float t5, float t6)
{
    std::vector<float> msg;
    msg.push_back(t0);
    msg.push_back(t1);
    msg.push_back(t2);
    msg.push_back(t3);
    msg.push_back(t4);
    msg.push_back(t5);
    msg.push_back(t6);
    m_leftArmStatus.setArmGoalTorque(msg);
}

//Methods for operating right arm
float JustinaHardware::getRightArmCurrentGripper()
{
    return m_rightArmStatus.getArmCurrentGripper();
}

void JustinaHardware::getRightArmCurrentPose(std::vector<float>& currentPose)
{
    currentPose = m_rightArmStatus.getArmCurrentPose();
}

void JustinaHardware::setRightArmGoalGripper(float goalGripper)
{
    m_rightArmStatus.setGoalGripper(goalGripper);
}

void JustinaHardware::setRightArmGoalPose(std::vector<float>& goalAngles)
{
    m_rightArmStatus.setGoalPose(goalAngles);
}

void JustinaHardware::setRightArmGoalPose(float theta0, float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{
    std::vector<float> msg;
    msg.push_back(theta0);
    msg.push_back(theta1);
    msg.push_back(theta2);
    msg.push_back(theta3);
    msg.push_back(theta4);
    msg.push_back(theta5);
    msg.push_back(theta6);
    m_rightArmStatus.setGoalPose(msg);
}

void JustinaHardware::setRightArmGoalTorqueGrip(float torqueGripper)
{
    m_rightArmStatus.setTorqueGrip(torqueGripper);
}

void JustinaHardware::setRightArmGoalTorque(std::vector<float>& goalTorques)
{
    m_rightArmStatus.setArmGoalTorque(goalTorques);
}

void JustinaHardware::setRightArmGoalTorque(float t0, float t1, float t2, float t3, float t4, float t5, float t6)
{
    std::vector<float> msg;
    msg.push_back(t0);
    msg.push_back(t1);
    msg.push_back(t2);
    msg.push_back(t3);
    msg.push_back(t4);
    msg.push_back(t5);
    msg.push_back(t6);
    m_rightArmStatus.setArmGoalTorque(msg);
}

//Methods for operating the mobile base
void JustinaHardware::setBaseSpeeds(float leftSpeed, float rightSpeed)
{
    std::vector<float> msg;
    msg.push_back(leftSpeed);
    msg.push_back(rightSpeed);
    m_mobileBase.setMobileBaseSpeeds(msg);
}

void JustinaHardware::setBaseCmdVel(float linearX, float linearY, float angular)
{
    m_mobileBase.setMobileBaseCmdVel(linearX, linearY, angular);
}

//Methods for operating robot state
void JustinaHardware::stopRobot()
{
    std::cout << "JustinaHardware.->Sending stop robot... " << std::endl;
    m_robotStatus.sendStopHardwareIndication();
}

float JustinaHardware::baseBattery()
{
    return m_mobileBase.getMobileBaseBattery();
}

float JustinaHardware::leftArmBattery()
{
    return m_leftArmStatus.getArmBatteryLevel();
}

float JustinaHardware::rightArmBattery()
{
    return m_rightArmStatus.getArmBatteryLevel();
}

float JustinaHardware::headBattery()
{
    return m_headStatus.getHeadBattery();
}

int JustinaHardware::baseBatteryPerc()
{
    float b = m_mobileBase.getMobileBaseBattery();
    return (int)((b - 17.875)/(21.0 - 17.875)*100); 
}

int JustinaHardware::leftArmBatteryPerc()
{
    float b = m_leftArmStatus.getArmBatteryLevel();
    return (int)((b - 10.725)/(12.6 - 10.725)*100);
}

int JustinaHardware::rightArmBatteryPerc()
{
    float b = m_rightArmStatus.getArmBatteryLevel();
    return (int)((b - 10.725)/(12.6 - 10.725)*100);
}

int JustinaHardware::headBatteryPerc()
{
    float b = m_headStatus.getHeadBattery();
    return (int)((b - 17.875)/(21.0 - 17.875)*100);
}

//Methods for operating point_cloud_man
bool JustinaHardware::getRgbdWrtKinect(sensor_msgs::PointCloud2& cloud)
{
    if(!m_sensorsTasks.getKinnectRGBD(cloud))
    {
        std::cout << "JustinaHardware.->Cannot get point cloud wrt kinnect" << std::endl;
        return false;
    }
    return true;
}

bool JustinaHardware::getRgbdWrtRobot(sensor_msgs::PointCloud2& cloud)
{
    if(!m_sensorsTasks.getRobotRGBD(cloud))
    {
        std::cout << "JustinaHardware.->Cannot get point cloud wrt robot" << std::endl;
        return false;
    }
    return true;
}

void JustinaHardware::startSavingCloud(std::string fileName)
{
    m_sensorsTasks.startSavingCloud(fileName);
}

void JustinaHardware::stopSavingCloud()
{
    m_sensorsTasks.stopSavingCloud();
}
