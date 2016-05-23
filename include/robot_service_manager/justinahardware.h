/**
 * @class JustinaHardware
 * @brief Contains static methods to perform robot hardware actions.
 *
 * This library is based on the original JustinaHardware library of the 
 * justina_tools package created by Marco Negrete and it is implemented in this
 * package in order to allow compatibility with users of the original 
 * justina_tools package.
 *
 * @author Marco Negrete (the_magnificent)
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINAHARDWARE_H_
#define _JUSTINAHARDWARE_H_
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "justina_tools/headstatus.h"
#include "justina_tools/mobilebasestatus.h"
#include "justina_tools/robotarmstatus.h"
#include "justina_tools/robotstatus.h"
#include "justina_tools/sensorstasks.h"


class JustinaHardware
{
private:
    static HeadStatus m_headStatus;
    static MobileBaseStatus m_mobileBase;
    static RobotArmStatus m_rightArmStatus;
    static RobotArmStatus m_leftArmStatus;
    static RobotStatus m_robotStatus;
    static SensorsTasks m_sensorsTasks;

    static bool is_node_set;
    //
    //Variables for head position
    static float headPan;
    static float headTilt;
    //Variables for arms
    static float leftArmCurrentGripper;
    static float rightArmCurrentGripper;
    static std::vector<float> leftArmCurrentPose;
    static std::vector<float> rightArmCurrentPose;
    //Variables for robot state;
    static float _baseBattery;
    static float _leftArmBattery;
    static float _rightArmBattery;
    static float _headBattery;
    static int _baseBatteryPerc;
    static int _leftArmBatteryPerc;
    static int _rightArmBatteryPerc;
    static int _headBatteryPerc;

public:
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for operating head
    static void getHeadCurrentPose(float& pan, float& tilt);
    static float getHeadCurrentPan();
    static float getHeadCurrentTilt();
    static void setHeadGoalPose(float pan, float tilt);
    //Methods for operating the left arm
    static float getLeftArmCurrentGripper();
    static void getLeftArmCurrentPose(std::vector<float>& currentPose);
    static void setLeftArmGoalGripper(float goalGripper);
    static void setLeftArmGoalPose(std::vector<float>& goalAngles);
    static void setLeftArmGoalPose(float theta0, float theta1, float theta2, 
            float theta3, float theta4, float theta5, float theta6);
    static void setLeftArmGoalTorqueGrip(float torqueGripper);
    static void setLeftArmGoalTorque(std::vector<float>& goalTorques);
    static void setLeftArmGoalTorque(float t0, float t1, float t2, float t3, 
            float t4, float t5, float t6);
    //Methods for operating right arm
    static float getRightArmCurrentGripper();
    static void getRightArmCurrentPose(std::vector<float>& currentPose);
    static void setRightArmGoalGripper(float goalGripper);
    static void setRightArmGoalPose(std::vector<float>& goalAngles);
    static void setRightArmGoalPose(float theta0, float theta1, float theta2, 
            float theta3, float theta4, float theta5, float theta6);
    static void setRightArmGoalTorqueGrip(float torqueGripper);
    static void setRightArmGoalTorque(std::vector<float>& goalTorques);
    static void setRightArmGoalTorque(float t0, float t1, float t2, float t3, 
            float t4, float t5, float t6);
    //Methods for operating the mobile base
    static void setBaseSpeeds(float leftSpeed, float rightSpeed);
    static void setBaseCmdVel(float linearX, float linearY, float angular);
    //Methods for operating robot state
    static void stopRobot();
    static float baseBattery();
    static float leftArmBattery();
    static float rightArmBattery();
    static float headBattery();
    static int baseBatteryPerc();
    static int leftArmBatteryPerc();
    static int rightArmBatteryPerc();
    static int headBatteryPerc();
    //Methods for operating point_cloud_manager
    static bool getRgbdWrtKinect(sensor_msgs::PointCloud2& cloud);
    static bool getRgbdWrtRobot(sensor_msgs::PointCloud2& cloud);
    static void startSavingCloud(std::string fileName);
    static void stopSavingCloud();
};
#endif
