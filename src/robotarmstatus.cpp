#include "justina_tools/robotarmstatus.h"

RobotArmStatus::RobotArmStatus(ros::NodeHandle *nh, int armDOF,
        std::string armCurrentPoseTopic, std::string armCurrentGripperTopic,
        std::string armGoalGripperTopic, std::string armGoalPoseTopic, 
        std::string armGripperTorqueTopic, std::string armGoalTorqueTopic,
        std::string armCurrentBatteryTopic) : 
    m_armDOF(armDOF),
    m_armCurrentPoseTopic(armCurrentPoseTopic),
    m_armCurrentGripperTopic(armCurrentGripperTopic),
    m_armGoalGripperTopic(armGoalGripperTopic),
    m_armGoalPoseTopic(armGoalPoseTopic),
    m_armGripperTorqueTopic(armGripperTorqueTopic),
    m_armGoalTorqueTopic(armGoalTorqueTopic),
    m_armCurrentBatteryTopic(armCurrentBatteryTopic)
{
    
    m_armCurrentGripper = 0;
    m_armBatteryLevel = 0;
    m_armBatteryPerc = 0;
    m_armCurrentPose = std::vector<float>(m_armDOF,0.0);
    /**
     * Subscribe to the arm ros topics if the communication with 
     * ROS is initialized.
     */
    m_isInitialized = false; 
    if(ros::isInitialized())
    {
        /**
         * If no node handler provided, then create a new one.
         */
        if(nh == 0)
        {
            nh = new ros::NodeHandle;
        }

        /*
         * Initialize subscribers, publishers and verification.
         */
        prepareRosConnection(nh);
    }
    /*
     * TODO: Print error message if ros is not initialized.
     */
}

void RobotArmStatus::setArmGoalTorque(std::vector<float> &goalTorques)
{
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    if(m_pubArmGoalTorque)
    {
        std_msgs::Float32MultiArray goalTorquesMsg;
        goalTorquesMsg.data = goalTorques;
        m_pubArmGoalTorque.publish(goalTorquesMsg);
    }
    else {}
}

void RobotArmStatus::setTorqueGrip(float torqueGrip)
{
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    if(m_pubArmTorqueGrip)
    {
        std_msgs::Float32 torqueGripMsg;
        torqueGripMsg.data = torqueGrip;
        m_pubArmTorqueGrip.publish(torqueGripMsg);
    }
    else {}
}

void RobotArmStatus::setGoalGripper(float goalGripper)
{
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    if(m_pubArmGoalGripper)
    {
        std_msgs::Float32 goalGripperMsg;
        goalGripperMsg.data = goalGripper;
        m_pubArmGoalGripper.publish(goalGripperMsg);
    }
    else {}
}

void RobotArmStatus::setGoalPose(std::vector<float> &goalAngles)
{
    /**
     * TODO: Print error message if the publisher is not valid.
     */
    if(m_pubArmGoalPose)
    {
        std_msgs::Float32MultiArray goalPoseMsg;
        goalPoseMsg.data = goalAngles;
        m_pubArmGoalPose.publish(goalPoseMsg);
    }
    else {}
}

void RobotArmStatus::initRosConnection(ros::NodeHandle *nh)
{
    /**
     * Subscribe to the arm ros topics if the communication with 
     * ROS is initialized.
     */
    if(ros::isInitialized())
    {
        /**
         * If no node handler provided, then create a new one.
         */
        if(nh == 0)
        {
            nh = new ros::NodeHandle;
        }
        /*
         * Initialize subscribers, publishers and verification.
         */
        prepareRosConnection(nh);
        return;
    }
    /*
     * TODO: Print error message if ros is not initialized.
     */
}

void RobotArmStatus::armCurrentPoseCallback(
        const std_msgs::Float32MultiArray::ConstPtr& floatArrayMsg)
{
    if(floatArrayMsg->data.size() != m_armDOF)
    {
        //std::cout << "JustinaHardware.->Error in callback for left arm 
        //current pose: msg must have 7 values" << std::endl;
        return;
    }
    m_armCurrentPose = std::vector<float>(m_armDOF,0.0);
    for(int i=0; i<m_armDOF; i++)
    {
        m_armCurrentPose[i] = floatArrayMsg->data[i];
    }
}

void RobotArmStatus::armCurrentGripperCallback(
        const std_msgs::Float32::ConstPtr& floatMsg)
{
    m_armCurrentGripper = floatMsg->data;
}

void RobotArmStatus::armCurrentBatteryCallback(
        const std_msgs::Float32::ConstPtr& floatMsg)
{
    m_armBatteryLevel = floatMsg->data;
    m_armBatteryPerc = (int)((m_armBatteryLevel - 10.725)/(12.6 - 10.725)*100);
}

float RobotArmStatus::getArmCurrentGripper()
{
    return m_armCurrentGripper;
}

std::vector<float> RobotArmStatus::getArmCurrentPose()
{
    return m_armCurrentPose;
}

float RobotArmStatus::getArmBatteryLevel()
{
    return m_armBatteryLevel; 
}

int RobotArmStatus::getArmBatteryPerc()
{
    return m_armBatteryPerc; 
}

void RobotArmStatus::prepareRosConnection(ros::NodeHandle *nh)
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_subArmCurrentGripper)
    {
        if((m_subArmCurrentGripper=nh->subscribe(m_armCurrentGripperTopic, 100, 
                        &RobotArmStatus::armCurrentGripperCallback, this)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_subArmCurrentPose)
    {
        if((m_subArmCurrentPose= nh->subscribe(m_armCurrentPoseTopic, 100, 
                &RobotArmStatus::armCurrentPoseCallback, this)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_subArmCurrentBattery)
    {
        if((m_subArmCurrentBattery=nh->subscribe(m_armCurrentBatteryTopic, 100, 
                &RobotArmStatus::armCurrentBatteryCallback, this)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_pubArmGoalGripper)
    {
        if((m_pubArmGoalGripper = nh->advertise<std_msgs::Float32>(
                m_armGoalGripperTopic, 100)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_pubArmGoalPose)
    {
        if((m_pubArmGoalPose = nh->advertise<std_msgs::Float32MultiArray>(
                m_armGoalPoseTopic, 100)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_pubArmTorqueGrip)
    {
        if((m_pubArmTorqueGrip = nh->advertise<std_msgs::Float32>(
                m_armGripperTorqueTopic, 100)))
        {
            m_isInitialized = true; 
        } else {}
    }
    if(!m_pubArmGoalTorque)
    {
        if((m_pubArmGoalTorque = nh->advertise<std_msgs::Float32MultiArray>(
                m_armGoalTorqueTopic, 100)))
        {
            m_isInitialized = true; 
        } else {}
    }
}
