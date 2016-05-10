#include "justina_tools/headstatus.h"

HeadStatus::HeadStatus(ros::NodeHandle *nh, std::string headPoseTopic):
    m_headPoseTopic(headPoseTopic)
{
    /**
     * Subscribe to the recognized speech ros topic if the communication with 
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
        m_subHeadCurrentPose = nh->subscribe(m_headPoseTopic, 100,
                &HeadStatus::headPoseCallback, this);

        m_headPosePublisher = nh->advertise<std_msgs::Float32MultiArray>(
                m_headPoseTopic, 100);

        /*
         * TODO: Print error messages if one of the object could not be 
         * initialized.
         */
        if(m_headPosePublisher)
        {
            m_isInitialized = true; 
        }
        if(m_subHeadCurrentPose)
        {
            m_isInitialized = true; 
        }
    }
}

void HeadStatus::initRosConnection(ros::NodeHandle *nh)
{
    /**
     * Subscribe to the recognized speech ros topic if the communication with 
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
         * Initialize uninitializes subscribers, publishers and verification.
         */
        if(!m_subHeadCurrentPose)
        {
            m_subHeadCurrentPose = nh->subscribe(m_headPoseTopic, 100,
                    &HeadStatus::headPoseCallback, this);
        }
        if(!m_headPosePublisher)
        {
            m_headPosePublisher = nh->advertise<std_msgs::Float32MultiArray>(
                    m_headPoseTopic, 100);
        }

        /*
         * TODO: Print error messages if one of the object could not be 
         * initialized.
         */
        if(m_subHeadCurrentPose)
        {
            m_isInitialized = true; 
        }
        if(m_headPosePublisher)
        {
            m_isInitialized = true; 
        }

        return;
    }
    /*
     * TODO: Print error message if ros is not initialized.
     */
}

void HeadStatus::headPoseCallback(const std_msgs::Float32MultiArray::ConstPtr
        &poseMsg)
{
    m_headPan = poseMsg->data[0];
    m_headTilt = poseMsg->data[1];
}

float HeadStatus::getHeadPan()
{
    return m_headPan;
}

float HeadStatus::getHeadTilt()
{
    return m_headTilt;
}

void HeadStatus::getHeadPose(float &headPan, float &headTilt)
{
    headPan = m_headPan;
    headTilt = m_headTilt;
}

void HeadStatus::setHeadPose(float headPan, float headTilt)
{
    std_msgs::Float32MultiArray headPoseMsg;
    headPoseMsg.data.push_back(headPan);
    headPoseMsg.data.push_back(headTilt);
    m_headPosePublisher.publish(headPoseMsg);

}
