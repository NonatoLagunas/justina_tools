/**
 * @class FaceRecognitionTasks
 * @brief Containt methods to perform tasks related with face recognition.
 *
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_FACERECOGNITIONTASKS_H_
#define _JUSTINA_FACERECOGNITIONTASKS_H_
#include <string>
#include "ros/ros.h"
class FaceRecognitionTasks
{
    private:

        ros::NodeHandle *m_nh;

    public:
        
        /**
         * @brief Class constructor
         * 
         * Creates a new FaceRecognitionTasks object 
         *
         * @param nh The ROS Node Handler of the calling node. If 
         * no node handler is provided then a new one will be created.
         */
        FaceRecognitionTasks(ros::NodeHandle *t_nh = 0);

};
#endif
