/**
 * @class JustinaHRI
 * @brief Contains static methods to perform human robot interaction actions.
 *
 * This library is based on the original JustinaHRI library of the 
 * justina_tools package created by Marco Negrete and it is implemented in this
 * package in order to allow compatibility with users of the original 
 * justina_tools package.
 * Examples of actions classes included (but no limited to) in this class 
 * are those like:
 *  - Follow a human.
 *  - Recognize a voice sentence.
 *  - Speech a voice sentence.
 *
 * @author R. Nonato Lagunas (nonato)
 * @author Marco Negrete
 * @version 0.1
*/
#ifndef _JUSTINA_HRI_H_
#define _JUSTINA_HRI_H_
#include <string>
#include "ros/ros.h"
#include "justina_tools/speechrecognitionstatus.h"
#include "justina_tools/speechgeneratortasks.h"
class JustinaHRI
{
    private:

        static SpeechGeneratorTasks m_spgen; /**< To perform the robot speech
                                               generation tasks (say, asay, 
                                               etc). */

        static SpeechRecognitionStatus m_sprec; /** To perform the robot speech
                                                  recognition tasks (wait for
                                                  reco sentence, etc). */

        static bool m_isNodeSet; /**< Flag that indicates if the class is 
                                   already subscribed to the corresponding 
                                   ROS topics and advertising the corresponding
                                   ROS services. */

    public:

        /**
         * @brief Initiates the communication of this class with the ROS core.
         * 
         * @param[in,out] nodeHandler A node handler of the calling node. 
         * @return bool True if the communication was estabilished succesfully 
         * or if the communication is currently estabilished. False otherwise.
         */
        static bool setNodeHandle(ros::NodeHandle* nodeHandler);

        /**
         * @brief Blocks the thread process until the speech recognition node
         * recognize a sentence from the user.
         *
         * @param recognizedSentence[out] The recognized sentence.
         * @param timeout The max time (milliseconds) to wait for a sentence 
         * to be recognized.
         * @return bool True if a senence was recognized during the allowed
         * time. False otherwise.
         */
        static bool waitForSpokenSentence(std::string& recognizedSentence, 
                int timeout);

        /**
        static void fakeSpokenSentence(std::string sentence);
        */

        /**
         * @brief Performs an asynchronous speech generation tasks. It does
         * not block the calling thread.
         *
         * @param textToSpeech The message to be played by the robot.
         * @return void
         */
        static void startSay(std::string textToSpeech);

        /**
         * @brief Performs a synchronous speech generation task. It does block
         * the calling thread.
         *
         * @param textToSpeech The message to be played by the robot.
         * @param timeout The max time (milliseconds) to wait for the speech 
         * task to be completed.
         * @return bool True if the task was performed during the allowed time. 
         * False otherwise.
         */
        static bool say(std::string textToSpeech, int timeout);

        /*
         * @brief Enable the follow human mode on the robot. 
         * (Not implemente yet).
         */
        static void startFollowHuman();

        /*
         * @brief Disable the follow human mode on the robot. 
         * (Not implemente yet).
         */
        static void stopFollowHuman();
};
#endif
