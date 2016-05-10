#include "justina_tools/justinahri.h"

SpeechGeneratorTasks JustinaHRI::m_spgen;
SpeechRecognitionStatus JustinaHRI::m_sprec;
bool JustinaHRI::m_isNodeSet = false; 

bool JustinaHRI::setNodeHandle(ros::NodeHandle* nodeHandler)
{
    /**
     * Verify if the communication with ROS was previously initialize.
     */
    if(JustinaHRI::m_isNodeSet)
        return true;
    if(nodeHandler == 0)
        return false;

    m_sprec.initRosConnection(nodeHandler);
    m_isNodeSet = true;
    return true;
}


bool JustinaHRI::waitForSpokenSentence(std::string& recognizedSentence, 
        int timeout)
{
    /**
     * Not implemented yed. I think this task must be implemented in the simple
     * task planner.
     */
    return false;
}

void JustinaHRI::startSay(std::string textToSpeech)
{
    m_spgen.asyncSpeech(textToSpeech);
}

bool JustinaHRI::say(std::string textToSpeech, int timeout)
{
    return m_spgen.syncSpeech(textToSpeech, timeout);
}

void JustinaHRI::startFollowHuman()
{
    /**
     * Not implenented yet.
     */
}

void JustinaHRI::stopFollowHuman()
{
    /**
     * Not implenented yet.
     */
}
