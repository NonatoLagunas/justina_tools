#include "justina_tools/JustinaNavigation.h"

bool JustinaNavigation::is_node_set = false;
//
//Variables for navigation
float JustinaNavigation::currentRobotX = 0;
float JustinaNavigation::currentRobotY = 0;
float JustinaNavigation::currentRobotTheta = 0;
nav_msgs::Path JustinaNavigation::lastCalcPath;
bool JustinaNavigation::_isGoalReached = 0;
NavigationTasks JustinaNavigation::m_navTasks;
NavigationStatus JustinaNavigation::m_navStatus;

//
//The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
//The others, block until a goal-reached signal is received
//

bool JustinaNavigation::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaNavigation::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaNavigation.->Setting ros node..." << std::endl;
    m_navTasks.initRosConnection(nh);
    m_navStatus.initRosConnection(nh);
    
    is_node_set = true;
    return true;
}

bool JustinaNavigation::isGoalReached()
{
    return m_navStatus.isGoalReached();
}

bool JustinaNavigation::waitForGoalReached(int timeOut_ms)
{
    return m_navTasks.waitForGoalReached(timeOut_ms);
}

void JustinaNavigation::getRobotPose(float& currentX, float& currentY, float& currentTheta)
{
    m_navStatus.getCurrentPose(currentX, currentY, currentTheta);
}

//These methods use the simple_move node
void JustinaNavigation::startMoveDist(float distance)
{
    m_navStatus.setGoalDist(distance);
}

void JustinaNavigation::startMoveDistAngle(float distance, float angle)
{
    m_navStatus.setGoalDistAngle(distance, angle);
}

void JustinaNavigation::startMovePath(nav_msgs::Path& path)
{
    std::cout << "JustinaNavigation.->Publishing goal path.." << std::endl;
    m_navStatus.setGoalPath(path);
}

void JustinaNavigation::startGoToPose(float x, float y, float angle)
{
    m_navStatus.setGoalPose(x, y, angle);
}

void JustinaNavigation::startGoToRelPose(float relX, float relY, float relTheta)
{
    m_navStatus.setGoalRelPose(relX, relY, relTheta);
}

bool JustinaNavigation::moveDist(float distance, int timeOut_ms)
{
    return m_navTasks.syncMove(distance, timeOut_ms);
}

bool JustinaNavigation::moveDistAngle(float distance, float angle, int timeOut_ms)
{
    return m_navTasks.syncMove(distance, angle, timeOut_ms);
}

bool JustinaNavigation::movePath(nav_msgs::Path& path, int timeOut_ms)
{
    return m_navTasks.syncMove(path, timeOut_ms);
}

bool JustinaNavigation::goToPose(float x, float y, float angle, int timeOut_ms)
{
    return m_navTasks.syncGoToPose(x, y, angle, timeOut_ms);
}

bool JustinaNavigation::goToRelPose(float relX, float relY, float relTheta, int timeOut_ms)
{
    return m_navTasks.syncGoToRelPose(relX, relY, relTheta, timeOut_ms);
}

//These methods use the mvn_pln node.
void JustinaNavigation::startGetClose(float x, float y)
{
    m_navStatus.setGetCloseGoal(x, y);
}

void JustinaNavigation::startGetClose(float x, float y, float angle)
{
    m_navStatus.setGetCloseGoal(x, y, angle);
}

void JustinaNavigation::startGetClose(std::string location)
{
    m_navStatus.setGetCloseGoal(location);
}

bool JustinaNavigation::getClose(float x, float y, int timeOut_ms)
{
    return m_navTasks.syncGetClose(x, y, timeOut_ms);
}

bool JustinaNavigation::getClose(float x, float y, float angle, int timeOut_ms)
{
    return m_navTasks.syncGetClose(x, y, angle, timeOut_ms);
}

bool JustinaNavigation::getClose(std::string location, int timeOut_ms)
{
    return m_navTasks.syncGetClose(location, timeOut_ms);
}

//This functions call services, so, they block until a response is received. They use the path_calculator node
bool JustinaNavigation::getOccupancyGrid(nav_msgs::OccupancyGrid& map)
{
    return m_navTasks.getOccupancyGrid(map);
}

bool JustinaNavigation::calcPathFromMapAStar(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result)
{
    std::cout << "JustinaNavig.->Calculating path from " << startX << " " << startX << " to " << goalX <<" " << goalY;
    std::cout << "by A* using only map"<<std::endl;

    return m_navTasks.calcAStarPathFromMap(startX, startY, goalX, goalY, result);
}

bool JustinaNavigation::calcPathFromMapAStar(float goalX, float goalY, nav_msgs::Path& result)
{
    return m_navTasks.calcAStarPathFromMap(goalX, goalY, result);
}

bool JustinaNavigation::calcPathFromMapWaveFront(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result)
{
    std::cout << "JustinaNavig.->Calculating path from " << startX << " " << startX << " to " << goalX <<" " << goalY;
    std::cout << "by wave front using only map"<<std::endl;

    return m_navTasks.calcWaveFrontPathFromMap(startX, startY, goalX, goalY, result);
}

bool JustinaNavigation::calcPathFromMapWaveFront(float goalX, float goalY, nav_msgs::Path& result)
{
    return m_navTasks.calcWaveFrontPathFromMap(goalX, goalY, result);
}

bool JustinaNavigation::planPath(float startX, float startY, float goalX, float goalY, nav_msgs::Path& path)
{
    return m_navTasks.planPath(startX, startY, goalX, goalY, path);
}

bool JustinaNavigation::planPath(float goalX, float goalY, nav_msgs::Path& path)
{
    return m_navTasks.planPath(goalX, goalY, path);
}

bool JustinaNavigation::planPath(std::string start_location, std::string goal_location, nav_msgs::Path& path)
{
    return m_navTasks.planPath(start_location, goal_location, path);
}

bool JustinaNavigation::planPath(std::string goal_location, nav_msgs::Path& path)
{
    return m_navTasks.planPath(goal_location, path);
}

bool JustinaNavigation::planPath(std::string start_location, float goalX, float goalY, nav_msgs::Path& path)
{
    return m_navTasks.planPath(start_location, goalX, goalY, path);
}

bool JustinaNavigation::planPath(float startX, float startY, std::string goal_location, nav_msgs::Path& path)
{
    return m_navTasks.planPath(startX, startY, goal_location, path);
}
