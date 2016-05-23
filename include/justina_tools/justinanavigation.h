#ifndef _JUSTINANAVIGATION_H_
#define _JUSTINANAVIGATION_H_
#include <iostream>
#include <cmath>
#include "justina_tools/navigationtasks.h"
#include "justina_tools/navigationstatus.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "bbros_bridge/RecognizedSpeech.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "navig_msgs/PathFromMap.h"
#include "navig_msgs/PathFromAll.h"
#include "point_cloud_manager/GetRgbd.h"
#include "tf/transform_listener.h"

class JustinaNavigation
{
private:
    static bool is_node_set;

    //Variables for navigation
    static float currentRobotX;
    static float currentRobotY;
    static float currentRobotTheta;
    static nav_msgs::Path lastCalcPath;
    static bool _isGoalReached;
    static NavigationStatus m_navStatus;
    static NavigationTasks m_navTasks;

public:
    //
    //The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
    //The others, block until a goal-reached signal is received
    //
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    static bool isGoalReached();
    static bool waitForGoalReached(int timeOut_ms);
    static void getRobotPose(float& currentX, float& currentY, float& currentTheta);
    //These methods use the simple_move node
    static void startMoveDist(float distance);
    static void startMoveDistAngle(float distance, float angle);
    static void startMovePath(nav_msgs::Path& path);
    static void startGoToPose(float x, float y, float angle);
    static void startGoToRelPose(float relX, float relY, float relTheta);
    static bool moveDist(float distance, int timeOut_ms);
    static bool moveDistAngle(float distance, float angle, int timeOut_ms);
    static bool movePath(nav_msgs::Path& path, int timeOut_ms);
    static bool goToPose(float x, float y, float angle, int timeOut_ms);
    static bool goToRelPose(float relX, float relY, float relTheta, int timeOut_ms);

    //These methods use the mvn_pln node.
    static void startGetClose(float x, float y);
    static void startGetClose(float x, float y, float angle);
    static void startGetClose(std::string location);
    static bool getClose(float x, float y, int timeOut_ms);
    static bool getClose(float x, float y, float angle, int timeOut_ms);
    static bool getClose(std::string location, int timeOut_ms);

    //This functions call services, so, they block until a response is received. They use the path_calculator node
    static bool getOccupancyGrid(nav_msgs::OccupancyGrid& map);
    static bool calcPathFromMapAStar(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromMapAStar(float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromMapWaveFront(float startX, float startY, float goalX, float goalY, nav_msgs::Path& result);
    static bool calcPathFromMapWaveFront(float goalX, float goalY, nav_msgs::Path& result);
    static bool planPath(float startX, float startY, float goalX, float goalY, nav_msgs::Path& path);
    static bool planPath(float goalX, float goalY, nav_msgs::Path& path);
    static bool planPath(std::string start_location, std::string goal_location, nav_msgs::Path& path);
    static bool planPath(std::string goal_location, nav_msgs::Path& path);
    static bool planPath(std::string start_location, float goalX, float goalY, nav_msgs::Path& path);
    static bool planPath(float startX, float startY, std::string goal_location, nav_msgs::Path& path);
};
#endif
