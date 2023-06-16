#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <std_msgs/String.h>

#include <geometry_msgs/TwistStamped.h>
#include <pose_controller_lib/pose_controller.h>
#include <pose_controller_lib/controller.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// Includes for actions
#include <uavr_nav_msgs/TakeoffAction.h>
#include <uavr_nav_msgs/FollowPathAction.h>
#include <uavr_nav_msgs/LandAction.h>
#include <actionlib/server/simple_action_server.h>

// Evaluation (ROSBAG)
#include <rosbag/bag.h>

//uavr_nav_msgs::FollowPathFeedback feedbackFollowPath;
//uavr_nav_msgs::FollowPathResult resultFollowPath;

#define STABILIZE 0
#define GUIDED 1
#define ARMED 2
#define TAKING_OFF 3
#define HOVERING 4
#define CONTROL 5
#define LANDING 6
#define END 7
#define PILOT_TAKEOVER 8

#define UI_NOT_AVAILABLE 100
#define UI_GUIDED 101
#define UI_ARM 102
#define UI_TAKEOFF 103
#define UI_CONTROL 104
#define UI_LAND 105
#define UI_RESTART 106
#define UI_HOVERING 200;

#define MODE_UNKNOWN 10000
#define MODE_STABILIZE 10001
#define MODE_GUIDED 10002
#define MODE_LAND 10003

ros::NodeHandle *nh_private;

double KP = 1.0;
double KI = 0.0;
double KD = 0.0;

float DISTANCE_TOLERANCE = 0.15;
float TAKEOFF_ALTITUDE_ERROR_TOLERANCE = 0.1;
const float PI = 3.14159;

float FILTER_CIRCLE_RADIUS = 0.43;
float SAFETY_DISTANCE = 0.6;

double MIN_SPHERE_RADIUS = 0.8; // 0.5, 0.75, 1.0 
double MAX_SPHERE_RADIUS = 3.0; // 2.0, 4.0, 6.0
double SPHERE_RADIUS_DELTA = 0.5; //* How much the sphere grows in each iteration. 
double SPHERE_RADIUS = MIN_SPHERE_RADIUS; //* Distance with which the chasingPose is calculated.

float MAX_VELOCITY = 0.3; // was 0.5
float MIN_VELOCITY = 0.1; // was 0.2

float GROUND_HEIGHT = 0.05;
float GROUND_HEIGHT_OFFSET = 0.1;

bool isSimulation = true; //* Change depending on wether program is run in simulation or the real UAV.
int state = STABILIZE;
int stateBeforeTakeover = STABILIZE; // Keeps track of state before getting into PILOT_TAKEOVER to change back
int userInput = UI_NOT_AVAILABLE;
bool modes_initialized = false;
int expected_mode = MODE_STABILIZE;
int actual_mode = MODE_UNKNOWN;
bool armed = false;

tf2_ros::Buffer buffer;

geometry_msgs::PoseStamped pose_; //* The current pose of the UAV.
bool pose_received = false;


uavr_nav_msgs::Path action_path_; //* Waypoints in map-coordinate frame.
nav_msgs::Path path_; //* Waypoints in map-coordinate frame.

std::vector<geometry_msgs::PoseStamped> transformedWaypoints_; //* Waypoints in base-link coordinate frame.

geometry_msgs::PoseStamped chasingPose_; //* Pose which the UAV chases after.
geometry_msgs::PoseStamped velocityPose_; //* Pose of sphere for intersection


//! ONLY FOR EVALUATION
std::vector<double> minSquaredDistanceToPath;
bool minSquaredDistanceToPathInitialized = false;
rosbag::Bag bag_path;
geometry_msgs::TwistStamped current_veloctiy;


int pathIndex_ = 0; //* Index for chasingPose
int backtrackIndex_ = 0;
int fallbackPathIndex_ = 0;
int velocityPathIndex_ = 0; //* Index for the velocitySphere
bool waypoints_received = false;

//* Is set to true if last waypoint of path is reached.
//* Set to false if new path through FollowPath-Action is received. (Does NOT work for waypoints-topic)
bool last_waypoint_reached = false;

//* Is set to true if no radius for an alternative vector found.
//* Set to false if new path is received.
bool path_blocked = false;

float distance_to_goal = 0; //todo
float closest_obstacle = 0; //todo

sensor_msgs::PointCloud2 cloud2_;

pcl::PointCloud<pcl::PointXYZ> cloud_;

geometry_msgs::PoseStamped alternativePose_;

//! Only for visualization purposes (remove after debugging)
geometry_msgs::PointStamped problemPoint;

//* Object that gets returned from getPose()
struct getPoseType
{
    int poseType; // 0 = intersection, 1 = inside sphere (endpoint), 2 = not available
    geometry_msgs::PoseStamped returnPose;
};

//* Automatic state transitions from stabalize to hovering
bool automatic_to_guided = false;
bool automatic_to_arm = false;
bool automatic_to_takingoff = false;
ros::Time beginTakingOff;
double takingOffWaitingTime = 4; // in seconds 

//* Action stuff
typedef actionlib::SimpleActionServer<uavr_nav_msgs::TakeoffAction> TakeoffServer;
typedef actionlib::SimpleActionServer<uavr_nav_msgs::FollowPathAction> FollowPathServer;
typedef actionlib::SimpleActionServer<uavr_nav_msgs::LandAction> LandServer;

TakeoffServer* takeoffActionServer_;
FollowPathServer* followPathActionServer_;
LandServer* landActionServer_;

bool takeoffGoalHandleInitialized = false;
bool followPathGoalHandleInitialized = false;
bool landGoalHandleInitialized = false;

ros::Duration takeoffDelay;
float takeoffAgl = 0.35; // overwritten when action is called.

bool gotPathFromAction = false;

bool record_bagfiles = false;

typedef const boost::function< void(const geometry_msgs::PoseStamped& )>  callback;

// Reads parameters from parameter server
void readParams()
{
    nh_private->param<float>("takeoff_altitude_error_tolerance", TAKEOFF_ALTITUDE_ERROR_TOLERANCE, 0.1);
    nh_private->param<float>("last_waypoint_distance_tolerance", DISTANCE_TOLERANCE, 0.15);

    nh_private->param<float>("pointcloud_filter_sphere_radius", FILTER_CIRCLE_RADIUS, 0.43);

    nh_private->param<float>("safety_distance", SAFETY_DISTANCE, 0.6);

    nh_private->param<double>("min_sphere_radius", MIN_SPHERE_RADIUS, 0.8);
    nh_private->param<double>("max_sphere_radius", MAX_SPHERE_RADIUS, 2.0);
    nh_private->param<double>("sphere_radius_increment", SPHERE_RADIUS_DELTA, 0.5);

    nh_private->param<float>("max_linear_velocity", MAX_VELOCITY, 0.3);
    nh_private->param<float>("min_linear_velocity", MIN_VELOCITY, 0.1);

    nh_private->param<float>("ground_height", GROUND_HEIGHT, 0.05);
    nh_private->param<float>("ground_height_offset", GROUND_HEIGHT_OFFSET, 0.1);
}

// Returns the squared distance between two points given as vectors.
double getDistanceSquared(std::vector<double> a, std::vector<double> b)
{
    return (b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]) + (b[2] - a[2]) * (b[2] - a[2]);
}

// Converts quaternion to a yaw (Euler angels)
float quaternionToYaw(geometry_msgs::Quaternion quaternion)
{
    // Extract values from quaternion.
    float qx = quaternion.x;
    float qy = quaternion.y;
    float qz = quaternion.z;
    float qw = quaternion.w;

    // Conversion formula for yaw is atan2(qw * qz + qx * qy, 1 - 2 * (qy² + qz²))
    float yaw = atan2(2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qy*qy + qz*qz));

    // Return the yaw-value.
    return yaw;
}

// Converts a PoseStamped to a vector of the form [x,y,z,yaw]
std::vector<double> convertPoseStampedToVector(geometry_msgs::PoseStamped p)
{
    std::vector<double> poseVector = {
        p.pose.position.x, 
        p.pose.position.y, 
        p.pose.position.z, 
        quaternionToYaw(p.pose.orientation)
        };

    return poseVector;
}

//! ONLY FOR EVALUATION
void initializeMinDistanceToPath()
{
    std::vector<double> tmp(transformedWaypoints_.size(), 10000);
    minSquaredDistanceToPath = tmp;
    ////ROS_INFO("minSquaredDistanceToPath initialized.");
}

//! ONLY FOR EVALUATION
void updateMinSquaredDistanceToPath()
{
    double tmp;
    std::vector<double> zeroVec(4, 0);
    std::vector<double> waypointVec;

    for(int i = 0; i < minSquaredDistanceToPath.size(); i++)
    {
        waypointVec = convertPoseStampedToVector(transformedWaypoints_[i]);
        tmp = getDistanceSquared(zeroVec, waypointVec);

        if(tmp < minSquaredDistanceToPath[i])
        {
            minSquaredDistanceToPath[i] = tmp;
        }
    }
}

void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    if(msg->mode.compare("STABILIZE") == 0){
        actual_mode = MODE_STABILIZE; 
    } else if(msg->mode.compare("GUIDED") == 0){
        actual_mode = MODE_GUIDED;
    } else if(msg->mode.compare("LAND") == 0){
        actual_mode = MODE_LAND;
    } else {
        actual_mode = MODE_UNKNOWN;
    }
    armed = msg->armed;
    modes_initialized = true;
}


//forces finite automata to go into HOVERING state for testing!
void forceHovering()
{
    state = HOVERING;
}

void userInputCallback(const std_msgs::String& msg)
{
    if(msg.data.compare("guided") == 0)
    {
        userInput = UI_GUIDED;
    } 
    else if(msg.data.compare("arm") == 0)
    {
        userInput = UI_ARM;
    } 
    else if(msg.data.compare("takeoff") == 0)
    {
        userInput = UI_TAKEOFF;
    } 
    else if(msg.data.compare("control") == 0)
    {
        userInput = UI_CONTROL;
    } 
    else if(msg.data.compare("land") == 0)
    {
        userInput = UI_LAND;
    } 
    else if(msg.data.compare("restart") == 0)
    {
        userInput = UI_RESTART;
    }
    else if(msg.data.compare("hovering") == 0)
    {
        automatic_to_guided = true;
        automatic_to_arm = true;
        automatic_to_takingoff = true;
    }
    else if(msg.data.compare("force_hovering") == 0) // DO ONLY USE FOR TESTING
    {
        forceHovering();
    }
}

// Checks if certain duration has passed since a begin-time
bool checkTime(ros::Time begin, double duration)
{
    ros::Time current = ros::Time::now();

    ros::Duration timeDifferenceDuration = current - begin;
    
    if(timeDifferenceDuration.toSec() >= duration)
    {
        return true;
    }
    
    return false;
}

void INFO_STATE()
{
    if(state == 0)
    {
        ROS_INFO_THROTTLE(2,"state: STABILIZE");
    } 
    else if(state == 1)
    {
        ROS_INFO_THROTTLE(2,"state: GUIDED");
    }
    else if(state == 2)
    {
        ROS_INFO_THROTTLE(2,"state: ARMED");
    }
    else if(state == 3)
    {
        ROS_INFO_THROTTLE(2,"state: TAKING_OFF");
    }
    else if(state == 4)
    {
        ROS_INFO_THROTTLE(2,"state: HOVERING");
    }
    else if(state == 5)
    {
        ROS_INFO_THROTTLE(2,"state: CONTROL");
    }
    else if(state == 6)
    {
        ROS_INFO_THROTTLE(2,"state: LANDING");
    }
    else if(state == 7)
    {
        ROS_INFO_THROTTLE(2,"state: END");
    }
    else if(state == 8)
    {
        ROS_INFO_THROTTLE(2,"state: PILOT_TAKEOVER");
    }
}

bool waypoint_reached(geometry_msgs::PoseStamped wp)
{  
    float x,y,z;
    x = wp.pose.position.x;
    y = wp.pose.position.y;
    z = wp.pose.position.z;
    float distanceSquared = x*x + y*y + z*z;

    return distanceSquared <= DISTANCE_TOLERANCE*DISTANCE_TOLERANCE;
}

float distance_to_waypoint(geometry_msgs::PoseStamped wp)
{  
    float x,y,z;
    x = wp.pose.position.x;
    y = wp.pose.position.y;
    z = wp.pose.position.z;
    float distance = sqrt(x*x + y*y + z*z);

    return distance;
}

// Returns controlvector with no velocity and no yaw-rate.
geometry_msgs::Vector3 getZeroVelocityVector()
{
    geometry_msgs::Vector3 velocity_vector;
    velocity_vector.x = 0.0;
    velocity_vector.y = 0.0;
    velocity_vector.z = 0.0;

    return velocity_vector;
}

void waypointsCallback(const nav_msgs::Path& msg)
{ 
    path_ = msg; //nur zum testen

    //! for evaliation
    if(false)//!waypoints_received)
    {
        bag_path.open("path.bag", rosbag::bagmode::Write);
        
        for(int i = 0; i < msg.poses.size(); i++)
        {
            //geometry_msgs::PoseStamped pathPose = path_.poses[i].pose;
            bag_path.write("path", ros::Time::now(), path_.poses[i].pose);
        }

        bag_path.close();
    }

    waypoints_received = true;
}

void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cloud2_ = *msg;
    
    geometry_msgs::TransformStamped velodyne_to_base_link_transform;
    try
    {
        velodyne_to_base_link_transform = buffer.lookupTransform("base_link", "velodyne", ros::Time(0));
        sensor_msgs::PointCloud2 cloud2Transformed;
        tf2::doTransform(*msg, cloud2Transformed, velodyne_to_base_link_transform);
        pcl::fromROSMsg(cloud2Transformed, cloud_);
    }
    catch (tf2::TransformException &ex) 
    {
        pcl::fromROSMsg(*msg, cloud_);
        ROS_INFO("\n\n\nError while transforming PointCloud!\n\n\n");
        ROS_WARN("%s",ex.what());
    }
}

// Function: Transforms waypoints in "path_" in map-coordinate frame to "transformedWaypoints_" in base_link-coordinate frame.
// If during execution a path is received through the FollowPath-Action, paths received through the waypoints-topic will be ignored.
void transformWaypoints()
{
    // Check wether waypoints are received.
    if(!waypoints_received) return;


    geometry_msgs::TransformStamped map_to_base_link_transform;

    try
    {
        map_to_base_link_transform = buffer.lookupTransform("base_link", "map", ros::Time(0));    
        std::vector<geometry_msgs::PoseStamped> transformed_wps;

        if(gotPathFromAction)
        {

            for(int i = 0; i < action_path_.poses.size(); i++)
            {
                geometry_msgs::Pose wp_unstamped = action_path_.poses[i];
                geometry_msgs::PoseStamped wp;
                wp.pose = wp_unstamped;
                wp.header.stamp = ros::Time::now();
                wp.header.frame_id = "map";

                geometry_msgs::PoseStamped t_wp;
                tf2::doTransform(wp, t_wp, map_to_base_link_transform);
                transformed_wps.push_back(t_wp);
            }
        }
        else
        {
            for(int i = 0; i < path_.poses.size(); i++)
            {
                geometry_msgs::PoseStamped wp = path_.poses[i];
                geometry_msgs::PoseStamped t_wp;
                tf2::doTransform(wp, t_wp, map_to_base_link_transform);
                transformed_wps.push_back(t_wp);
            }
        }
        ROS_INFO("waypoints transformed.");
        transformedWaypoints_ = transformed_wps;

        if(!minSquaredDistanceToPathInitialized)
        {
            initializeMinDistanceToPath();
            minSquaredDistanceToPathInitialized = true;
        }
        
    } 
    catch (tf2::TransformException &ex) 
    {
        ROS_INFO("\n\n\nError in transformWaypoints!\n\n\n");
        ROS_WARN("%s",ex.what());
    }
}

void poseCallback(const geometry_msgs::PoseStamped& msg)
{       
        //ROS_INFO("PosCallback aufgerufen!");
        pose_ = msg;
        pose_received = true;
        transformWaypoints();
}

void resetUserInput()
{
    userInput = UI_NOT_AVAILABLE;
}

bool verifyTakeoff(float altitude)
{
    //ROS_INFO("current altitude: %.3f, expected altitude: %.3f", pose_.pose.position.z, altitude);
    return abs(pose_.pose.position.z - altitude) <= TAKEOFF_ALTITUDE_ERROR_TOLERANCE;
}

double absDouble(double value)
{
    if(value <= 0)
    {
        return value * (-1);
    }
    else
    {
        return value;
    }
}

double getSmallerYaw(double yawCurrent, double yawNext)
{

    double yawClockwiseRotation;

    if(yawCurrent >= yawNext)
    {
        // Clockwise rotation possible without going over the boundary.
        yawClockwiseRotation = yawNext - yawCurrent;
    }
    else
    {
        // Clockwise rotation going over the boundary at -pi.

        // Get rotation to the boundary at -pi.
        double ClockwiseRotationToMinusPi = (-PI) - yawCurrent; 
        
        // Get rotation from pi to waypoint.
        double ClockwiseRotationToWaypoint = yawNext - PI;
 
        // Combine both rotations to the rotation over the boundary.
        yawClockwiseRotation = ClockwiseRotationToMinusPi + ClockwiseRotationToWaypoint;
    }

    // Calculate the rotation needed for a counterclockwise rotation.
    double yawCounterclockwiseRotation;
    // Check wether the counterclockwise-rotation goes through the boundary at pi.
    if(yawCurrent <= yawNext)
    {
        // Counterclockwise rotation possible without going through boundary at pi.
        yawCounterclockwiseRotation = yawNext - yawCurrent;
    }
    else
    {
        // Counterclockwise rotation going over boundary at pi.
        
        // Get rotation to the boundary at pi.
        double CounterclockwiseRotationToPi = PI - yawCurrent;


        // Get rotation from -pi to waypoint.
        double CounterclockwiseRotationToWaypoint = yawNext + PI;

        // Combine both rotation to the rotation over the boundary.
        yawCounterclockwiseRotation = CounterclockwiseRotationToPi + CounterclockwiseRotationToWaypoint;
    }


    //? If difference is 2*PI it might lead to oscillations.
    //? So if there is no clear smaller rotation choose the clockwise rotation (arbitrarily) 
    double yawDifference = absDouble(absDouble(yawClockwiseRotation) - absDouble(yawCounterclockwiseRotation));
    if(yawDifference < 0.05)
    {
        return yawClockwiseRotation;
    }



    // Return the smaller of both rotations.
    if(absDouble(yawClockwiseRotation) <= absDouble(yawCounterclockwiseRotation))
    {
        return yawClockwiseRotation;
    }
    else
    {
        return yawCounterclockwiseRotation;
    }
}

geometry_msgs::Vector3 toVector3(float x, float y, float z)
{
    geometry_msgs::Vector3 vector;
    vector.x = x;
    vector.y = y;
    vector.z = z; 
    return vector;
}

std::vector<double> toDoubleVector(double x, double y, double z)
{
    std::vector<double> vec;

    vec.push_back(x);
    vec.push_back(y);
    vec.push_back(z);

    return vec;
}

geometry_msgs::Quaternion yawToQuaternion(float yaw)
{
    tf2::Quaternion quaternion;

    quaternion.setRPY(0, 0, yaw);
    quaternion.normalize();
        
    // Change the quaternion to geometry_msgs format.
    geometry_msgs::Quaternion quaternionFormatted;
    quaternionFormatted.x = quaternion.x();
    quaternionFormatted.y = quaternion.y();
    quaternionFormatted.z = quaternion.z();
    quaternionFormatted.w = quaternion.w();

    return quaternionFormatted;
}

// Returns the smallest yaw-error between two orientations.
float getYawError(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped waypoint)
{   
    // Extract Quaternions from poses.
    geometry_msgs::Quaternion poseOrientation = pose.pose.orientation;
    geometry_msgs::Quaternion waypointOrientation = waypoint.pose.orientation;
    
    // Convert Quaternion to Euler angels (only yaw is needed).
    float yawOfPose = quaternionToYaw(poseOrientation);
    float yawOfWaypoint = quaternionToYaw(waypointOrientation);
    
    // Calculate the clockwise and the counterclockwise rotation.

    return static_cast<float>(getSmallerYaw(static_cast<double>(yawOfPose), static_cast<double>(yawOfWaypoint)));
}

// Returns the yaw error to waypoint for base_link coordinate frame.
float getYawErrorToWaypoint(geometry_msgs::PoseStamped waypoint)
{   
    geometry_msgs::PoseStamped redundantPose;
    return getYawError(redundantPose, waypoint);
}

// Extracts error values based on the currently focused waypoint (in base_link-coordinate frame).
std::vector<float> getErrorVector(geometry_msgs::PoseStamped waypoint)
{
    std::vector<float> vec;
    vec.push_back(waypoint.pose.position.x);
    vec.push_back(waypoint.pose.position.y);
    vec.push_back(waypoint.pose.position.z);

    float yawError = getYawErrorToWaypoint(waypoint);
    vec.push_back(yawError);
    return vec;
}

geometry_msgs::PoseStamped convertVectorToPoseStamped(std::vector<double> vec)
{
    geometry_msgs::PoseStamped p;
    p.pose.position.x = vec[0];
    p.pose.position.y = vec[1];
    p.pose.position.z = vec[2];
    p.pose.orientation = yawToQuaternion(vec[3]);

    return p;
}

double getRelativeYaw(double yawCurrent, double yawNext, double ratio)
{
    double smallerYaw = getSmallerYaw(yawCurrent, yawNext);

    return yawCurrent + ratio * smallerYaw;
}

// Calculates intersection of a sphere, which is centered in the coordinate system, and a line between 2 points with relative 4th value.
// Returns 0,1 or 2 solutions.
std::vector<std::vector<double>> getIntersections(double radius, std::vector<double> n, std::vector<double> m)
{
    std::vector<std::vector<double>> intersections;

    double a = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
    double b = 2 * (n[0] * (m[0] - n[0]) + n[1] * (m[1] - n[1]) + n[2] * (m[2] - n[2]));
    double c = (m[0] - n[0]) * (m[0] - n[0]) + (m[1] - n[1]) * (m[1] - n[1]) + (m[2] - n[2]) * (m[2] - n[2]);

    
    // Calculate values for pq-formula.
    double p = b / c;
    double q = (a - radius * radius) / c;
    
    // Calculate discriminant
    double discriminant = (p / 2) * (p / 2) - q;

    //ROS_INFO("Discriminant: %.2f", discriminant);
    
    // Only count the solutions where 0 <= x <= 1 in order to reduce the intersections to the ones that are between the 2 points.
    int numberOfSolutions;
    std::vector<double> solutions;

    if(discriminant > 0)
    {
        // Two solutions exist
        double tmp1 = (-1) * (p / 2);
        double tmp2 = sqrt((p / 2) * (p / 2) - q);
        double solution1 = tmp1 - tmp2;
        double solution2 = tmp1 + tmp2;
        
        if(solution1 >= 0 && solution1 <= 1) 
        {
            solutions.push_back(solution1);
        }

        if(solution2 >= 0 && solution2 <= 1)
        {
            solutions.push_back(solution2);
        }
    } 
    else if(discriminant == 0)
    {   
        // One solutions exists.
        double solution = (-1) * (p / 2) + sqrt((p / 2) * (p / 2) - q);

        if(solution >= 0 && solution <= 1)
        {
            solutions.push_back(solution);
        }
    } 
    
    ////ROS_INFO("Number of solutions: %ld", solutions.size());
    for(int i = 0; i < solutions.size(); i++)
    {
        std::vector<double> intersection;
        intersection.push_back(n[0] + solutions[i] * (m[0] - n[0]));
        intersection.push_back(n[1] + solutions[i] * (m[1] - n[1]));
        intersection.push_back(n[2] + solutions[i] * (m[2] - n[2]));
        double relativeYaw =  getRelativeYaw(n[3], m[3], solutions[i]);
        ////ROS_INFO("relative Yaw: %.2f", relativeYaw);
        intersection.push_back(relativeYaw);
        intersections.push_back(intersection);
    }
    ////ROS_INFO("(in function) intersections.size() = %ld", intersections.size());
    return intersections;
}

bool currentWaypointIsLast(int* index)
{
    //ROS_INFO("ahhh");
    return (*index) == (transformedWaypoints_.size() - 1);
}

bool poseIsInsideSphere(double radius, geometry_msgs::PoseStamped p)
{
    double x = p.pose.position.x;
    double y = p.pose.position.y;
    double z = p.pose.position.z;

    return ((x*x + y*y + z*z) <= (radius*radius));
}

// Calculates chasingPose for a given distance. Retruns true if it was possible, false if not.
getPoseType getPose(double distance, int* index)
{   
    // Get current and next pose.
    geometry_msgs::PoseStamped currentPose;
    geometry_msgs::PoseStamped nextPose;
    std::vector<std::vector<double>> intersections;
    int intersectionsNumber;
    getPoseType returnObject;


    //? Look for chasePoint between current and next pose.
    //? There are 3 scenarios with a few sub-scenarios.

    while(true){
        //* get the waypoint for the current given index
        currentPose = transformedWaypoints_[(*index)];
        //* check weather the current waypoint is the last one
        if(!currentWaypointIsLast(index))
        {   
            //ROS_INFO("getPose 1");
            //* current waypoint is not the last one, so get the next waypoint and calculate intersections
            nextPose = transformedWaypoints_[(*index) + 1];
            intersections = getIntersections(distance, convertPoseStampedToVector(currentPose), convertPoseStampedToVector(nextPose));
            intersectionsNumber = intersections.size();
        }
        else 
        {   
            //* current waypoint is the last one.

            //* check if it's inside the sphere
            if(poseIsInsideSphere(distance, currentPose))
            {
                //* its inside the sphere :)
                //ROS_INFO("getPose(): pose is inside sphere");
                returnObject.returnPose = transformedWaypoints_[(*index)];
                returnObject.poseType = 1;
                return returnObject;
            }
            else
            {
                // Was soll gemacht werden, wenn der letzte Wegpunkt angesteuert werden soll, dieser aber außerhalb
                // der chasing-kugel liegt?
                // -> Kugel wird vergrößert
                returnObject.poseType = 2;
                return returnObject;
            }
        }
        if(intersectionsNumber == 0)
        {
            // Scenario 1: There is no intersection.
            if(poseIsInsideSphere(distance, currentPose))
            { 
                // Sub-scenario 1.1: Both points inside the sphere.
                // Update the current waypoint.
                (*index)++;
            }
            else
            { 
                // Sub-scenario 1.2: Both points outside of the sphere and no intersection. 
                returnObject.poseType = 2;
                return returnObject;
            }
        }
        else if(intersectionsNumber == 1)
        {
            // Scenario 2: There is 1 intersection.
    
            if(poseIsInsideSphere(distance, currentPose))
            {
                // Sub-scenario 2.1: Current pose is inside the sphere and next pose is outide the sphere.
                // ChasePose is the intersection.
                returnObject.returnPose = convertVectorToPoseStamped(intersections[0]);
                //returnObject.returnPose.header = transformedWaypoints_[(*index)].header;
                returnObject.returnPose.header.stamp = ros::Time::now();
                returnObject.returnPose.header.frame_id = "base_link";
                returnObject.poseType = 0;
                return returnObject;
            }
            else
            {
                // Sub-scenaio 2.2: Next pose is inside the sphere and current pose is outside the sphere.
                // Update current pose.
                (*index)++;
            }
        }
        else
        {
            // Scenario 3: There are 2 intersections.
            // Intersections are sorted by x value, so the second intersection is closer to the next waypoint.
            returnObject.returnPose = convertVectorToPoseStamped(intersections[1]);
            returnObject.returnPose.header.stamp = ros::Time::now();
            returnObject.returnPose.header.frame_id = "base_link";
            returnObject.poseType = 0;
            return returnObject;
        }
        //ROS_INFO("getPose 4");

    } // end while
    //ROS_INFO("getPose 5");
    returnObject.poseType = 2;
    return returnObject;

}

// Line is given as: startPoint + x * (endPoint - startPoint)
// Returns value x with smallest distance to a point.
double smallestDistanceToLine(std::vector<double> startPoint, std::vector<double> endPoint, std::vector<double> point)
{
    
    // For the extremely unlikely case that startpoint is exactly the endpoint, so that division through 0 is prevented.
    if((startPoint[0] == endPoint[0]) &&
       (startPoint[1] == endPoint[1]) &&
       (startPoint[2] == endPoint[3]))
    {
        startPoint[0] = startPoint[0] + 0.0001;
    }

    double diff0 = (endPoint[0] - startPoint[0]);
    double diff1 = (endPoint[1] - startPoint[1]);
    double diff2 = (endPoint[2] - startPoint[2]); 
    double num0 = (startPoint[0] - point[0]) * diff0;
    double num1 = (startPoint[1] - point[1]) * diff1;
    double num2 = (startPoint[2] - point[2]) * diff2;
    return (-1) * (num0 + num1 + num2) / (diff0 * diff0 + diff1 * diff1 + diff2 * diff2);
}

// Input: vectorStart: Point where the vector begins.
//        vectorEnd: Point where the vector ends.
//        distance: the distance that should be kept between the points in the pointcloud and the vector.
bool isValidVector(std::vector<double> vectorStart, std::vector<double> vectorEnd, pcl::PointCloud<pcl::PointXYZ> cloud, double distance)
{

    for (auto& point: cloud)
    {
        ////ROS_INFO("Current Point: x: %.2f, y: %.2f, z: %.2f", point.x, point.y, point.z);

        // Ignore point if it's inside uav-filter-circle
        if(FILTER_CIRCLE_RADIUS * FILTER_CIRCLE_RADIUS >= ((point.x - vectorStart[0])*(point.x - vectorStart[0]) + 
                                                           (point.y - vectorStart[1])*(point.y - vectorStart[1]) + 
                                                           (point.z - vectorStart[2])*(point.z - vectorStart[2])))
        {
            ////ROS_INFO("Ignoring Point because it's inside the filter-sphere");
            continue;
        }

        ////ROS_INFO("Point is NOT inside the filter-sphere");

        // Check circle around endpoint
        if(distance * distance >= ((point.x - vectorEnd[0])*(point.x - vectorEnd[0]) +
                                   (point.y - vectorEnd[1])*(point.y - vectorEnd[1]) + 
                                   (point.z - vectorEnd[2])*(point.z - vectorEnd[2])))
        {
            ////ROS_INFO("Point is inside of the Endpoint sphere. Invalid.");
            ////ROS_INFO("vectorEnd: x = %.2f, y = %.2f, z = %.2f", vectorEnd[0], vectorEnd[1], vectorEnd[2]);
            ////ROS_INFO("problem point: x = %.2f, y = %.2f, z = %.2f", point.x, point.y, point.z);

            problemPoint.point.x = point.x;
            problemPoint.point.y = point.y;
            problemPoint.point.z = point.z;
 

            return false;
        }
        
        problemPoint.point.x = point.x;
        problemPoint.point.y = point.y;
        problemPoint.point.z = point.z;

        // Check Distance to Line
        std::vector<double> pointConverted = toDoubleVector(point.x, point.y, point.z);
        
        // Position on line through start- and endpoint closest to point
        double sValue = smallestDistanceToLine(vectorStart, vectorEnd, pointConverted);
        
        // Check if position is on the vector.
        if(sValue >= 0 && sValue <= 1)
        {
            // Calculate closest point on vector.
            std::vector<double> closestPoint = toDoubleVector(vectorStart[0] + sValue * (vectorEnd[0] - vectorStart[0]),
                                                              vectorStart[1] + sValue * (vectorEnd[1] - vectorStart[1]),
                                                              vectorStart[2] + sValue * (vectorEnd[2] - vectorStart[2]));

            // Calculate the squared distance from point to closest point on vector.
            float distanceSquared = getDistanceSquared(closestPoint,pointConverted);

            // Check if distance from vector to point is too small
            if(distance*distance >= distanceSquared)
            {
                ////ROS_INFO("Point is too close to vector. Invalid.");
                return false;
            }

        }


        ////ROS_INFO("This Point was not a problem. Go next.");
    }
    
    // No point in pointcloud is too close to the vector.
    // Vector is valid.
    return true;

}

// Returns 0 if alternativePose is chasingPose_;
// Returns 1 if alternativePose is set and not chasingPose_;
// Returns 2 if valid alternativePose could not be found.
int setAlternativePose(ros::Publisher pub)
{
    // Check if chasingPose is valid

    std::vector<double> vectorStart = toDoubleVector(0,0,0);
    std::vector<double> vectorEnd = toDoubleVector(chasingPose_.pose.position.x, chasingPose_.pose.position.y, chasingPose_.pose.position.z);
    //ROS_INFO("x: %.2f, y: %.2f, z: %.2f",vectorEnd[0], vectorEnd[1], vectorEnd[2]);
    if(isValidVector(vectorStart, vectorEnd, cloud_, SAFETY_DISTANCE))
    {   
        //ROS_INFO("x: %.2f, y: %.2f, z: %.2f",vectorEnd[0], vectorEnd[1], vectorEnd[2]);
        alternativePose_ = chasingPose_;
        alternativePose_.header.stamp = ros::Time::now();
        alternativePose_.header.frame_id = "base_link";
        return 0;
    }

    // chasingPose_ is invalid. Look for alternativePose.
    double alphaChange = 0.2;
    double alpha = alphaChange;

    // Temporaray vector for alternativePose_
    std::vector<double> vectorNew = toDoubleVector(0,0,0);

    // alternativePose_ has the same z-component as chasingPose_
    vectorNew[2] = vectorEnd[2];
    
    float two = 2;
    while(absDouble(alpha) < static_cast<double>(PI/two))
    {
        vectorNew[0] = vectorEnd[0] * cos(alpha) - vectorEnd[1] * sin(alpha);
        vectorNew[1] = vectorEnd[0] * sin(alpha) + vectorEnd[1] * cos(alpha);

        //! TESTING ONLY (Visualization of alternative Pose)
        geometry_msgs::PointStamped alternativePoint;
        alternativePoint.point.x = vectorNew[0];
        alternativePoint.point.y = vectorNew[1];
        alternativePoint.point.z = vectorNew[2];
        alternativePoint.header.stamp = ros::Time::now(); 
        alternativePoint.header.frame_id = "base_link";
        pub.publish(alternativePoint);

        // check if vector to goal is smaller than drone to goal AND vector is valid
        if((getDistanceSquared(vectorNew, vectorEnd) < getDistanceSquared(vectorStart, vectorEnd)) && isValidVector(vectorStart, vectorNew, cloud_, SAFETY_DISTANCE))
        {
            // Set the valid alternativePose
            alternativePose_.pose.position.x = vectorNew[0];
            alternativePose_.pose.position.y = vectorNew[1];
            alternativePose_.pose.position.z = vectorNew[2];
            alternativePose_.header.stamp = ros::Time::now();
            alternativePose_.header.frame_id = "base_link";
            ////ROS_INFO("Alternative Pose was found! Alpha: %.3f", alpha);
            //ROS_INFO("FOUND ALTERNATIVE");
            return 1;
        }

        if(!(getDistanceSquared(vectorNew, vectorEnd) < getDistanceSquared(vectorStart, vectorEnd)))
        {
            //ROS_INFO("Alternativer Punkt außerhalb gültigen bereichs. Abbruch.");
            break;
        }
        
        if(alpha >= 0)
        {
            alpha = alpha * (-1);
        } else {
            alpha = alpha * (-1);
            alpha = alpha + alphaChange;
        }   
    }    

    // AlternativePose could not be found, do not move.
    alternativePose_.pose.position.x = 0;
    alternativePose_.pose.position.y = 0;
    alternativePose_.pose.position.z = 0;
    alternativePose_.pose.orientation.x = 0;
    alternativePose_.pose.orientation.y = 0;
    alternativePose_.pose.orientation.z = 0;
    alternativePose_.pose.orientation.w = 1.0;
    alternativePose_.header.stamp = ros::Time::now();
    alternativePose_.header.frame_id = "base_link";
    //ROS_INFO("do not move...!");
    return 2;
}

std::vector<float> scaleErrorVector(float length, std::vector<float> errorVector)
{
    std::vector<float> scaledErrorVector;

    //float lengthOfErrorVector = sqrt(errorVector[0]*errorVector[0] + errorVector[1]*errorVector[1] + errorVector[2]*errorVector[2]);
    scaledErrorVector.push_back(errorVector[0] * (length / static_cast<float>(SPHERE_RADIUS)));
    scaledErrorVector.push_back(errorVector[1] * (length / static_cast<float>(SPHERE_RADIUS)));
    scaledErrorVector.push_back(errorVector[2] * (length / static_cast<float>(SPHERE_RADIUS)));
    scaledErrorVector.push_back(errorVector[3] * (length / static_cast<float>(SPHERE_RADIUS)));
    return scaledErrorVector;
}

float scalingFunction(float x)
{
    return (1 + cos(((-1)*(x/PI)+2)*x))/2;
}

// calculates velocity from yaw
float yawToScalingValue(float yaw)
{   
    return (MAX_VELOCITY - MIN_VELOCITY) * scalingFunction(yaw) + MIN_VELOCITY;
}

//Todo: Implement delay for takeoff
void takeoffGoalCB()
{   
    //ROS_INFO("hier (test2)");
    takeoffGoalHandleInitialized = true;
    uavr_nav_msgs::TakeoffGoal goalTakeoff = *(takeoffActionServer_->acceptNewGoal());
    takeoffDelay = goalTakeoff.delay;
    takeoffAgl = goalTakeoff.agl;
    automatic_to_guided = true;
    automatic_to_arm = true;
    automatic_to_takingoff = true;
}

void takeoffPreemptCB()
{
    takeoffActionServer_->setPreempted();
}

//todo add low battery
void takeoffAnalysisCB(const geometry_msgs::PoseStamped& msg)
{
    if(takeoffGoalHandleInitialized)
    {
        uavr_nav_msgs::TakeoffResult result; 

        if(state == HOVERING)
        {
            result.result = 0;
            takeoffActionServer_->setSucceeded(result);
            takeoffGoalHandleInitialized = false;

        }
        else if(state == PILOT_TAKEOVER)
        {
            result.result = 1;
            takeoffActionServer_->setAborted(result);
            takeoffGoalHandleInitialized = false;
        }
        else if(state == STABILIZE || state == GUIDED || state == ARMED || state == TAKING_OFF)
        {
            uavr_nav_msgs::TakeoffFeedback feedback;
            feedback.current_agl = msg.pose.position.z;
            takeoffActionServer_->publishFeedback(feedback);
        }
        else
        {
            result.result = 255;
            takeoffActionServer_->setAborted(result);
            takeoffGoalHandleInitialized = false;
        }
    }
}

void followpathGoalCB()
{
    //ROS_INFO("here (in followpathGoalCB)");
    followPathGoalHandleInitialized = true;
    uavr_nav_msgs::FollowPathGoal goal = *(followPathActionServer_->acceptNewGoal());
    action_path_ = goal.path;
    pathIndex_ = 0; // reset path index
    gotPathFromAction = true;
    last_waypoint_reached = false;
    path_blocked = false;
    waypoints_received = true;
    transformWaypoints();
    userInput = UI_CONTROL;
}

void followpathPreemptCB()
{
    followPathActionServer_->setPreempted();
}

void followpathAnalysisCB(const geometry_msgs::PoseStamped& msg)
{

    if(followPathGoalHandleInitialized)
    {
        float distanceToGoal = distance_to_waypoint(transformedWaypoints_[transformedWaypoints_.size() - 1]);
        uavr_nav_msgs::FollowPathFeedback feedback;
        feedback.distance_to_goal = distanceToGoal;

        uavr_nav_msgs::FollowPathResult result;


        if(state == CONTROL)
        {
            followPathActionServer_->publishFeedback(feedback);
        }
        else if((state == HOVERING))
        {
            if(last_waypoint_reached)
            {
                result.distance_to_goal = distanceToGoal;
                result.result = 0; // success
                followPathActionServer_->setSucceeded(result);
                followPathGoalHandleInitialized = false;
                waypoints_received = false;
            }
            else if(path_blocked)
            {
                result.distance_to_goal = distanceToGoal;
                result.result = 2; // path blocked
                followPathActionServer_->setAborted(result);
                followPathGoalHandleInitialized = false;
                waypoints_received = false;
            }
            else if(userInput == UI_CONTROL)
            {
                followPathActionServer_->publishFeedback(feedback);
            }
            else
            {
                result.distance_to_goal = distanceToGoal;
                result.result = 255; // unknown error
                followPathActionServer_->setAborted(result); 
                followPathGoalHandleInitialized = false;
                waypoints_received = false;
            }

        }
        /*/else
        {
            result.distance_to_goal = distanceToGoal;
            result.result = 255; // unknown error
            followPathActionServer_->setAborted(result);
            followPathGoalHandleInitialized = false;
            waypoints_received = false;
        }*/
    }
}

void landGoalCB()
{
    landGoalHandleInitialized = true;
    uavr_nav_msgs::LandGoal goal = *(landActionServer_->acceptNewGoal());
    userInput = UI_LAND;
}

void landPreemptCB()
{
    landActionServer_->setPreempted();
}

void landAnalysisCB(const geometry_msgs::PoseStamped& msg)
{
    if(landGoalHandleInitialized)
    {
        uavr_nav_msgs::LandResult result;
        uavr_nav_msgs::LandFeedback feedback;

        float distanceToGround = msg.pose.position.z;

        if(state == GUIDED)
        {
            result.result = 0; // success
            landActionServer_->setSucceeded(result);
            landGoalHandleInitialized = false;
        }
        else if((state == HOVERING || state == CONTROL))
        {   
            // In HOVERING and CONTROL state, when land command is received, callbacks are executed once before state is changed LAND
            // Making sure program is in HOVERING or CONTROL state only once while callbacks are called.
            // (UserInput is reset only after a whole while-loop-iteration, so if state is HOVERING or CONTROL but userInput is still land it's still normal behaviour)
            if(userInput == UI_LAND)
            {
                feedback.distance_to_ground = distanceToGround;
                landActionServer_->publishFeedback(feedback); 
            }
            else
            {
                result.result = 255; // unknown error
                landActionServer_->setAborted(result);
                landGoalHandleInitialized = false;
            }

        }
        else if(state == LANDING)
        {
            feedback.distance_to_ground = distanceToGround;
            landActionServer_->publishFeedback(feedback);
        }
        else if(state == PILOT_TAKEOVER)
        {
            result.result = 1; // pilot takeover
            landActionServer_->setAborted(result);
            landGoalHandleInitialized = false;
        }
        else
        {
            result.result = 255; // unknown error
            landActionServer_->setAborted(result);
            landGoalHandleInitialized = false;
        }
    }
}

void velocityCallback(const geometry_msgs::TwistStamped& msg)
{
        current_veloctiy = msg;
        //ROS_INFO("velocity: %.2f",current_veloctiy.twist.linear.x);
}



int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "test1");
    ros::NodeHandle nh;
    nh_private = new ros::NodeHandle("~");

    readParams();

    //----------Subscriber----------//
    ros::Subscriber sub_userInput = nh.subscribe("user_input", 1, userInputCallback);
    ros::Subscriber sub_state = nh.subscribe("mavros/state",1 , stateCallback);
    ros::Subscriber sub_pose;
    ros::Subscriber sub_velocity = nh.subscribe("mavros/local_position/velocity_body",1 , velocityCallback);


    if(isSimulation){
        sub_pose = nh.subscribe("mavros/local_position/pose", 1, poseCallback);
    } else {
        sub_pose = nh.subscribe("mavros/vision_pose/pose", 1, poseCallback);
    }

    //ros::Subscriber sub_waypoints = nh.subscribe("waypoints", 1, waypointsCallback);

    ros::Subscriber sub_velodyne;

    if(isSimulation)
    {
        sub_velodyne = nh.subscribe("velodyne", 1, velodyneCallback);
    }
    else
    {
        sub_velodyne = nh.subscribe("velodyne_points", 1, velodyneCallback);
    }
    
    //----------ServiceClients----------//
    ros::ServiceClient service_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient service_arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient service_takeoff = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");

    //----------Publisher----------//
    ros::Publisher pub_setpoint = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
    ros::Publisher pub_vel_vector = nh.advertise<geometry_msgs::TwistStamped>("velocity_vector", 1);
    ros::Publisher pub_chasingPose = nh.advertise<geometry_msgs::PoseStamped>("chasing_pose", 1);
    ros::Publisher pub_marker1 = nh.advertise<visualization_msgs::Marker>("filter_sphere_marker", 1);
    ros::Publisher pub_marker2 = nh.advertise<visualization_msgs::Marker>("safety_sphere_marker", 1);
    ros::Publisher pub_problemPoint = nh.advertise<geometry_msgs::PointStamped>("problem_point", 1);
    ros::Publisher pub_chasingPoint = nh.advertise<geometry_msgs::PointStamped>("pub_chasingPoint", 1);
    ros::Publisher pub_alternativePoint = nh.advertise<geometry_msgs::PointStamped>("pub_alternativePoint", 1);

    //---------Actions-----------//

    //* Takeoff-Action
    takeoffActionServer_ = new TakeoffServer(nh, "takeoff", false);
    takeoffActionServer_->registerGoalCallback(boost::bind(&takeoffGoalCB));
    takeoffActionServer_->registerPreemptCallback(boost::bind(&takeoffPreemptCB));
    ros::Subscriber takeoffPoseSub;
    if(isSimulation){
        takeoffPoseSub = nh.subscribe("mavros/local_position/pose", 1, takeoffAnalysisCB);
    } else {
        takeoffPoseSub = nh.subscribe("mavros/vision_pose/pose", 1, takeoffAnalysisCB);
    }
    
    //* FollowPath-Action
    followPathActionServer_ = new FollowPathServer(nh, "followPath", false);
    followPathActionServer_->registerGoalCallback(boost::bind(&followpathGoalCB));
    followPathActionServer_->registerPreemptCallback(boost::bind(&followpathPreemptCB));
    ros::Subscriber followPathPoseSub;
    if(isSimulation){
        followPathPoseSub = nh.subscribe("mavros/local_position/pose", 1, followpathAnalysisCB);
    } else {
        followPathPoseSub= nh.subscribe("mavros/vision_pose/pose", 1, followpathAnalysisCB);
    }
   
    //* Land-Action
    landActionServer_ = new LandServer(nh, "land", false);
    landActionServer_->registerGoalCallback(boost::bind(&landGoalCB));
    landActionServer_->registerPreemptCallback(boost::bind(&landPreemptCB));
    ros::Subscriber landPoseSub;
    if(isSimulation){
        landPoseSub = nh.subscribe("mavros/local_position/pose", 1, landAnalysisCB);
    } else {
        landPoseSub= nh.subscribe("mavros/vision_pose/pose", 1, landAnalysisCB);
    }

    //* Starting action servers

    //ROS_INFO("versuche takeoffActionServer zu starten.");
    takeoffActionServer_->start();
    //ROS_INFO("server erfolgreich gestartet.");
    followPathActionServer_->start();
    landActionServer_->start();


    //Server followPathActionServer(nh, "follow_path", boost::bind(&executeFollowPathAction, _1, &followPathActionServer), false);
    //followPathActionServer.start();
    
    //----------Messages----------//
    mavros_msgs::SetMode srv_mode;
    srv_mode.request.base_mode = 0;
    
    mavros_msgs::CommandBool srv_arming;
    srv_arming.request.value = true;

    mavros_msgs::CommandTOL srv_takeoff; 
    srv_takeoff.request.min_pitch = 0.0;
    srv_takeoff.request.yaw = 0.0;
    srv_takeoff.request.latitude = 0.0;
    srv_takeoff.request.longitude = 0.0;
    srv_takeoff.request.altitude = 0.35;  //* set altitude for takeoff
    mavros_msgs::PositionTarget msg_setpoint;
    msg_setpoint.coordinate_frame = 8;
    msg_setpoint.type_mask = 1991;
    msg_setpoint.yaw_rate = 0.0;

    ros::Rate setpoint_loop_rate(50);

    //* Controller initialization
    PoseController controller(
        1.0, 0, 0, //* x 
        1.0, 0, 0, //* y
        1.0, 0, 0, //* z
        KP, KI, KD  //* yaw
    );
    
    //! TESTING ONLY
    geometry_msgs::Vector3 velocity_vector;
    geometry_msgs::Vector3 velocity_vector_to_next_wp;

    tf2_ros::TransformListener listener(buffer);


    //! EVALUATION
    rosbag::Bag bag_poses;
    rosbag::Bag bag_velocity;
    rosbag::Bag bag_sphere_radius;

    if(record_bagfiles)
    {
        bag_poses.open("poses.bag", rosbag::bagmode::Write);
        bag_velocity.open("velocity.bag", rosbag::bagmode::Write);
        bag_sphere_radius.open("sphere_radius.bag", rosbag::bagmode::Write);
    }
    

    ros::Time timeOfControlStart;
    bool startedTimeOfControlStart = false;
    ros::Time timeOfEndPointReached;
    bool startedTimeOfEndPointReached = false;
    ros::Duration flightDuration;

    while(ros::ok()){
        ////ROS_INFO("expected_mode: %d", expected_mode);
        ////ROS_INFO("actual_mode: %d", actual_mode);
        ////ROS_INFO("modes_initialized: %d", modes_initialized);

        INFO_STATE();

        if(!modes_initialized)
        {
            resetUserInput();
            setpoint_loop_rate.sleep();
            ros::spinOnce();
            continue;
        }

        if(state != PILOT_TAKEOVER && expected_mode != actual_mode)
        {
            stateBeforeTakeover = state;
            state = PILOT_TAKEOVER;
        }

        switch(state){
            case STABILIZE:
                if(userInput == UI_GUIDED){
                    srv_mode.request.custom_mode = "GUIDED";
                    service_mode.call(srv_mode);
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    if(actual_mode == MODE_GUIDED){
                        expected_mode = MODE_GUIDED;
                        state = GUIDED;
                    }
                }

                if(automatic_to_guided)
                {
                    automatic_to_guided = false;
                    srv_mode.request.custom_mode = "GUIDED";
                    service_mode.call(srv_mode);
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    if(actual_mode == MODE_GUIDED){
                        expected_mode = MODE_GUIDED;
                        state = GUIDED;
                    }
                }
                break;
        
            case GUIDED:
                if(userInput == UI_ARM){
                    service_arming.call(srv_arming);
                    service_mode.call(srv_mode);
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    if(armed){
                        state = ARMED;
                    }
                }

                if(automatic_to_arm)
                {
                    automatic_to_arm = false;
                    service_arming.call(srv_arming);
                    service_mode.call(srv_mode);
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    if(armed){
                        state = ARMED;
                        beginTakingOff = ros::Time::now();
                    }
                }

                break;
                

            case ARMED:
                if(!armed){
                    state = GUIDED;
                    break;
                }
                
                if(userInput == UI_TAKEOFF){
                    service_takeoff.call(srv_takeoff);
                    if(srv_takeoff.response.success){
                        state = TAKING_OFF;
                    } else {
                        //ROS_INFO("takeoff didn't work");
                    }
                }
                
                if(automatic_to_takingoff && checkTime(beginTakingOff, takingOffWaitingTime)) //todohere
                {
                    automatic_to_takingoff = false;
                    srv_takeoff.request.altitude = takeoffAgl;  //* get altitude from action
                    service_takeoff.call(srv_takeoff);
                    if(srv_takeoff.response.success){
                        state = TAKING_OFF;
                    } else {
                        //ROS_INFO("takeoff didn't work");
                    }
                }
                break;

            case TAKING_OFF:
                if(verifyTakeoff(srv_takeoff.request.altitude)){
                    state = HOVERING;
                }
                break;

            case HOVERING:
                
                if(userInput == UI_CONTROL){
                    state = CONTROL;
                }
                else if(userInput == UI_LAND)
                {
                    // Send land request.
                    srv_mode.request.custom_mode = "LAND";
                    service_mode.call(srv_mode);
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    // Check if the UAV is in landing-mode
                    if(actual_mode == MODE_LAND)
                    {
                        expected_mode = MODE_LAND;
                        state = LANDING;
                    }
                }

                break;

            case CONTROL:

                /*
                //! ONLY FOR EVALUATION
                updateMinSquaredDistanceToPath();
                if(!startedTimeOfControlStart)
                {
                    startedTimeOfControlStart = true;
                    timeOfControlStart = ros::Time::now();
                }

                if(!last_waypoint_reached)
                {
                    geometry_msgs::PoseStamped local_pose = pose_;
                    bag_poses.write("/mavros/local_position/pose", ros::Time::now(), local_pose);

                    geometry_msgs::TwistStamped local_velocity = current_veloctiy;
                    bag_velocity.write("/mavros/local_position/velocity", ros::Time::now(), current_veloctiy);

                    //! Für Evaluation des Fallback-Radius -.-.-.-.
                    //* x und y Komponenten sind die Pose des UAV. z-Komponente ist der Sphere Radius.
                    geometry_msgs::PoseStamped sphereRadiusPose = pose_;
                    sphereRadiusPose.pose.position.z = SPHERE_RADIUS;
                    bag_sphere_radius.write("sphere radius", ros::Time::now(), sphereRadiusPose);
                    //! .-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

                }
                */

                // Check wether inputs "land".
                if(userInput == UI_LAND)
                {
                    // Send land request.
                    srv_mode.request.custom_mode = "LAND";
                    service_mode.call(srv_mode);
                    ros::Duration(1).sleep();
                    // Update the callbacks to get the actual mode of the UAV.
                    ros::spinOnce();
                    // Check if the UAV is in landing-mode
                    if(actual_mode == MODE_LAND)
                    {
                        // UAV is expected ry_msgs::PoseStamped rviz_chasingPose;
                        expected_mode = MODE_LAND;
                        state = LANDING;
                    }
                }
                else
                {

                    getPoseType chasingPoseObject;
                    int alternativePoseFeedback; 
                    
                    // Pfad der betrachtet wird, beginnt ab dem letzten Pfadabschnitt mit der ChasingKugel.
                    

                    /*
                    if(fallbackPathIndex_ > pathIndex_)
                    {
                        pathIndex_ = fallbackPathIndex_;
                        while(pathIndex_ >= 0)
                        {
                            chasingPoseObject = getPose(MIN_SPHERE_RADIUS, &pathIndex_);
                            if(chasingPoseObject.poseType == 2)
                            {
                                pathIndex_--;
                            }
                            else
                            {
                                chasingPose_ = chasingPoseObject.returnPose;
                                alternativePoseFeedback = setAlternativePose(pub_alternativePoint);
                                if(alternativePoseFeedback != 2)
                                {
                                    //ROS_INFO("Kugel erfolgreich verkleinert.");
                                    break;
                                }
                            }
                        }
                    }
                    */




                    //ROS_INFO("path index: %d", pathIndex_);

                    // Radius wird auf normale größere zurückgesetzt.
                    SPHERE_RADIUS = MIN_SPHERE_RADIUS;
                    
                    //* Finden der ChasingPose und AlternativePose.
                    while(true)
                    {   

                        //if(SPHERE_RADIUS == MIN_SPHERE_RADIUS)
                        //{
                            // Chasing-Sphere is used
                            
                            backtrackIndex_ = pathIndex_;
                            while(backtrackIndex_ >= 0)
                            {
                                chasingPoseObject = getPose(SPHERE_RADIUS, &backtrackIndex_);

                                if(chasingPoseObject.poseType == 2)
                                {
                                    backtrackIndex_--;
                                }
                                else
                                {
                                    pathIndex_ = backtrackIndex_;
                                    break;
                                }
                            }
                        //}
                        //else
                        //{
                            // Fallback-Sphere is used
                            //fallbackPathIndex_ = pathIndex_;
                            //chasingPoseObject = getPose(SPHERE_RADIUS, &fallbackPathIndex_);
                            //chasingPoseObject = getPose(SPHERE_RADIUS, &pathIndex_);
                        //}
                        
                        chasingPose_ = chasingPoseObject.returnPose;
                        alternativePoseFeedback = setAlternativePose(pub_alternativePoint);

                        //ROS_INFO("setAlternativePose: %d", alternativePoseFeedback);

                        if((chasingPoseObject.poseType != 2) && (alternativePoseFeedback != 2))
                        {
                            break;
                        }

                        if(SPHERE_RADIUS + SPHERE_RADIUS_DELTA <= MAX_SPHERE_RADIUS)
                        {
                            SPHERE_RADIUS = SPHERE_RADIUS + SPHERE_RADIUS_DELTA;
                        }
                        else
                        {
                            //! Kommen hier mit chasingPoseObject.poseType == 2 oder mit alternativePoseFeedback == 2 raus.
                            //! Diese Fälle werden unten abgefangen.
                            break;
                            
                        }
                    }

                    
                    
                    //ROS_INFO("chasingPoseObject.poseType: %d" ,chasingPoseObject.poseType);
                    //ROS_INFO("alternativePoseFeedback: %d", alternativePoseFeedback);

                    //* Für keinen Radius wurde eine Schnittstelle gefunden.
                    if(chasingPoseObject.poseType == 2)
                    {
                        //* No intersection available.
                        //* Set speed to zero and try again.

                            // No alternative for maximal radius found.
                            // Abbort the mission by changing to guided.
                            //ROS_INFO("No intersection for maximal radius found. Abborting mission.");
                            //todo: Modus waiting oder so einführen.
                            srv_mode.request.custom_mode = "GUIDED";
                            service_mode.call(srv_mode);
                            ros::Duration(1).sleep();
                            ros::spinOnce();
                            if(actual_mode == MODE_GUIDED){
                                expected_mode = MODE_GUIDED;
                                state = HOVERING;
                            }
                            break;
                        
                    }
                    
                    //* Für keinen Radius wurde eine Alternative gefunden.
                    if(alternativePoseFeedback == 2){
                        path_blocked = true;
                        //ROS_INFO("No alternative Pose for every possible Radius. Abborting mission");
                        //todo: Modus waiting oder so einführen.
                        srv_mode.request.custom_mode = "GUIDED";
                        service_mode.call(srv_mode);
                        ros::Duration(1).sleep();
                        ros::spinOnce();
                        if(actual_mode == MODE_GUIDED){
                            expected_mode = MODE_GUIDED;
                            state = HOVERING;
                        }
                        break;
                    }

                    float velocityValue = 0; 
                    velocityPathIndex_ = pathIndex_;

                    getPoseType velocityPoseObject = getPose(SPHERE_RADIUS * 1.5, &velocityPathIndex_);
                    //ROS_INFO("velocity path index: %d", velocityPathIndex_);
                    velocityPose_ = velocityPoseObject.returnPose;


                    //! Visualizazion------
                    geometry_msgs::PointStamped chasingPoint;         
                    chasingPoint.point = chasingPose_.pose.position;  
                    chasingPoint.header = chasingPose_.header;        
                    pub_chasingPoint.publish(chasingPoint);           
                    //!---------------------

                    //ROS_INFO("alternative pose feedback: %d", alternativePoseFeedback);

                    //* Keine alternative wurde verwendet. Originaler ChasingPose vektor wird verwendet.
                    if(alternativePoseFeedback == 0)
                    {
                        // if it was found through chasing_sphere and velocity_pose also has an intersection
                        if(SPHERE_RADIUS == MIN_SPHERE_RADIUS)
                        {
                            //* Chasing sphere and velocity sphere both have intersections with path
                            if((chasingPoseObject.poseType == 0) && (velocityPoseObject.poseType == 0))
                            {
                                //* Winkel berechnen
                                float yaw = getYawError(chasingPose_, velocityPose_);
                                velocityValue = yawToScalingValue(yaw);
                            }
                            else if((chasingPoseObject.poseType == 0) && (velocityPoseObject.poseType == 1))
                            {
                                //* Minimale Geschwindigkeit
                                velocityValue = MIN_VELOCITY;
                            }
                            else if((chasingPoseObject.poseType == 1) && (velocityPoseObject.poseType == 1))
                            {
                                if(waypoint_reached(transformedWaypoints_[transformedWaypoints_.size() - 1]))
                                {   

                                    velocityValue = 0;

                                    if(record_bagfiles)
                                    {
                                        bag_poses.close();
                                        bag_velocity.close();
                                        bag_sphere_radius.close();
                                    }
                                    
                                    //* Change into "Hovering" mode
                                    state = HOVERING;
                                    last_waypoint_reached = true;
                                }
                                else
                                {
                                    //* MIN_VELOCITY but scaled to distance to last waypoint
                                    velocityValue = MIN_VELOCITY + ((distance_to_waypoint(transformedWaypoints_[transformedWaypoints_.size() - 1]) / SPHERE_RADIUS) * (MAX_VELOCITY - MIN_VELOCITY));
                                }
                            }
                        }
                        else
                        {
                            //* Fliegen mittels Fallback-Kugel deshalb mindeste Geschwindigkeit.
                            velocityValue = MIN_VELOCITY;
                        }
                        
                    }
                    //* chasingPose_ is invalid, but valid alternativePose_ was found and set.
                    else if(alternativePoseFeedback == 1)
                    {   
                        velocityValue = MIN_VELOCITY;
                    }

                    // Get the error-vector to alternativePose_
                    alternativePose_.header.stamp = ros::Time::now();
                    std::vector<float> errorVector = getErrorVector(alternativePose_);
                    std::vector<float> scaledErrorVector = scaleErrorVector(velocityValue, errorVector);

                    // Give error-vector to controller.
                    controller.setError(scaledErrorVector[0], scaledErrorVector[1], scaledErrorVector[2], scaledErrorVector[3]);

                    // Get the control-values from the controller.
                    std::vector<float> controlVector = controller.getControl();
                    geometry_msgs::Vector3 velocityVector = toVector3(controlVector[0], controlVector[1], controlVector[2]);

                    //ROS_INFO("Moving towards: x=%.2f, y=%.2f, z=%.2f",controlVector[0], controlVector[1], controlVector[2]);

                    // Send control-values to the UAV.
                    msg_setpoint.velocity = velocityVector;
                    msg_setpoint.yaw_rate = controlVector[3];
                    pub_setpoint.publish(msg_setpoint);
                }
                
                break;

            case LANDING:

                // Switch to state GUIDED when the UAV is disarmed.
                if(!armed)
                {
                    srv_mode.request.custom_mode = "GUIDED";
                    service_mode.call(srv_mode);
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    if(actual_mode == MODE_GUIDED){
                        expected_mode = MODE_GUIDED;
                        state = GUIDED;
                    }
                }
                break;

            case END:
                {
                    // Go back to stabalize-mode if user wants to restart the UAV.
    
                    //! Only for Evaluation----
                    double meanSquaredError= 0;
                    for(int z = 0; z < minSquaredDistanceToPath.size(); z++)
                    {
                        meanSquaredError = meanSquaredError + minSquaredDistanceToPath[z];
                    }
    
                    meanSquaredError = meanSquaredError / minSquaredDistanceToPath.size();
    
                    //ROS_INFO("mean squared error: %.5f", meanSquaredError);
                    //ROS_INFO("---------------");

                    if(!startedTimeOfEndPointReached){
                        startedTimeOfEndPointReached = true;
                        timeOfEndPointReached = ros::Time::now();
                        flightDuration = timeOfEndPointReached - timeOfControlStart;
                    }

                    //ROS_INFO("flight duration: %lf", flightDuration.toSec());
                    //!---------------------------
                    
    
                    if(userInput == UI_RESTART)
                    {
                        // Send land request.
                        srv_mode.request.custom_mode = "STABILIZE";
                        service_mode.call(srv_mode);
                        ros::Duration(1).sleep();
    
                        // Update the callbacks to get the actual mode of the UAV.
                        ros::spinOnce();
    
                        // Check if the UAV is in landing-mode
                        if(actual_mode == MODE_STABILIZE)
                        {
                            // UAV is expected to be in landing-mode from now on.
                            expected_mode = MODE_STABILIZE;
                            state = STABILIZE;
                        }
                    }
                }
                break;

            case PILOT_TAKEOVER:
                //ROS_INFO("expected mode (%d), actual mode (%d)", expected_mode, actual_mode);
                //ROS_INFO("stateBeforeTakeover: %d", stateBeforeTakeover);

                // change into mode guided if drone is disarmed and guided
                if((actual_mode == MODE_GUIDED) && !armed)
                {
                    expected_mode = MODE_GUIDED;
                    state = GUIDED;
                }
                
                // uav is armed, in mode guided and flying
                if((actual_mode == MODE_GUIDED) && armed && (pose_.pose.position.z > GROUND_HEIGHT + GROUND_HEIGHT_OFFSET))
                {
                    //ROS_INFO("here1");
                    if((stateBeforeTakeover == HOVERING) || (stateBeforeTakeover == CONTROL))
                    {
                        //ROS_INFO("here2");
                        state = stateBeforeTakeover;
                    }
                    
                }

                if(userInput == UI_GUIDED){
                    srv_mode.request.custom_mode = "GUIDED";
                    service_mode.call(srv_mode);
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    if(actual_mode == MODE_GUIDED){
                        expected_mode = MODE_GUIDED;
                        state = GUIDED;
                    }
                }

                break;
        }

        //* Visualization for rviz.
        if(state == CONTROL)
        {   
            
            // Visualize control-vector
            geometry_msgs::TwistStamped rviz_vel_vector;
            rviz_vel_vector.header.stamp = pose_.header.stamp;
            rviz_vel_vector.header.frame_id = "base_link";
            rviz_vel_vector.twist.linear.x = msg_setpoint.velocity.x;
            rviz_vel_vector.twist.linear.y = msg_setpoint.velocity.y;
            rviz_vel_vector.twist.linear.z = msg_setpoint.velocity.z;
            rviz_vel_vector.twist.angular.z = msg_setpoint.yaw_rate;
            pub_vel_vector.publish(rviz_vel_vector);

            // Visualize chasePose
            geometry_msgs::PoseStamped rviz_chasingPose;
            rviz_chasingPose = alternativePose_;
            rviz_chasingPose.header.stamp = ros::Time::now();
            rviz_chasingPose.header.frame_id = "base_link";
            pub_chasingPose.publish(rviz_chasingPose);

            visualization_msgs::Marker safety_sphere;
            safety_sphere.header.frame_id = "base_link";
            safety_sphere.header.stamp = ros::Time::now();
            safety_sphere.ns = "safety";
            safety_sphere.id = 0;
            safety_sphere.type = visualization_msgs::Marker::SPHERE;
            safety_sphere.action = visualization_msgs::Marker::ADD;
            safety_sphere.pose.position.x = alternativePose_.pose.position.x;
            safety_sphere.pose.position.y = alternativePose_.pose.position.y;
            safety_sphere.pose.position.z = alternativePose_.pose.position.z;
            safety_sphere.pose.orientation.x = 0.0;
            safety_sphere.pose.orientation.y = 0.0;
            safety_sphere.pose.orientation.z = 0.0;
            safety_sphere.pose.orientation.w = 1.0;
            safety_sphere.scale.x = SAFETY_DISTANCE * 2;
            safety_sphere.scale.y = SAFETY_DISTANCE * 2;
            safety_sphere.scale.z = SAFETY_DISTANCE * 2;
            //safety_sphere.scale.x = 0.05;
            //safety_sphere.scale.y = 0.05;
            //safety_sphere.scale.z = 0.05;

            safety_sphere.color.r = 0.0f;
            safety_sphere.color.g = 0.0f;
            safety_sphere.color.b = 1.0f;
            safety_sphere.color.a = 1.0;

            pub_marker2.publish(safety_sphere);
            
            //Publish point from Pointcloud which is makes the vector invalid.
            problemPoint.header.stamp = ros::Time::now();
            problemPoint.header.frame_id = "base_link";
            pub_problemPoint.publish(problemPoint);

        }

        //* Visualization of the Filter-Circle

        visualization_msgs::Marker filter_sphere;
        filter_sphere.header.frame_id = "base_link";
        filter_sphere.header.stamp = ros::Time::now();
        filter_sphere.ns = "filter";
        filter_sphere.id = 0;
        filter_sphere.type = visualization_msgs::Marker::SPHERE;
        filter_sphere.action = visualization_msgs::Marker::ADD;
        filter_sphere.pose.position.x = 0;
        filter_sphere.pose.position.y = 0;
        filter_sphere.pose.position.z = 0;
        filter_sphere.pose.orientation.x = 0.0;
        filter_sphere.pose.orientation.y = 0.0;
        filter_sphere.pose.orientation.z = 0.0;
        filter_sphere.pose.orientation.w = 1.0;
        filter_sphere.scale.x = FILTER_CIRCLE_RADIUS * 2;
        filter_sphere.scale.y = FILTER_CIRCLE_RADIUS * 2;
        filter_sphere.scale.z = FILTER_CIRCLE_RADIUS * 2;
        filter_sphere.color.r = 0.0f;
        filter_sphere.color.g = 1.0f;
        filter_sphere.color.b = 0.0f;
        filter_sphere.color.a = 1.0;

        pub_marker1.publish(filter_sphere);

        // Resets user input after each iteration to default.
        resetUserInput();

        setpoint_loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}