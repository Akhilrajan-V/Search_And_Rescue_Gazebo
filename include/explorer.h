#ifndef EXPLORER_H
#define EXPLORER_H

#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

/**
 * @brief A class to control the functionality of the explorer robot
 * Contains the various flags to control roatation, broadcast and listen operations
 * Function declarion of m_rotate to rotate the explorer once it reaches target 
 * Declaration of m_stop to stop rotation once it detects Aruco marker
 * Declaration of Listener, Publisher to publish topic on cmd_vel and Subscriber to subscribe to /fiducial_transforms
 */
class Explorer
{
public:
    Explorer(ros::NodeHandle *nodehandle);
    bool m_broadcast_to_call;
    bool m_broadcasted ;
    bool m_explorer_done;    
    int m_aruco_fiducial_id;
    std::array<std::array<double,2>,4> m_follower_locations;
    void m_rotate();
    void m_stop();
    void m_listen(tf2_ros::Buffer& tfBuffer);

private:
    double m_angular_speed;
    ros::NodeHandle m_nh;
    ros::Subscriber m_fiducial_subscriber;
    ros::Publisher m_velocity_publisher;
    void m_broadcast_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);     
    void m_broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg1); 
    void m_initialize_subscriber();
    void m_initialize_publisher();
};
#endif