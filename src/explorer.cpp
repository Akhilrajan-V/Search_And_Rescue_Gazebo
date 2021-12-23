#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include "explorer.h"

//The constructor of class Explorer
Explorer::Explorer(ros::NodeHandle* nodehandle):
    m_nh{ *nodehandle },
    m_angular_speed{ 0.4}, 
    m_broadcast_to_call{false},
    m_broadcasted {false},
    m_explorer_done {false},    
    m_aruco_fiducial_id{0},
    m_follower_locations{0}
{
    m_initialize_subscriber();//Initialize the Subscriber to topic /fiducial_transforms
    m_initialize_publisher();//Initialize the publisher to topic /cmd_vel
}

void Explorer::m_initialize_publisher() {
    ROS_INFO("Initializing Publishers");
    m_velocity_publisher = m_nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100);

}

void Explorer::m_initialize_subscriber() {
    ROS_INFO("Initializing Subscribers");
    m_fiducial_subscriber = m_nh.subscribe("/fiducial_transforms", 1, &Explorer::m_broadcast_callback, this);
   
}
/**
 * @brief This is the TF Broadcaster function
 * The parent is frame explorer_tf/camera_rgb_optical_frame
 * It creates a new child tf frame called marker_frame * 
 * @param msg1 
 */
void Explorer::m_broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg1) {


  //broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "marker_frame";

  //Access the fiducial_transforms Translation and Rotation  
  transformStamped.transform.translation.x = msg1->transforms[0].transform.translation.x;
  transformStamped.transform.translation.y = msg1->transforms[0].transform.translation.y;
  transformStamped.transform.translation.z = msg1->transforms[0].transform.translation.z;
  
    
  transformStamped.transform.rotation.x = msg1->transforms[0].transform.rotation.x;
  transformStamped.transform.rotation.y = msg1->transforms[0].transform.rotation.y;
  transformStamped.transform.rotation.z = msg1->transforms[0].transform.rotation.z;
  transformStamped.transform.rotation.w = msg1->transforms[0].transform.rotation.w;
  m_aruco_fiducial_id = msg1->transforms[0].fiducial_id;//Access fiducial id to store location with proper order of precedence
  
  ROS_INFO("Broadcasting");
  br.sendTransform(transformStamped);//broadcast the new frame to /tf Topic
  m_broadcasted = true;//reseting flag to stop explorer rotation and to call listener

}

void Explorer::m_broadcast_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {

 //Check if the explorer reached target and is scanning for Aruco marker
  if(m_broadcast_to_call && !msg->transforms.empty() && (msg->transforms[0].fiducial_area>750))
  {                                                                                            
    m_broadcast(msg);
  }
    
}

void Explorer::m_listen(tf2_ros::Buffer& tfBuffer) {
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));//Transform marker frame from camera frame to the map frame
        auto trans_x = transformStamped.transform.translation.x;
        auto trans_y = transformStamped.transform.translation.y;
        auto trans_z = transformStamped.transform.translation.z;

        tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        ROS_INFO_STREAM("Position in map frame: ["
        << trans_x << ","
        << trans_y << ","
        << trans_z << "]"
        );
        ROS_INFO_STREAM("Fiducial ID: " << m_aruco_fiducial_id);

        //offset the coordinates for the follower to reach
        double offset = 0.4; 
        double theta = -3.1417/2 + yaw;

        //Store the transformed target location
        m_follower_locations[m_aruco_fiducial_id][0]= trans_x + offset*cos(theta);//fiducial id acts as array index to store target locations in order 
        m_follower_locations[m_aruco_fiducial_id][1]= trans_y + offset*sin(theta);

    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

//Function to rotate the explorer once it reaches target
void Explorer::m_rotate() {
    geometry_msgs::Twist msg;
    msg.angular.z = m_angular_speed;//Set angular velocity
    m_velocity_publisher.publish(msg);//calling the publisher
}

//Function to stop explorer rotation
void Explorer::m_stop() {
    ROS_DEBUG_STREAM("Stopping robot");
    geometry_msgs::Twist msg;
    msg.angular.z = 0; //reset angular velocity to 0 
    m_velocity_publisher.publish(msg);
}