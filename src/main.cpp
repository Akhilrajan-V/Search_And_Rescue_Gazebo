/**
 * @file main.cpp
 * @author Aditya Varadaraj (varadarajaditya@gmail.com)
 * @author Akhilrajan Vethirajan (v.akhilrajan@gmail.com)
 * @author Saurabh Palande (saurabhpalande60@gmail.com)
 * @brief This is a simple Search and Rescue project developed as the final project
 * for the course ENPM 809Y at the University of Maryland, USA
 * @version 1.0
 * @date 2021-12-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include "explorer.h"
//This is for the MoveBase package usage
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//This is the main function of the code
int main(int argc, char** argv)
{
    bool explorer_goal_sent = false;
    bool follower_goal_sent = false;
    
    int i{1};
    int f{0};

    ros::init(argc, argv, "simple_navigation_goals");//Initialize a ros node
    ros::NodeHandle nh;//the ros node handle
    Explorer explorer(& nh);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate loop_rate(10);
    move_base_msgs::MoveBaseGoal explorer_goal;
    move_base_msgs::MoveBaseGoal follower_goal;
    XmlRpc::XmlRpcValue my_list;//To access and store target locations for explorer

    // tell the action client that we want to spin a thread by default
    MoveBaseClient explorer_client("/explorer/move_base", true);
    // tell the action client that we want to spin a thread by default
    MoveBaseClient follower_client("/follower/move_base", true);

    // wait for the action server to come up
    while (!explorer_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for explorer");
    }

    while (!follower_client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for follower");
    }
    
    while (ros::ok()) {
        

        if(i<=4){
            nh.getParam("aruco_lookup_locations/target_" + std::to_string(i), my_list);//obtain target locations from rosparameter server
            explorer_goal.target_pose.header.frame_id = "map";
            explorer_goal.target_pose.header.stamp = ros::Time::now();
            explorer_goal.target_pose.pose.position.x = my_list[0];//store obtained target locations in array
            explorer_goal.target_pose.pose.position.y = my_list[1];
            explorer_goal.target_pose.pose.orientation.w = 1.0;
            
            //Check if MoveBase already has a target location to work with
            if (!explorer_goal_sent)     {
                ROS_INFO("Sending goal for explorer");
                explorer_client.sendGoal(explorer_goal);
                explorer_goal_sent = true;
            }
            //Check if robot reached the target location
            if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Hooray, robot reached goal");
                explorer.m_broadcast_to_call = true;
                explorer.m_rotate();//call rotate function to detect Aruco tag
                if(explorer.m_broadcasted == true)
                {
                    explorer.m_listen(tfBuffer);//Call listener
                    explorer.m_stop();//Stop rotation
                    //reset flags for next iteration                    
                    explorer_goal_sent = false;
                    explorer.m_broadcast_to_call = false;
                    explorer.m_broadcasted=false;
                    i++;         
                }

            }

        
        }
        //After all 4 target locations are done send explorer to home position
        if(i==5 && !(explorer.m_explorer_done))
        {
            explorer_goal.target_pose.header.frame_id = "map";
            explorer_goal.target_pose.header.stamp = ros::Time::now();
            explorer_goal.target_pose.pose.position.x = -4;
            explorer_goal.target_pose.pose.position.y = 2.5;
            explorer_goal.target_pose.pose.orientation.w = 1.0;
            if (!explorer_goal_sent){
                ROS_INFO("Sending goal for explorer");
                explorer_client.sendGoal(explorer_goal);
                explorer_goal_sent = true;
            }
            if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Hooray, robot reached home position");
                explorer.m_explorer_done = true;//Shutdown explorer
            }
        }
        //Check if explorer has completed its task
        if(explorer.m_explorer_done)
        {
            explorer_client.waitForResult();//Wait for Movebase completion confirmation
            
            if(f<4)
            {
                //Build goal for follower
                follower_goal.target_pose.header.frame_id = "map";
                follower_goal.target_pose.header.stamp = ros::Time::now();
                follower_goal.target_pose.pose.position.x = explorer.m_follower_locations[f][0];//Accessing the locations found by explorer
                follower_goal.target_pose.pose.position.y = explorer.m_follower_locations[f][1];
                follower_goal.target_pose.pose.orientation.w = 1.0;
                //check if movebase already has a target location for follower
                if (!follower_goal_sent){
                    ROS_INFO_STREAM("Sending goal for follower");
                    follower_client.sendGoal(follower_goal);
                    follower_goal_sent = true;
                }
                //Check if follower reached the target
                if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO_STREAM("Hooray, follower reached goal");
                    follower_goal_sent = false;
                    f++;
                }
                
            }
            //After all the 4 locations have been visited send follower to home position
            else{
                follower_goal.target_pose.header.frame_id = "map";
                follower_goal.target_pose.header.stamp = ros::Time::now();
                follower_goal.target_pose.pose.position.x = -4;
                follower_goal.target_pose.pose.position.y = 3.5;
                follower_goal.target_pose.pose.orientation.w = 1.0;
                
                if (!follower_goal_sent){
                    ROS_INFO_STREAM("Sending goal for follower");
                    follower_client.sendGoal(follower_goal);
                    follower_goal_sent = true;
                }
                if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO_STREAM("Hooray, follower reached home position");
                    ros::shutdown();//Terminate after completion
                }
            }
        }
        ros::spinOnce();//for callback
        loop_rate.sleep();
    }

}