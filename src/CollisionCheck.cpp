/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: June 2022                        
 * File: CollisionCheck.cpp                          */
 
 #include <iostream>
 #include <vector>
 
 #include <ros/ros.h>
 #include <std_msgs/UInt16MultiArray.h>
 #include <std_msgs/Bool.h>
 
 #include <nav_map/CollisionCheck.h>
 
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void CollisionCheck::obs_callback(const std_msgs::UInt16MultiArray::Ptr &obs_msg){
     int n = obs_msg->data.size();
     for (int i {0}; i<n; i++){
        obs.push_back(obs_msg->data.at(i));
         }
     // check for collision
     check();
     }
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void CollisionCheck::path_callback(const std_msgs::UInt16MultiArray::Ptr &path_msg){
     int n = path_msg->data.size();
     for (int i {0}; i<n; i++){
        path.push_back(path_msg->data.at(i));
         }
     }
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void CollisionCheck::check(){
     // check for path availability
     if (path.size() == 0)
         ROS_WARN_STREAM("Path is empty. Not receiving data.");
         
     for (std::vector<int>::iterator obs_cell = obs.begin(); obs_cell != obs.end(); obs_cell++){
         for (std::vector<int>::iterator path_cell = path.begin(); path_cell != path.end(); path_cell++){
             if (*obs_cell == *path_cell){
                 collision.data = true;
                 break;
                 } else
                     collision.data = false;
             }
         }
      collision_pub.publish(collision);
    }
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // Constructor
 CollisionCheck::CollisionCheck(ros::NodeHandle *n){
     std::cout << "\033[1;32m CollisionCheck constructor called.\033[0m" << std::endl;
     // create ROS Publisher
     collision_pub = n->advertise<std_msgs::Bool>("collision", 1);
     // create ROS Subscriber
     obs_sub = n->subscribe("/obstacle_cells", 1, &CollisionCheck::obs_callback, this);
     path_sub = n->subscribe("path_cells",1, &CollisionCheck::path_callback, this);
     
     collision.data = false;
     
 }
 
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // Destructor
 CollisionCheck::~CollisionCheck(){
     std::cout << "\033[1;32m CollisionCheck destructor called.\033[0m" << std::endl;
 }
 
 
 
 