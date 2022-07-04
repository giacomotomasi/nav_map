/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: June 2022                        
 * File: CollisionCheck.h                          */
 
#ifndef _COLLISIONCHECK_H_
#define _COLLISIONCHECK_H_

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Bool.h>

class CollisionCheck {
private:
    ros::Publisher collision_pub;
    ros::Subscriber obs_sub;
    ros::Subscriber path_sub;
    std_msgs::Bool collision;
    std::vector<int> obs {};
    std::vector<int> path {};

public:
    void obs_callback(const std_msgs::UInt16MultiArray::Ptr &obs_msg);
    void path_callback(const std_msgs::UInt16MultiArray::Ptr &path_msg);
    void check();
    // Constructor
    CollisionCheck(ros::NodeHandle *n);
    // Destructor
    ~CollisionCheck();
    
};

#endif