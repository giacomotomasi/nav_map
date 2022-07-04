/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: June 2022                        
 * File: CollisionCheck_node.cpp                          */
 
 #include <iostream>
 #include <ros/ros.h>
 #include <nav_map/CollisionCheck.h>
 
 int main (int argc, char **argv){
     
     ros::init(argc, argv, "CollisionCheck_node");
     
     ros::NodeHandle n;
     CollisionCheck cc(&n);
     
     ros::spin();
     
     return 0;
     }