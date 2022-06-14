/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: June 2022                        
 * File: MapGenerator_node.cpp                          */
 
#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv){
    
    ros::init(argc, argv, "map_generator_node");
    
    ros::NodeHandle n;
    //MapGenerator mg(&n);
    
    ros::spin();
    
    return 0;
    }