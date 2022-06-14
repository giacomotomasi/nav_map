/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: June 2022                        
 * File: MapGenerator.cpp                          */
 
#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>

#include "ros/ros.h"
#include<lidar_xyz/BoundingBox3D.h>


void obs_callback(const lidar_xyz::BoundingBox3D::ConstPtr& obs_msg){
    
    }


std::vector<std::vector<int>> get_matrix(){
    
    }


void obs_points(){
    
    }


void get_grids(std::vector<double> &x, std::vector<double> &y, double &resolution, std::vector<std::vector<int>> &grid){
    
    }


void update_map(std::vector<std::vector<int>> &map_matrix, std::vector<std::vector<int>> &obs){
    
    }


// Constructor
MapGenerator(ros::nodeHandle *n){
    
    }


// Destructor
~MapGenerator(){
    
    }
