/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: June 2022                        
 * File: MapGenerator.h                          */
 
 
#ifndef _MAPGENERATOR_H_
#define _MAPGENERATOR_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>

#include "ros/ros.h"
#include<nav_map/BoundingBox3DArray.h>
#include<nav_msgs/OccupancyGrid.h>

class MapGenerator {
private:
        ros::Publisher map_pub;
        ros::Publisher obs_pub;
        ros::Subscriber obs_sub;
        ros::Subscriber path_sub;
        std::string map_file_path;
        std::string reference_frame;
        nav_msgs::OccupancyGrid::Ptr map;
        int width {};
        int height {};
        double resolution {};
        std::vector<int> init_map {};
        std::vector<int> obs_grid {};
        int safety;
        int count_id {};
public:
        void obs_callback(const nav_map::BoundingBox3DArray::ConstPtr& obs_msg);
        void get_map();
        //void obs_points(); // --> substituted by the callback
        void get_grids(std::vector<double> &x, std::vector<double> &y, std::vector<int> &grid_vec);
        void restore_map();
        void update_map(std::vector<int> &obs);
        // Constructor
        MapGenerator(ros::NodeHandle *n);
        // Destructor
        ~MapGenerator(); 
    };


#endif