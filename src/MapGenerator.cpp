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
#include <math.h>

#include "ros/ros.h"
#include <tf/tf.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Odometry.h>
#include<nav_map/BoundingBox3DArray.h>
#include <nav_map/MapGenerator.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MapGenerator::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){
    robot_x = odom_msg->pose.pose.position.x;
    robot_y = odom_msg->pose.pose.position.y;
    // convert quaternions to rpy
    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll {}, pitch {}, yaw {};
    m.getRPY(roll, pitch, yaw);
    robot_theta = yaw;
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MapGenerator::obs_callback(const nav_map::BoundingBox3DArray::ConstPtr& obs_msg){
    /*   01234567
     *  0a------b
     *  1-      -
     *  2-      -
     *  3c------d
     */
     
     obs_grid.clear();
     
     int n = obs_msg->bboxes.size();
     if (n > 0){
         for (int i {0};i<n;i++){
            double ax{0.0}, bx{0.0}, cx{0.0}, ay{0.0}, by{0.0}, cy{0.0};
            // define coordinates
            ax = obs_msg->bboxes.at(i).center.position.x - (obs_msg->bboxes.at(i).size.x/2);
            ay = obs_msg->bboxes.at(i).center.position.y + (obs_msg->bboxes.at(i).size.y/2);
            bx = obs_msg->bboxes.at(i).center.position.x + (obs_msg->bboxes.at(i).size.x/2);
            by = ay;
            cx = ax;
            cy = obs_msg->bboxes.at(i).center.position.y - (obs_msg->bboxes.at(i).size.y/2);
            // rotating obstacles
//            ax = ax*cos(robot_theta) - ay*sin(robot_theta) + robot_x;
//            ay = ax*sin(robot_theta) + ay*cos(robot_theta) + robot_y;
//            bx = bx*cos(robot_theta) - by*sin(robot_theta) + robot_x;
//            by = bx*sin(robot_theta) + by*cos(robot_theta) + robot_y;
//            cx = cx*cos(robot_theta) - cy*sin(robot_theta) + robot_x;
//            cy = cx*sin(robot_theta) + cy*cos(robot_theta) + robot_y;
            //
            std::vector<double> x{ax,bx,cx};
            std::vector<double> y{ay,by,cy};
            // find coordinates in matrix (map)

            get_grids(x, y, obs_grid);
            update_map(obs_grid); 
            }
        }else
            update_map(obs_grid);   
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MapGenerator::get_map(){
    std::fstream mapp;
    std::string line;
    // open the file
    mapp.open(map_file_path.c_str(), std::ios::in);
    // check if the file is open
    if (mapp.is_open()) {
          while (std::getline(mapp, line)){ // (!map.eof()) makes it loop once more, i.e. adding an extra line | (map >> count) skips first character, i.e. first column
              for(std::string::iterator it = line.begin(); it != line.end(); it++) {
                  if (*it != ','){
                      map->data.push_back(*it - '0'); // " - '0' " is to make correct conversion fron char to int
                      init_map.push_back(*it - '0');
                      }
                  }
              }
          mapp.close();
     } else {
         std::cerr << "Failed To Open File" << std::endl;
         }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MapGenerator::get_grids(std::vector<double> &x, std::vector<double> &y, std::vector<int> &grid){
    int n = x.size(); // or x = y.size()
    int cell_x {0}, cell_y {0};
    std::vector<std::vector<int>> obstacle;
    for (int idx=0;idx<n;idx++){
        if (x.at(idx) < 0)
            cell_x = int((x.at(idx)-0.05)/resolution)+100;
            else
                cell_x = int(x.at(idx)/resolution)+100;
                
        if (y.at(idx) < 0)
                cell_y = int((y.at(idx)-0.05)/resolution)+100;
                else
                    cell_y = int(y.at(idx)/resolution)+100;
                    
        obstacle.push_back(std::vector<int> {cell_x, cell_y});
    
        }
    for (int row=obstacle.at(2).at(1)-safety;row<obstacle.at(1).at(1)+1+safety;row++){
        for (int col=obstacle.at(0).at(0)-safety;col<obstacle.at(1).at(0)+1+safety;col++){
            int idxx {};
            idxx = col + row*width;
            grid.push_back(idxx);
            }
        }       
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MapGenerator::restore_map(){
    map->data.clear();
    for (std::vector<int>::iterator m = init_map.begin(); m!=init_map.end(); m++){
        map->data.push_back(*m);
        }
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MapGenerator::update_map(std::vector<int> &obs){
    int num = obs.size();
    
    map->header.seq = count_id++;
    map->header.stamp = ros::Time::now();
    map->header.frame_id = reference_frame;
    map->info.map_load_time = ros::Time::now();
    map->info.resolution = resolution;
    map->info.width = width;
    map->info.height = height;
    map->info.origin.position.x = ((-height*resolution)/2) + (resolution/2);
    map->info.origin.position.y = ((-width*resolution)/2) + (resolution/2);
    map->info.origin.position.z = 0;
    map->info.origin.orientation.x = 0.0;
    map->info.origin.orientation.y = 0.0;
    map->info.origin.orientation.z = 0.0;
    map->info.origin.orientation.w = 0.1;
    
    restore_map(); // restore map to initial map before adding obstacles (so you remove old ones)
    for (int aa {0};aa<num;aa++){
        int index = obs.at(aa);
        map->data.at(index) = 100;
        }
        
    map_pub.publish(map);
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
MapGenerator::MapGenerator(ros::NodeHandle *n){
    std::cout << "\033[1;32m MapGenerator constructor called.\033[0m" << std::endl; // print in green color
    // get ros parameters
    n->param<std::string>("/map/file_path",map_file_path,"/home/administrator/catkin_ws/src/nav_map/excel_map/map_new.csv");
    n->param("/map/width",width,200);
    n->param("/map/height",height,200);
    n->param("/map/resolution",resolution,0.1);
    n->param<std::string>("/map/frame_id",reference_frame,"/map");
    n->param("/map/safety_cells",safety,3);
    map = nav_msgs::OccupancyGrid::Ptr (new nav_msgs::OccupancyGrid());
    count_id = 0;
    std::vector<int> init_map {};
    std::vector<int> obs_grid {};
    robot_x = 0;
    robot_y = 0;
    robot_theta = 0;
    // create ROS Subscriber
    obs_sub = n->subscribe("/boundingBoxArray",1, &MapGenerator::obs_callback, this);
    odom_sub = n->subscribe("/odometry/filtered",1,&MapGenerator::odom_callback, this);
    // create ROS Publisher
    map_pub = n->advertise<nav_msgs::OccupancyGrid>("updated_map",1);
    get_map();
//    // publish first map
//    map_pub.publish(map);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructor
MapGenerator::~MapGenerator(){
    std::cout << "\033[1;32m MapGenerator destructor called.\033[0m" << std::endl;
    }
