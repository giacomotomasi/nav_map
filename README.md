# nav_map
This package is thought to work with [camera3d_xyzrgb](https://github.com/giacomotomasi/camera3d_xyzrgb.git) and [lidar_xyz](https://github.com/giacomotomasi/lidar_xyz.git) packages. It takes an csv file of a known environment and creates an OccupancyGrid map. It also updates the map using the data coming from camera3d_xyzrgb or lidar_xyz packages.

Additionally it provides a node for collisions check. Given a path it return a true message if the path cells overplap any cell that represents an obstacle area in the map.



<div style="width: 10%; height: 10%">
  
  ![Excel map](https://github.com/giacomotomasi/nav_map/blob/main/img/map_excel.png) 
  
</div>


<div style="width: 50%; height: 50%">
  
  ![OccupancyGrid map](https://github.com/giacomotomasi/nav_map/blob/main/img/map_free.png) 
  
</div>


<div style="width: 50%; height: 50%">
  
  ![Update map with obstacle](https://github.com/giacomotomasi/nav_map/blob/main/img/map_obs.png)
  
</div>
