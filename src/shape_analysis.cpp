/*
 * shape_analysis.cpp
 *
 *  Created on: October 24, 2017
 *      Author: Silvia Cruciani
*/

#include "ros/ros.h"
#include <iostream>
#include <shape_analysis/shape_analyzer.hpp>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

using namespace shape_analysis;

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "shape_analysis");
    ros::NodeHandle n;

    ShapeAnalyzer a=ShapeAnalyzer();
    
    a.set_object_from_pointcloud("/home/silvia/catkin_ws/src/shape_analysis/shapes/shape2_high_res.pcd");
    a.set_supervoxel_parameters(1.5, 15.0, 0.0, 1.0, 1.0, true, 8); //try to find a way of defining them authomatically according to the dimension and resolution

    a.get_supervoxels();

    geometry_msgs::Point p;
    geometry_msgs::Quaternion q;
    p.x=0.0;
    p.y=-0.01;
    p.z=0.0;
    q.w=1.0;
    //a.set_initial_contact(p, q, 1);

    p.x=0.04;
    //a.set_desired_contact(p, q, 1);
    //a.compute_path(1);

    a.set_finger_length(0.04); //in meters
    a.refine_adjacency();

    a.spin_viewer();

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}