/*
 * in_hand_path_server.cpp
 *
 *  Created on: November 15, 2017
 *      Author: Silvia Cruciani
*/

#include <iostream>
#include "shape_analysis/in_hand_path_server.hpp"

using namespace shape_analysis;

InHandPathServer::InHandPathServer(){

}

InHandPathServer::~InHandPathServer(){

}

bool InHandPathServer::compute_path(InHandPath::Request &req, InHandPath::Response &res){
    std::cout<<"Call received"<<std::endl;
    //set the initial contact points
    std::cout<<"setting initial positions: "<<req.initial_grasp[0].position.x<<std::endl;
    shape_analizer.set_initial_contact(req.initial_grasp[0].position, req.initial_grasp[0].orientation, 0);
    shape_analizer.set_initial_contact(req.initial_grasp[1].position, req.initial_grasp[1].orientation, 1);
    //now set the desired contacts
    std::cout<<"setting desired positions: "<<req.desired_grasp[0].position.x<<std::endl;
    shape_analizer.set_desired_contact(req.desired_grasp[0].position, req.desired_grasp[0].orientation, 0);
    shape_analizer.set_desired_contact(req.desired_grasp[1].position, req.desired_grasp[1].orientation, 1);
    //for the path computation, still implementation in the other class is needed
    shape_analizer.compute_path(0); //the master/slave finger still has to be improved and detailed how to chose which is which
    //now transform everything into the srv compatible types
    res.translations=shape_analizer.get_translation_sequence();
    res.rotations=shape_analizer.get_angle_sequence();
    res.distance_variations=shape_analizer.get_distance_sequence();
    res.master_index=0; //no change in the requested master finger

    return true;

}

void InHandPathServer::init(ros::NodeHandle &n){
    std::string object_name, object_file_path;
    double finger_length, voxel_resolution, seed_resolution, color_importance, spatial_importance, normal_importance;
    int refinement_iterations;
    bool disable_transform;

    if (!n.getParam("object_name", object_name)){
        object_name="shape2_high_res.pcd";
        std::cout<<"No object name specified. Using default: "<<object_name<<std::endl;
    }
    if(!n.getParam("object_file_path", object_file_path)){
        object_file_path=ros::package::getPath("shape_analysis")+"/shapes/";
        std::cout<<"No file path specified. Using default: "<<object_file_path<<std::endl;
    }
    if(!n.getParam("finger_length", finger_length)){
        finger_length=0.04;
        std::cout<<"No gripper's fingers length specified. Using default: "<<finger_length<<std::endl;
    }
    if(!n.getParam("voxel_resolution", voxel_resolution)){
        voxel_resolution=1.0;
        std::cout<<"No voxel resolution specified. Using default: "<<voxel_resolution<<std::endl;
    }
    if(!n.getParam("seed_resolution", seed_resolution)){
        seed_resolution=15.0;
        std::cout<<"No seed resolution specified. Using default: "<<seed_resolution<<std::endl;
    }
    if(!n.getParam("color_importance", color_importance)){
        color_importance=0.0;
        std::cout<<"No color importance specified. Using default: "<<color_importance<<std::endl;
    }
    if(!n.getParam("spatial_importance", spatial_importance)){
        spatial_importance=1.0;
        std::cout<<"No spatial importance specified. Using default: "<<spatial_importance<<std::endl;
    }
    if(!n.getParam("normal_importance", normal_importance)){
        normal_importance=1.0;
        std::cout<<"No normal importance specified. Using default: "<<normal_importance<<std::endl;
    }
    if(!n.getParam("refinement_iterations", refinement_iterations)){
        refinement_iterations=10;
        std::cout<<"No refinement_iterations specified. Using default: "<<refinement_iterations<<std::endl;
    }
    if(!n.getParam("disable_transform", disable_transform)){
        disable_transform=true;
        std::cout<<"No disable transform specified. Using default: "<<disable_transform<<std::endl;
    }


    shape_analizer=ShapeAnalyzer();
    shape_analizer.set_object_from_pointcloud(object_file_path+object_name);
    shape_analizer.set_supervoxel_parameters(voxel_resolution, seed_resolution, color_importance, spatial_importance, normal_importance, disable_transform, refinement_iterations);
    shape_analizer.set_finger_length(finger_length);
    shape_analizer.get_supervoxels();
    shape_analizer.refine_adjacency();

    service=n.advertiseService("in_hand_path", &InHandPathServer::compute_path, this);
    ROS_INFO("Shape analysis complete. Ready to compute in hand path.");
}

void InHandPathServer::spinOnce(){
    shape_analizer.spin_viewer_once();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "in_hand_path_server");
    ros::NodeHandle n("~");

    InHandPathServer path_server=InHandPathServer();
    path_server.init(n);    
    
    ros::Rate r(30);

    while(ros::ok()){
        ros::spinOnce();
        path_server.spinOnce();
        r.sleep();
    }

    return 0;
}