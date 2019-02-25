/*
 * extendedDMG_server.cpp
 *
 *  Created on: April 24, 2018
 *      Author: Silvia Cruciani
*/

#include <iostream>
#include "shape_analysis/extendedDMG_server.hpp"


using namespace shape_analysis;

ExtendedDMGServer::ExtendedDMGServer(){

}

ExtendedDMGServer::~ExtendedDMGServer(){

}

void ExtendedDMGServer::init(ros::NodeHandle &n){
    shape_analizer=new ExtendedDMG(n);

    std::string object_name, object_file_path, service_name;
    double finger_length, voxel_resolution, seed_resolution, color_importance, spatial_importance, normal_importance, normal_threshold;
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
    if(!n.getParam("service_name", service_name)){
        service_name="in_hand_path";
        std::cout<<"No service name specified. Using default: "<<service_name<<std::endl;
    }
    if(!n.getParam("normal_threshold", normal_threshold)){
        normal_threshold=0.07;
        std::cout<<"No service name specified. Using default: "<<normal_threshold<<std::endl;
    }

    shape_analizer->set_object_from_pointcloud(object_file_path+object_name);
    shape_analizer->set_supervoxel_parameters(voxel_resolution, seed_resolution, color_importance, spatial_importance, normal_importance, disable_transform, refinement_iterations);
    shape_analizer->set_finger_length(finger_length);
    shape_analizer->set_normal_threshold(normal_threshold);
    shape_analizer->get_supervoxels();
    shape_analizer->refine_adjacency();

    service=n.advertiseService(service_name, &ExtendedDMGServer::compute_path, this);
    ROS_INFO("Shape analysis complete. Ready to compute in hand path.");
}

void ExtendedDMGServer::spinOnce(){
    shape_analizer->spin_viewer_once();
}

bool ExtendedDMGServer::compute_path(ExtendedInHandPath::Request &req, ExtendedInHandPath::Response &res){
    shape_analizer->set_initial_contact(req.initial_grasp[0].position, req.initial_grasp[0].orientation, 0);
    shape_analizer->set_initial_contact(req.initial_grasp[1].position, req.initial_grasp[1].orientation, 1);
    //now set the desired contacts
    //std::cout<<"setting desired positions: "<<req.desired_grasp[0].position.x<<std::endl;
    shape_analizer->set_desired_contact(req.desired_grasp[0].position, req.desired_grasp[0].orientation, 0);
    shape_analizer->set_desired_contact(req.desired_grasp[1].position, req.desired_grasp[1].orientation, 1);
    //for the path computation, still implementation in the other class is needed
    try{
        shape_analizer->compute_extended_path(0); //the master/slave finger still has to be improved and detailed how to chose which is which
        shape_analizer->visualize_results();
    }catch(...){
        std::cout<<"ERROR OCCURRED"<<std::endl;
    }
    //now transform everything into the srv compatible types
    std::vector<std::vector<geometry_msgs::Point>> translations=shape_analizer->get_translation_sequence();
    res.translations_1=translations[0];
    res.translations_2=translations[1];
    res.translations_3=translations[2];
    std::vector<std::vector<double>> rotations=shape_analizer->get_angle_sequence();
    res.rotations_1=rotations[0];
    res.rotations_2=rotations[1];
    res.rotations_3=rotations[2];
    std::vector<std::vector<double>> distances=shape_analizer->get_distance_sequence();
    res.distance_variations_1=distances[0];
    res.distance_variations_2=distances[1];
    res.distance_variations_3=distances[2];
    res.master_index=0; //no change in the requested master finger   
    res.first_gripper_regrasp=shape_analizer->get_regrasp_pose(0);
    res.second_gripper_regrasp=shape_analizer->get_regrasp_pose(1);

    return true;

}

int main(int argc, char **argv){
    ros::init(argc, argv, "extendedDMG_server");
    ros::NodeHandle n("~");

    ExtendedDMGServer dmg_server=ExtendedDMGServer();
    dmg_server.init(n);
    ros::Rate r(30);

    while(ros::ok()){
        ros::spinOnce();
        dmg_server.spinOnce();
        r.sleep();
    }

    return 0;
}