/**
 *  extended_dmg.cpp
 *
 *  Created on: March 14 2018
 *      Author: Silvia Cruciani
*/

#include "shape_analysis/extended_dmg.hpp"

using namespace shape_analysis;

ExtendedDMG::ExtendedDMG(ros::NodeHandle n) : ShapeAnalyzer(){
    r_translations=std::vector<std::vector<geometry_msgs::Point>>(3);
    r_rotations=std::vector<std::vector<double>>(3);
    r_finger_distances=std::vector<std::vector<double>>(3);
    ray_tracing_service_name="ray_tracing";
    node_handle=n;
    ray_tracing_client=node_handle.serviceClient<RayTracing>(ray_tracing_service_name);
    bool active=ros::service::waitForService(ray_tracing_service_name, 1);
    if(!active){
        std::cout<<"No service named "<<ray_tracing_service_name<<" available"<<std::endl;
    }
}

ExtendedDMG::~ExtendedDMG(){

}


int ExtendedDMG::compute_extended_path(int finger_id){
    bool regrasp=ShapeAnalyzer::compute_extended_path(finger_id);
    //if there is no need for regrasp, return the path for the first (and only) contact point.
    if(!regrasp){
        r_rotations[0]=ShapeAnalyzer::get_angle_sequence();
        r_translations[0]=ShapeAnalyzer::get_translation_sequence();
        r_finger_distances[0]=ShapeAnalyzer::get_distance_sequence();
        return 0;
    }

    //now the magic happens. Check if it is possible to regrasp directly on the desired contact points:
    bool direct_regrasp_1=true;
    Eigen::Vector3f contact_point1, contact_point2, line;
    //the points are in meters, but the object's shape is in mm, so rescale:
    contact_point1=1000.0*Eigen::Vector3f(desired_pose_1(0, 0), desired_pose_1(1, 0), desired_pose_1(2, 0));
    contact_point2=1000.0*Eigen::Vector3f(desired_pose_2(0, 0), desired_pose_2(1, 0), desired_pose_2(2, 0));
    //the line between the two contacts:
    line=contact_point2-contact_point1;
    line.normalize(); //this is the direction for the two rays to intersect. It goes from contact1 to contact2

    //check if it is possible to grasp at the first contact
    Eigen::Vector3f end_point_ray=contact_point1-1000.0*line; //this point is the final point for the ray, from c1 outwards (opposite to line) of 1000 (1m)
    std::vector<Eigen::Vector3f> intersections=get_ray_intersections(contact_point1, end_point_ray);
    //if the intersections are 0, it is fine. If it is 1, it should be the contact point. (the first one is the closest to c1)
    //If it is not, or if it is more, then it is not possible to regrasp.
    if(intersections.size()>0){
        //1mm tolerance is used here if the contact point was not really on the object
        if(intersections.size()>1 || (intersections[0]-contact_point1).norm()>1.0){
            direct_regrasp_1=false;
        }
    }

    //if it is possible to regrasp with the principal finger, check with the other
    if(direct_regrasp_1){
        end_point_ray=contact_point2+1000.0*line; //this point is from c2 outwards (same direction of the line from c1 to c2)
        intersections=get_ray_intersections(contact_point2, end_point_ray);
        //same criteria as before
        if(intersections.size()>0){
            if(intersections.size()>1 || (intersections[0]-contact_point2).norm()>1.0){
                direct_regrasp_1=false;
            }
        }
    }

    //in both cases, it is assumed that the desired angle is an angle at which the gripper can grasp the object
    //TO DO: add a collision check with the desired angle given the max range of opening fingers to see if in between there is any collision 
    //with the whole finger, not just at the fingertip

    //if it is possible to directly regrasp, then we are happy and we can fill the last sequence with empty vectors
    //otherwise, we should propagate backwards in the DMG to find a proper regrasping area
    if(direct_regrasp_1){   
        r_rotations[2]=std::vector<double>();
        r_finger_distances[2]=std::vector<double>();
        r_translations[2]=std::vector<geometry_msgs::Point>();
        //the regrasp of the 1st gripper is exactly the desired grasp
        regrasp1_principal=contact_point1;
        regrasp1_secondary=contact_point2;
    }
    else{
        //for now, it is assumed that the 
        find_available_regrasping_points(contact_point1, contact_point2, true);
    }


    return 1;
}

std::pair<std::pair<Eigen::Vector3f, Eigen::Vector3f>, std::pair<Eigen::Vector3f, Eigen::Vector3f>> ExtendedDMG::get_regrasp_points(){
    std::pair<Eigen::Vector3f, Eigen::Vector3f> regrasp1=std::pair<Eigen::Vector3f, Eigen::Vector3f>(regrasp1_principal, regrasp1_secondary);
    std::pair<Eigen::Vector3f, Eigen::Vector3f> regrasp2=std::pair<Eigen::Vector3f, Eigen::Vector3f>(regrasp2_principal, regrasp2_secondary);
    //this returns first the regrasp for the second gripper and then the first (order of execution)
    return std::pair<std::pair<Eigen::Vector3f, Eigen::Vector3f>, std::pair<Eigen::Vector3f, Eigen::Vector3f>>(regrasp2, regrasp1);
}

std::vector<std::vector<geometry_msgs::Point>> ExtendedDMG::get_translation_sequence(){
    return r_translations;
}

std::vector<std::vector<double>> ExtendedDMG::get_angle_sequence(){
    return r_rotations;
}

std::vector<std::vector<double>> ExtendedDMG::get_distance_sequence(){
    return r_finger_distances;
}

void ExtendedDMG::set_ray_tracing_service_name(std::string name){
    ray_tracing_service_name=name;
    ray_tracing_client=node_handle.serviceClient<RayTracing>(ray_tracing_service_name);
    bool active=ros::service::waitForService(ray_tracing_service_name, 1);
    if(!active){
        std::cout<<"No service named "<<ray_tracing_service_name<<" available"<<std::endl;
    }
}

std::vector<Eigen::Vector3f> ExtendedDMG::get_ray_intersections(Eigen::Vector3f start, Eigen::Vector3f end){
    //rewrite the data in ros formats
    geometry_msgs::Point start_point;
    geometry_msgs::Point end_point;

    start_point.x=start(0);
    start_point.y=start(1);
    start_point.z=start(2);

    end_point.x=start(0);
    end_point.y=start(1);
    end_point.z=start(2);

    //get the mesh name from the object's name
    std::string name=object_name.substr(0, object_name.find("_"))+".stl";

    RayTracing srv;
    srv.request.mesh_name=name;
    srv.request.start_point=start_point;
    srv.request.end_point=end_point;

    //call the server
    if(ray_tracing_client.call(srv)){
        //rewrap the solution into a vector
        std::vector<Eigen::Vector3f> v;
        for(int i=0; i<srv.response.intersections.size(); i++){
            Eigen::Vector3f vec;
            vec<<srv.response.intersections[i].x, srv.response.intersections[i].y, srv.response.intersections[i].z;
            v.push_back(vec);
        }

        return v;
    }

    ROS_ERROR("Failed to call the service %s", ray_tracing_service_name.c_str());
    //assume no intersections
    return std::vector<Eigen::Vector3f>();

}

void ExtendedDMG::find_available_regrasping_points(Eigen::Vector3f principal_contact, Eigen::Vector3f secondary_contact, bool desired_angle){
    //get the grasp line first (to check for the secondary finger following the principal finger)
    Eigen::Vector3f line=secondary_contact-principal_contact;
    //get the closest node to the principal contact point
}


