/**
 *  extended_dmg.cpp
 *
 *  Created on: March 14 2018
 *      Author: Silvia Cruciani
*/

#include "shape_analysis/extended_dmg.hpp"
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

using namespace shape_analysis;

//helper functions 
Eigen::Matrix3f axis_angle_matrix(Eigen::Vector3f axis, double theta);
bool are_rectangles_intersecting(std::vector<Eigen::Vector3f> rectangle_1, std::vector<Eigen::Vector3f> rectangle_2);


ExtendedDMG::ExtendedDMG(ros::NodeHandle n) : ShapeAnalyzer(){
    r_translations=std::vector<std::vector<geometry_msgs::Point>>(3);
    r_rotations=std::vector<std::vector<double>>(3);
    r_finger_distances=std::vector<std::vector<double>>(3);
    ray_tracing_service_name="/ray_tracing";
    angle_collision_service_name="/collision_check";
    node_handle=n;
    regrasp1_angle=0;
    regrasp2_angle=0;

    ray_tracing_client=node_handle.serviceClient<RayTracing>(ray_tracing_service_name);
    bool active=ros::service::waitForService(ray_tracing_service_name, 1);
    if(!active){
        std::cout<<"No service named "<<ray_tracing_service_name<<" available"<<std::endl;
    }

    angle_collision_client=node_handle.serviceClient<CollisionCheck>(angle_collision_service_name);
    active=ros::service::waitForService(angle_collision_service_name, 1);
    if(!active){
        std::cout<<"No service named "<<angle_collision_service_name<<" available"<<std::endl;
    }

    regrasping_candidate_nodes.push_back(std::vector<std::pair<int, int>>()); //first gripper
    regrasping_candidate_nodes.push_back(std::vector<std::pair<int, int>>()); //second gripper

    max_fingers_opening_mm=70.0; //to be set from ros param server
    regrasp_distance_threshold=20.0; //to be set from ros param server (used to weight the regrasp area)
    min_clearence_threshold=20.0;

    //initialize the second viewer
    regrasp_viewer=new pcl::visualization::PCLVisualizer ("Regrasp Viewer");
    regrasp_viewer->setBackgroundColor(1, 1, 1);
    regrasp_viewer->initCameraParameters();

}

ExtendedDMG::~ExtendedDMG(){
    
}

void ExtendedDMG::set_object_from_pointcloud(std::string file_name){
    ShapeAnalyzer::set_object_from_pointcloud(file_name);
    regrasp_viewer->addModelFromPolyData(colorable_shape, "object_regrasp_view");
}


int ExtendedDMG::compute_extended_path(int finger_id){
    //first of all, delete the previous translation sequences
    r_translations=std::vector<std::vector<geometry_msgs::Point>>(3);
    r_rotations=std::vector<std::vector<double>>(3);
    r_finger_distances=std::vector<std::vector<double>>(3);
    bool no_regrasp=ShapeAnalyzer::compute_extended_path(finger_id);
    std::cout<<"REGRASP: "<<(!no_regrasp)<<std::endl;
    //if there is no need for regrasp, return the path for the first (and only) contact point.
    if(no_regrasp){
        std::cout<<"The desired grasp can be achieved without regrasping"<<std::endl;
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
    int desired_centroid_idx_1=get_supervoxel_index(contact_point1, false);
    int desired_centroid_idx_2=get_supervoxel_index(contact_point2, false);
    //the line between the two contacts:
    line=contact_point2-contact_point1;
    line.normalize(); //this is the direction for the two rays to intersect. It goes from contact1 to contact2

    //check if it is possible to grasp at the first contact
    Eigen::Vector3f end_point_ray=contact_point1-1000.0*line; //this point is the final point for the ray, from c1 outwards of 1000 (1m)
    std::cout<<"start point: "<<contact_point1.transpose()<<std::endl;
    std::cout<<"end point: "<<end_point_ray.transpose()<<std::endl;
    std::vector<Eigen::Vector3f> intersections=get_ray_intersections(contact_point1, end_point_ray);
    std::cout<<"number of intersections 1 found: "<<intersections.size()<<std::endl;
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
        end_point_ray=contact_point2+1000.0*line; //this point is from c2 outwards
        intersections=get_ray_intersections(contact_point2, end_point_ray);
        std::cout<<"start point: "<<contact_point2.transpose()<<std::endl;
        std::cout<<"end point: "<<end_point_ray.transpose()<<std::endl;
        std::cout<<"number of intersections 2 found: "<<intersections.size()<<std::endl;

        //same criteria as before
        if(intersections.size()>0){
            if(intersections.size()>1 || (intersections[0]-contact_point2).norm()>1.0){
                direct_regrasp_1=false;
            }
        }
    }

    std::cout<<"DIRECT REGRASP: "<<direct_regrasp_1<<std::endl;

    //in both cases, it is assumed that the desired angle is an angle at which the gripper can grasp the object
    //with the whole finger, not just at the fingertip
    bool angle_in_collision=false;

    //get the connected component of the desired contact
    int initial_contact_cc;
    Eigen::Quaternion<float> q;
    if(finger_id==1){
        initial_contact_cc=nodes_to_connected_component.at(initial_centroid_idx_1);
        q=Eigen::Quaternion<float>(initial_pose_1(6), initial_pose_1(3), initial_pose_1(4), initial_pose_1(5));
    }
    else{
        initial_contact_cc=nodes_to_connected_component.at(initial_centroid_idx_2);
        q=Eigen::Quaternion<float>(initial_pose_2(6), initial_pose_2(3), initial_pose_2(4), initial_pose_2(5));
    }

    Eigen::Quaternion<float> q_des;
    int desired_contact_cc;
    if(finger_id==1){
        desired_contact_cc=nodes_to_connected_component.at(desired_centroid_idx_1);
        q_des=Eigen::Quaternion<float>(desired_pose_1(6), desired_pose_1(3), desired_pose_1(4), desired_pose_1(5));
    }
    else{
        desired_contact_cc=nodes_to_connected_component.at(desired_centroid_idx_2);
        q_des=Eigen::Quaternion<float>(desired_pose_2(6), desired_pose_2(3), desired_pose_2(4), desired_pose_2(5));
    }

    std::cout<<"checking principal desired angle "<<std::endl;
    std::cout<<"PRINCIPAL pose to angle: "<<q_des.x()<<" "<<q_des.y()<<" "<<q_des.z()<<" "<<q_des.w()<<std::endl;
    std::cout<<"PRINCIPAL CC: "<<desired_contact_cc<<std::endl;
    int principal_desired_angle=pose_to_angle(q_des, desired_contact_cc);
    std::cout<<"PRINCIPAL angle: "<<principal_desired_angle<<std::endl;

    //get also the angle of the secondary finger at the desired pose
    if(finger_id==1){
        desired_contact_cc=nodes_to_connected_component.at(desired_centroid_idx_2);
        q_des=Eigen::Quaternion<float>(desired_pose_2(6), desired_pose_2(3), desired_pose_2(4), desired_pose_2(5));
    }
    else{
        desired_contact_cc=nodes_to_connected_component.at(desired_centroid_idx_1);
        q_des=Eigen::Quaternion<float>(desired_pose_1(6), desired_pose_1(3), desired_pose_1(4), desired_pose_1(5));
    }
    std::cout<<"checking secondary desired angle "<<std::endl;
    std::cout<<"SECONDARY pose to angle: "<<" "<<q_des.x()<<" "<<q_des.y()<<" "<<q_des.z()<<" "<<q_des.w()<<std::endl;
    std::cout<<"SECONDARY CC: "<<desired_contact_cc<<std::endl;
    int secondary_desired_angle=pose_to_angle(q_des, desired_contact_cc);
    std::cout<<"SECONDARY angle: "<<secondary_desired_angle<<std::endl;

    //to be able to directly regrasp, the desired angle should be free of collisions 

    std::cout<<"checking if the desired angle is in collision"<<std::endl;
    //collision check with the desired angle: check if the desired angle is reachable
    angle_in_collision=is_in_collision(contact_point1, double(principal_desired_angle)*M_PI/180.0);
    std::cout<<"is in collision: "<<angle_in_collision<<std::endl;
    if (!angle_in_collision){
        //check if the slave finger is also not in collision
        std::cout<<"checking if secondary angle is also not in collisiton"<<std::endl;
        angle_in_collision=is_in_collision(contact_point2, double(secondary_desired_angle)*M_PI/180.0);
        std::cout<<"is in collision: "<<angle_in_collision<<std::endl;

    }


    //if it is possible to directly regrasp, then we are happy and we can fill the last sequence with empty vectors
    //otherwise, we should propagate backwards in the DMG to find a proper regrasping area
    int int_regrasp1_angle=-1;
    if(direct_regrasp_1&& !angle_in_collision){
        //this is the best case
        r_rotations[2]=std::vector<double>();
        r_finger_distances[2]=std::vector<double>();
        r_translations[2]=std::vector<geometry_msgs::Point>();
        //the regrasp of the 1st gripper is exactly the desired grasp
        regrasp1_principal=contact_point1;
        std::cout<<"DIRECT REGRASP: "<<regrasp1_principal.transpose();
        regrasp1_secondary=contact_point2;
        int_regrasp1_angle=principal_desired_angle;
        //add it to the list
        int regrasp1_node=get_supervoxel_index(regrasp1_principal, false);
        std::cout<<"   node: "<<regrasp1_node<<std::endl;
        int ang_component=node_angle_to_angle_component.at(std::pair<int, int>(regrasp1_node, int_regrasp1_angle));
        regrasping_candidate_nodes[0].push_back(std::pair<int, int>(regrasp1_node, ang_component));
    }
    else{
        std::cout<<"MORE WORK MUST BE DONE! the 1st gripper cannot directly regrasp!"<<std::endl;
        //call this for the first gripper
        find_available_regrasping_points(contact_point1, contact_point2, principal_desired_angle, secondary_desired_angle, 0);

        if(regrasping_candidate_nodes.size()<1){
            std::cout<<"could not find suitable regrasping points! :("<<std::endl;
            return -1;
        }
        std::cout<<"Found available regrasping points for the 1st gripper!"<<std::endl;
        std::cout<<"candidate node[0][0]: "<<regrasping_candidate_nodes[0][0].first<<std::endl;

        //the first gripper must have a determined regrasp point, and so does the second gripper! for now it is only an area......

        //for now use the first node available among the regrasp candidates 
        //TODO: use all the candidates when it comes to find the 2nd gripper's regrasp
        pcl::PointXYZRGBA p=all_centroids_cloud->at(supervoxel_to_pc_idx.at(regrasping_candidate_nodes[0][0].first));
        regrasp1_principal<<p.x, p.y, p.z;
        std::cout<<"regrasp1_principal: "<<regrasp1_principal.transpose()<<std::endl;
        //check if the desired angle can be achieved at the regrasp point
        if(possible_angles.at(regrasping_candidate_nodes[0][0].first).find(principal_desired_angle)==possible_angles.at(regrasping_candidate_nodes[0][0].first).end()){
            //get the first available angle: (TO BE IMPROVED)
            std::cout<<"The desired angle is not achievable in the selected regrasp point"<<std::endl;
            int_regrasp1_angle=*(possible_angles.at(regrasping_candidate_nodes[0][0].first).begin());
        }
        else{
            std::cout<<"setting the regrasp angle to the desired angle"<<std::endl;
            int_regrasp1_angle=principal_desired_angle;
            std::cout<<"REGRASP ANGLE: "<<int_regrasp1_angle<<std::endl;
        }
        
        //get the opposite component
        Eigen::Vector3f regrasp_line=contact_point2-contact_point1; //direction from principal to secondary finger
        end_point_ray=regrasp1_principal+1000.0*regrasp_line;
        intersections=get_ray_intersections(regrasp1_principal, end_point_ray);
        //get the intersection furthest away from the point
        if(intersections.size()<1){
            std::cout<<"error in the secondary contact of regrasping point: no intersections."<<std::endl;
            return 0;
        }
        regrasp1_secondary=intersections[intersections.size()-1];

    }


    std::cout<<std::endl<<"going to the second gripper processing..."<<std::endl;
    std::cout<<"angles: "<<principal_desired_angle<<"    "<<secondary_desired_angle<<std::endl;    
    

    //now we have a regrasping area (or points) for the first gripper. We have to obtain one for the second gripper.
    //We assign to each node in the graph a weight wich depends on the sum of the distances from the release grasp and the 1st regrasp of the
    //first gripper
    
    //let's assume for now that the release contacts are the initial contacts
    Eigen::Vector3f release_contact_1, release_contact_2; //these two are kept in mm
    release_contact_1=1000.0*initial_pose_1.block<3,1>(0, 0);
    release_contact_2=1000.0*initial_pose_2.block<3,1>(0, 0);
    Eigen::Quaternion<float> quat(initial_pose_1(6), initial_pose_1(3), initial_pose_1(4), initial_pose_1(5));
    int release_angle=pose_to_angle(quat, nodes_to_connected_component.at(get_supervoxel_index(release_contact_1, false)));



    //we also must check if it is possible to release the grasp at the initial location, or if additional motions are required
    //This is a TODO for later, because for now one can assume that the object has just been grasped there, so release is always possible
    //it can be generalized "very easily" though
    std::cout<<"weighting regrasp area"<<std::endl;

    //now, we loop for all the possible contact points until we find a valid solution!
    bool valid_configuration=false;
    //get the extended target nodes (the final goal)
    int ang_component1=node_angle_to_angle_component.at(std::pair<int, int>(get_supervoxel_index(contact_point1, false), principal_desired_angle));
    std::pair<int, int> regrasp_node_goal1=std::pair<int, int>(get_supervoxel_index(contact_point1, false), ang_component1);
    int ang_component2=node_angle_to_angle_component.at(std::pair<int, int>(get_supervoxel_index(contact_point2, false), secondary_desired_angle));
    std::pair<int, int> regrasp_node_goal2=std::pair<int, int>(get_supervoxel_index(contact_point2, false), ang_component2);

    std::set<std::pair<int, int>> already_visited_nodes;

    for(int idx=0; idx<regrasping_candidate_nodes[0].size() && !valid_configuration; idx++){
        std::cout<<"Iteration: "<<idx<<std::endl;
        std::cout<<"current regrasping candidate: "<<regrasping_candidate_nodes[0][idx].first<<regrasping_candidate_nodes[0][idx].second<<std::endl<<std::endl;
        already_visited_nodes.insert(regrasping_candidate_nodes[0][idx]);
        //current values:
        pcl::PointXYZRGBA p=all_centroids_cloud->at(supervoxel_to_pc_idx.at(regrasping_candidate_nodes[0][idx].first));
        regrasp1_principal<<p.x, p.y, p.z;

        std::cout<<"CURRENT NODE: "<<regrasping_candidate_nodes[0][idx].first<<"   i.e.   "<<regrasp1_principal.transpose()<<std::endl;
        //get the opposite component
        Eigen::Vector3f regrasp_line=contact_point2-contact_point1; //direction from principal to secondary finger
        std::cout<<"regrasp line: "<<regrasp_line.transpose()<<std::endl;
        end_point_ray=regrasp1_principal+1000.0*regrasp_line;
        std::cout<<"end point ray: "<<end_point_ray.transpose()<<std::endl;
        intersections=get_ray_intersections(regrasp1_principal, end_point_ray);
        //get the intersection furthest away from the point
        if(intersections.size()<1){
            std::cout<<"error in the secondary contact of regrasping point: no intersections."<<std::endl;
            return 0;
        }
        regrasp1_secondary=intersections[intersections.size()-1];
        std::cout<<"regrasp1_secondary: "<<regrasp1_secondary.transpose()<<std::endl;
        //now see if the desired angle is ok there or not:
        std::set<int> available_angles=possible_angles.at(regrasping_candidate_nodes[0][idx].first);
        if(available_angles.find(principal_desired_angle)!=available_angles.end()){
            int_regrasp1_angle=principal_desired_angle;
        }
        else{
            //get a random angle for now
            *(possible_angles.at(regrasping_candidate_nodes[0][idx].first).begin());
        }     


        //second gripper regrasping area:
        nodes_distances_from_regrasps=weight_regrasping_area(release_contact_1, release_contact_2, regrasp1_principal, regrasp1_secondary);
        std::cout<<"--done!"<<std::endl;

        //check if there is at least one possible pose! If not, movements of the fingers must be considered (TODO). For now, return -1
        if(regrasp_poses_distance_values.size()<1){
            std::cout<<"No valid regrasp poses are directly achievable. More in-hand motions are required."<<std::endl;
            //basically here I should give the 1st gripper a different set of regrasp poses (the first neighbors for instance)
            return -1;
        }
        //now get the regrasp pose with max value (las element in the map)
        //TODO: try more than one possible value?
        std::map<std::pair<int, int>, double>::reverse_iterator r_it=regrasp_poses_distance_values.rbegin();

        //for now just get the position, not the orientation
        regrasp2_node1=r_it->first.first;
        regrasp2_node2=r_it->first.second;
        double maxit=r_it->second;
        //TODO find a more efficient way of sorting (e.g. swap keys and values in the map)
        for(; r_it!=regrasp_poses_distance_values.rend(); r_it++){
            if(r_it->second>maxit){
                maxit=r_it->second;
                regrasp2_node1=r_it->first.first;
                regrasp2_node2=r_it->first.second;
            }
        }
        std::cout<<std::endl<<"***** current max value: "<<maxit<<std::endl;
        std::cout<<"********* the regrasp nodes are: "<<regrasp2_node1<<" "<<regrasp2_node2<<std::endl<<std::endl;

        //store the values in eigen vectors:
        regrasp2_principal(0)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node1]).x;
        regrasp2_principal(1)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node1]).y;
        regrasp2_principal(2)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node1]).z;

        regrasp2_secondary(0)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node2]).x;
        regrasp2_secondary(1)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node2]).y;
        regrasp2_secondary(2)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node2]).z;

        int free_regrasp_angle=get_collision_free_regrasp_angle(regrasp2_principal, regrasp2_secondary, release_contact_1, release_contact_2, release_angle, regrasp1_principal, regrasp1_secondary, int_regrasp1_angle);
        if(free_regrasp_angle>=0){
            std::cout<<"WE ARE VERY HAPPY BECAUSE EVERYTHING WORKS NICELY"<<std::endl;
            std::cout<<"fist gripper regrasp angle: "<<int_regrasp1_angle<<std::endl;
            //there is a good collision-free angle!
            regrasp2_angle=M_PI*float(free_regrasp_angle)/180.0;
            regrasp1_angle=M_PI*float(int_regrasp1_angle)/180;
            std::cout<<"second gripper regrasp angle: "<<regrasp2_angle<<std::endl;
            valid_configuration=true;
            //if the modified angle is not the desired one, this must be considered for later in-hand manipulation planning
            if(abs(int_regrasp1_angle - principal_desired_angle)>=angle_jump){
                direct_regrasp_1=false;

            }
        }
        else{
            // //loop over all the possible angles (this could take time)
            // for(std::set<int>::iterator it=node_component_to_angles_subset.at(regrasping_candidate_nodes[0][idx]).begin(); it!=possible_angles.at(regrasping_candidate_nodes[0][idx].first).end(); it++){
            //     int_regrasp1_angle=*it;
            //     std::cout<<" OVERRIDING FIRST REGRASP ANGLE TO: "<<int_regrasp1_angle<<std::endl;
            //     regrasp1_angle=M_PI*float(int_regrasp1_angle)/180;

            //     //check if in this regrasp point the gripper is free of collisions
            //     int free_regrasp_angle=get_collision_free_regrasp_angle(regrasp2_principal, regrasp2_secondary, release_contact_1, release_contact_2, release_angle, regrasp1_principal, regrasp1_secondary, int_regrasp1_angle);
            //     if(free_regrasp_angle>=0){
            //         //there is a good collision-free angle!
            //         regrasp2_angle=M_PI*float(free_regrasp_angle/180.0);
            //         std::cout<<"second gripper regrasp angle: "<<regrasp2_angle<<std::endl;
            //         valid_configuration=true;
            //         break;
            //     }
            //     else{
            //         //A lot of TODOS
            //         //for now skip
            //         //std::cout<<"The candidate helper regrasp is not free of collisions. More in-hand motions are required"<<std::endl;
            //         // 1) move the 1st gripper regrasp angle (can be done in big intervals or randomly...)
            //     }
            // }
            //here, start making a different in-hand path for the 1st gripper: get all the neighbours (that possibly have the same regrasping angle)
            // int max_it_new_path=100;
            // int num_it_path=0;
            // while(!valid_configuration && num_it_path<max_it_new_path){
            // num_it_path++;
            std::pair<int, int> current_node=regrasping_candidate_nodes[0][0];
            //basically we add possible candidates to the candidate nodes to loop through
            std::multimap<std::pair<int, int>, std::pair<int, int>>::iterator adjacent_itr=extended_refined_adjacency.equal_range(current_node).first;
            for(; adjacent_itr!=extended_refined_adjacency.equal_range(current_node).second; adjacent_itr++){
                std::pair<int, int> neighbour=adjacent_itr->second;
                std::set<int> neighbour_possible_angles=node_component_to_angles_subset.at(neighbour);
                // if(find(neighbour_possible_angles.begin(); neighbour_possible_angles.end(), int_regrasp1_angle)!=neighbour_possible_angles.end()){
                    //then the regrasp angle is fine in the new node (in theory I should check if is graspable there. TO DO)
                if(already_visited_nodes.find(adjacent_itr->second)==already_visited_nodes.end()){
                    regrasping_candidate_nodes[0].push_back(adjacent_itr->second);
                    std::cout<<"adding: "<<adjacent_itr->second.first<<" "<<adjacent_itr->second.second<<std::endl;
                    direct_regrasp_1=false;
                }
                std::cout<<std::endl<<"================================="<<std::endl<<std::endl<<"adding more candidates"<<std::endl;
                std::cout<<"regrasp candidate nodes size: "<<regrasping_candidate_nodes[0].size()<<std::endl;
                // }

            }
            // }
        }

    }

    //if after all this, no success, oh well...
    if(!valid_configuration){
        return -1;
    }

    std::cout<<"Almost all is done... only final refinements! "<<std::endl;
    std::cout<<"regrasp2_secondary: "<<regrasp1_secondary.transpose()<<std::endl;

    // get the path from of the regrasping thing. Only if it is not direct regrasp
    if(!direct_regrasp_1||angle_in_collision){
        std::cout<<"computing the path to be executed after regrasp!"<<std::endl;
        int regrasp1_secondary_node=get_supervoxel_index(regrasp1_secondary, false);
        std::cout<<"11111111"<<std::endl;

        int regrasp1_angle_secondary= pose_to_angle(angle_to_pose(M_PI*float(int_regrasp1_angle)/180.0, regrasp1_principal), nodes_to_connected_component.at(regrasp1_secondary_node));
        std::cout<<"222222"<<std::endl;
        std::cout<<regrasp1_angle_secondary<<std::endl;
        
        if(node_angle_to_angle_component.count(std::pair<int, int>(regrasp1_secondary_node, regrasp1_angle_secondary))<1){
            std::cout<<"No valid regrasp angle secondary"<<std::endl;
        }
        int regrasp1_angle_component_secondary=node_angle_to_angle_component.at(std::pair<int, int>(regrasp1_secondary_node, regrasp1_angle_secondary));
        std::cout<<"3333333"<<std::endl;
        
        std::pair<std::stack<std::pair<int, int>>, std::stack<int>> regrasp_path=ShapeAnalyzer::get_extended_path(extended_refined_adjacency, get_supervoxel_index(regrasp1_principal, false), regrasp_node_goal1.first, contact_point2-contact_point1, std::pair<int, int>(regrasp1_secondary_node, regrasp1_angle_component_secondary), regrasp_node_goal2, int_regrasp1_angle, principal_desired_angle);
        std::cout<<"here is fine"<<std::endl;
        //put this into a vector for the sequence!
        std::pair<int, int> idx;
        int pc_idx;
        std::vector<std::pair<int, int>> vector_regrasp_path;
        pcl::PointXYZRGBA point;
        while(regrasp_path.first.size()>0){
            idx=regrasp_path.first.top();
            pc_idx=regrasp_path.second.top();
            regrasp_path.first.pop();
            regrasp_path.second.pop();
            vector_regrasp_path.push_back(idx);
            point=supervoxel_clusters.at(pc_idx)->centroid_;
            //put this into the final geometry msg, scales
            geometry_msgs::Point p;
            p.x=point.x/1000.0;
            p.y=point.y/1000.0;
            p.z=point.z/1000.0;
            r_translations[2].push_back(p);
        }
        compute_extended_angle_sequence(vector_regrasp_path, finger_id, int_regrasp1_angle, principal_desired_angle);
        r_rotations[2]=angle_sequence;

        //for now do not care about the distances
    }

    std::cout<<"until here, the regrasp1_angle is: "<<int_regrasp1_angle<<" in deg: "<<regrasp1_angle<<std::endl;


    

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

void ExtendedDMG::set_angle_collision_service_name(std::string name){
    angle_collision_service_name=name;
    angle_collision_client=node_handle.serviceClient<CollisionCheck>(angle_collision_service_name);
    bool active=ros::service::waitForService(angle_collision_service_name, 1);
    if(!active){
        std::cout<<"No service named "<<angle_collision_service_name<<" available"<<std::endl;
    }
}

std::vector<Eigen::Vector3f> ExtendedDMG::get_ray_intersections(Eigen::Vector3f start, Eigen::Vector3f end){
    //std::cout<<"calling ray tracing server..."<<std::endl;
    //rewrite the data in ros formats
    geometry_msgs::Point start_point;
    geometry_msgs::Point end_point;

    start_point.x=start(0);
    start_point.y=start(1);
    start_point.z=start(2);

    end_point.x=end(0);
    end_point.y=end(1);
    end_point.z=end(2);

    //get the mesh name from the object's name
    std::string name=object_name.substr(0, object_name.size()-13)+".stl";
    //std::cout<<"object_name: "<<name<<std::endl;

    RayTracing srv;
    srv.request.mesh_name=name;
    srv.request.start_point=start_point;
    srv.request.end_point=end_point;

    //call the server
    if(ray_tracing_client.call(srv)){
        //std::cout<<"success!"<<std::endl;
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

bool ExtendedDMG::is_in_collision(Eigen::Vector3f contact, double angle){
    int int_angle=filter_angle(angle);
    int supervoxel_idx=get_supervoxel_index(contact, false);
    //check if this angle is good
    std::vector<int> angle_components=node_to_angle_components.at(supervoxel_idx);
    for(int i=0; i<angle_components.size(); i++){
        std::set<int> valid_angles=node_component_to_angles_subset.at(std::pair<int, int>(supervoxel_idx, angle_components[i]));
        if(std::find(valid_angles.begin(), valid_angles.end(), int_angle)!=valid_angles.end()){

        //the angle is in the list of good angles, so it is not in collision!
        return false;
        }
    }
    std::cout<<"++++++++++++++ yup, collision"<<std::endl;
    return true;
}

void ExtendedDMG::find_available_regrasping_points(Eigen::Vector3f principal_contact, Eigen::Vector3f secondary_contact, int principal_angle, int secondary_angle, int gripper_id){
    //get the grasp line first (to check for the secondary finger following the principal finger)
    // std::cout<<"here in the processing: 1"<<std::endl;
    Eigen::Vector3f line=secondary_contact-principal_contact;

    //get the closest node to the principal contact point
    int principal_supervoxel_idx=get_supervoxel_index(principal_contact, false);
    //get the index of the angle component
    std::cout<<"principal angle: "<<principal_angle<<std::endl;
    std::cout<<"secondary angle: "<<secondary_angle<<std::endl;
    std::cout<<"supervoxel_idx: "<<principal_supervoxel_idx<<std::endl;
    std::cout<<"Angles here: ";
    try{
        for (auto a: possible_angles.at(principal_supervoxel_idx)){
            std::cout<<a<<"  ";
        }
    }
    catch(std::out_of_range& e){
        std::cout<<"Missing possible angles at: "<<principal_supervoxel_idx<<std::endl;
    }

    std::cout<<std::endl;
    // std::cout<<"here in the processing: 1"<<std::endl;

    int principal_angle_component=node_angle_to_angle_component.at(std::pair<int, int>(principal_supervoxel_idx, principal_angle));
    // std::cout<<"here in the processing: 2"<<std::endl;
    // std::cout<<"getting ext comp of: "<<principal_supervoxel_idx<<"  "<<principal_angle_component<<std::endl;

    //get the extended connected component
    int principal_cc=extended_nodes_to_connected_component.at(std::pair<int, int>(principal_supervoxel_idx, principal_angle_component));
    // std::cout<<"here in the processing: 3"<<std::endl;
    // std::cout<<"got component: "<<principal_cc<<std::endl;

    //do the same to get the secondary connected component
    int secondary_supervoxel_idx=get_supervoxel_index(secondary_contact, false);
    // std::cout<<"here in the processing: 3.5"<<std::endl;
    // std::cout<<"secondary_supervoxel_idx: "<<secondary_supervoxel_idx<<std::endl;
    // std::cout<<"secondary node angles: ";
    try{
        for (auto a: possible_angles.at(secondary_supervoxel_idx)){
            std::cout<<a<<"  ";
        }
    }
    catch(std::out_of_range& e){
        std::cout<<"Missing possible angles at: "<<principal_supervoxel_idx<<std::endl;
    }
    std::cout<<std::endl;

    int secondary_angle_component=node_angle_to_angle_component.at(std::pair<int, int>(secondary_supervoxel_idx, secondary_angle));
    // std::cout<<"here in the processing: 3.5.5"<<std::endl;

    int secondary_cc=extended_nodes_to_connected_component.at(std::pair<int, int>(secondary_supervoxel_idx, secondary_angle_component));
    // std::cout<<"here in the processing: 4"<<std::endl;


    //now to a BFS starting from the given contact, until we find points from which it is possible to regrasp
    std::queue<std::pair<int, int>> Q;
    std::set<std::pair<int, int>> visited_nodes;
    std::pair<int, int> n_init=std::pair<int, int>(principal_supervoxel_idx, principal_angle_component);
    Q.push(n_init);
    visited_nodes.insert(n_init);
    // std::cout<<"here in the processing: 5"<<std::endl;

    while(Q.size()>0){
        std::pair<int, int> n=Q.front();
        Q.pop(); //remove the first element, that is now n
        //visit all the children (neighbor of n)
        std::multimap<std::pair<int, int>, std::pair<int, int>>::iterator label_itr=extended_refined_adjacency.equal_range(n).first;
        // std::cout<<"here in the processing: 6"<<std::endl;
        for ( ; label_itr!=extended_refined_adjacency.equal_range(n).second; label_itr++) {
            std::pair<int, int> child=label_itr->second;
            // std::cout<<"child: "<<child.first<<" "<<child.second<<std::endl<<std::endl;
            //check if this child has already been explored
            if(visited_nodes.find(child)==visited_nodes.end()){
                visited_nodes.insert(child);
                // std::cout<<"visited nodes: "<<visited_nodes.size()<<std::endl;
                // std::cout<<"Q: "<<Q.size()<<std::endl;
                //now add this to the queue only if the secondary finger there can be in a valid configuration
                //this can also be added only up to a certain distance from the contact, to keep the regrasping area limited (when possible)
                // std::cout<<"here in the processing: 7"<<std::endl;
                // std::cout<<"current node: "<<n.first<<" "<<n.second<<std::endl;
                std::vector<std::pair<int, int>> secondary_nodes=get_opposite_finger_nodes(line, child);
                // std::cout<<"node -- "<<secondary_nodes[0].first<<" "<<secondary_nodes[0].second<<std::endl;

                for(std::pair<int, int> sec_n: secondary_nodes){
                    //check if this is in the correct connected component
                    // std::cout<<"node: "<<sec_n.first<<" "<<sec_n.second<<std::endl;

                    int node_cc=extended_nodes_to_connected_component.at(sec_n);
                    // std::cout<<"here in the processing: 8"<<std::endl;

                    if(node_cc==secondary_cc){
                        // std::cout<<"here!!!!"<<std::endl;
                        Q.push(child);
                        //check if child is good for regrasping, and if yes add it to a set of candidates to define the regraspable area
                        Eigen::Vector3f contact_point1;
                        contact_point1<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(child.first)).x,
                                all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(child.first)).y,
                                all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(child.first)).z;
                        //check if possible to grasp at the first contact
                        Eigen::Vector3f end_point_ray=contact_point1-1000.0*line;
                        std::vector<Eigen::Vector3f> intersections=get_ray_intersections(contact_point1, end_point_ray);
                        //this is good if there is 0 or 1 intersection and the one is the contact point
                        bool second_check=false;
                        bool add_point=false;
                        if(intersections.size()<1){
                            second_check=true;
                        }
                        else if(intersections.size()==1 && (intersections[0]-contact_point1).norm()<1.0){
                            second_check=true;
                        }
                        //if the first contact is graspable, check the second!
                        if(second_check){
                            Eigen::Vector3f contact_point2;
                            contact_point2<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(sec_n.first)).x,
                                    all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(sec_n.first)).y,
                                    all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(sec_n.first)).z;
                            end_point_ray=contact_point2+1000.0*line;
                            intersections=get_ray_intersections(contact_point2, end_point_ray);
                            //if here also there are 0 or 1 which is the second contact point, then it is good
                            if(intersections.size()<1){
                                add_point=true;
                            }
                            else if(intersections.size()==1 && (intersections[0]-contact_point2).norm()<1.0){
                                add_point=true;
                            }
                        }

                        //now, if this child is valid, we can add it to the regrasp area of the corresponding gripper.
                        if(add_point){
                            // std::cout<<"+++++ adding point: "<<contact_point1.transpose()<<std::endl;
                            regrasping_candidate_nodes[gripper_id].push_back(child);
                        }
                    }
                }
            }
            // std::cout<<"looping..."<<std::endl;
        }
    }
    std::cout<<"LOOP ENDED"<<std::endl<<std::endl;
}

int ExtendedDMG::get_supervoxel_index(Eigen::Vector3f contact, bool scale){
    //put the contact in pcl point
    pcl::PointXYZRGBA input;
    if(scale){
        input.x=contact(0)*1000.0;
        input.y=contact(1)*1000.0;
        input.z=contact(2)*1000.0;
    }
    else{
        input.x=contact(0);
        input.y=contact(1);
        input.z=contact(2);
    }

    //now start searching for the nearest neighbor
    int K=3;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    double min_dist=DBL_MAX;
    int min_dist_idx=-1;
    if (centroids_kdtree.nearestKSearch(input, K, pointIdxNKNSearch, pointNKNSquaredDistance)> 0){
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
            // std::cout<<" possible point: "<<pointIdxNKNSearch[i]<<" which is: "<<pc_to_supervoxel_idx.at(pointIdxNKNSearch[i])<<std::endl;
            // std::cout<<"    "<<all_centroids_cloud->at(pointIdxNKNSearch[i]).x<<" "<<all_centroids_cloud->at(pointIdxNKNSearch[i]).y<<" "<<all_centroids_cloud->at(pointIdxNKNSearch[i]).z<<std::endl;
            // std::cout<<"distance fom the input: "<<pointNKNSquaredDistance[i]<<std::endl;
            //manually check the distance because there's something wrong?
            // double node_distance=(contact(0)*1000.0-all_centroids_cloud->at(pointIdxNKNSearch[i]).x)*(contact(0)*1000.0-all_centroids_cloud->at(pointIdxNKNSearch[i]).x)+(contact(1)*1000.0-all_centroids_cloud->at(pointIdxNKNSearch[i]).y)*(contact(1)*1000.0-all_centroids_cloud->at(pointIdxNKNSearch[i]).y)+(contact(2)*1000.0-all_centroids_cloud->at(pointIdxNKNSearch[i]).z)*(contact(2)*1000.0-all_centroids_cloud->at(pointIdxNKNSearch[i]).z);
            // std::cout<<"custom distance: "<<node_distance<<std::endl;
            // std::cout<<contact(0)*1000.0-all_centroids_cloud->at(pointIdxNKNSearch[i]).x<<" "<<contact(1)*1000.0-all_centroids_cloud->at(pointIdxNKNSearch[i]).y<<" "<<contact(2)*1000.0-all_centroids_cloud->at(pointIdxNKNSearch[i]).z<<std::endl;
            if(pointNKNSquaredDistance[i]<min_dist){
                min_dist=pointNKNSquaredDistance[i];
                min_dist_idx=i;
            }
        }
        // std::cout<<" the closest supervoxel: "<<all_centroids_cloud->at(pointIdxNKNSearch[min_dist_idx]).x<<" "<<all_centroids_cloud->at(pointIdxNKNSearch[min_dist_idx]).y<<" "<<all_centroids_cloud->at(pointIdxNKNSearch[min_dist_idx]).z<<std::endl;
        // std::cout<<"given: "<<contact.transpose()<<"result: "<<pointIdxNKNSearch[min_dist_idx]<<" which is: "<<pc_to_supervoxel_idx.at(pointIdxNKNSearch[min_dist_idx])<<std::endl;
        return int(pc_to_supervoxel_idx.at(pointIdxNKNSearch[min_dist_idx])); //the first and only found index is the nearest neighbor
    }
    else{
        std::cout<<"No neighbour found."<<std::endl;
        return -1;
    }
}

Eigen::Vector3f ExtendedDMG::get_normal_at_contact(Eigen::Vector3f contact){
    //get the index of the supervoxel
    int idx=get_supervoxel_index(contact, false);
    // std::cout<<"supervoxel index used: "<<idx<<std::endl;
    //get the normal of that component
    // std::cout<<"component used for getting normals: "<<nodes_to_connected_component.at(idx)<<std::endl;
    Eigen::Vector3f normal=component_to_average_normal.at(nodes_to_connected_component.at(idx));
    // std::cout<<"which gives: "<<normal.transpose()<<std::endl;
    return normal;
}

Eigen::Vector3f ExtendedDMG::get_zero_angle_direction(Eigen::Vector3f contact){
    Eigen::Vector3f nx=component_to_average_normal.at(nodes_to_connected_component.at(get_supervoxel_index(contact, false)));
    Eigen::Vector3f ny=get_orthogonal_axis(nx); //ny is the axis at which the angle is 0, for all the nodes
    return ny;
}

Eigen::Vector3f ExtendedDMG::get_orthogonal_axis(Eigen::Vector3f nx){
    Eigen::Vector3f ny;
    if(fabs(nx(0))>0.00000001){
        ny(1)=0;
        ny(2)=sqrt(nx(0)*nx(0)/(nx(0)*nx(0)+nx(2)*nx(2)));
        ny(0)=-nx(2)*ny(2)/nx(0);
    }
    else if(fabs(nx(1))>0.00000001){
        ny(2)=0;
        ny(0)=sqrt(nx(1)*nx(1)/(nx(1)*nx(1)+nx(0)*nx(0)));
        ny(1)=-ny(0)*nx(0)/nx(1);
    }
    else{
        ny(0)=0;
        ny(1)=sqrt(nx(2)*nx(2)/(nx(1)*nx(1)+nx(2)*nx(2)));
        ny(2)=-nx(1)*ny(1)/nx(2);
    }
    return ny;
}

int ExtendedDMG::pose_to_angle(Eigen::Quaternion<float> q, int component){
    Eigen::Matrix3f pose_gripper=q.toRotationMatrix();
    Eigen::Matrix3f gripper_to_standard_orientation=Eigen::Matrix3f::Zero();
    //check if the normal is inwards
    Eigen::Vector3f normal=component_to_average_normal.at(component);
    pcl::PointXYZRGBA center=all_centroids_cloud->at(supervoxel_to_pc_idx.at(*(connected_component_to_set_of_nodes.at(component).begin())));
    Eigen::Vector3f c;
    c<<center.x, center.y, center.z;
    bool inwards=is_normal_inwards(c, normal);
    // std::cout<<"component: "<<component<<" inwards: "<<inwards<<std::endl;
    if(inwards){
        gripper_to_standard_orientation(0, 2)=-1;
        gripper_to_standard_orientation(1, 0)=1;
        gripper_to_standard_orientation(2, 1)=-1;
    }
    else{
        gripper_to_standard_orientation(0, 2)=1;
        gripper_to_standard_orientation(1, 0)=1;
        gripper_to_standard_orientation(2, 1)=1;
    }
    //this matrix is not used because in this world everything comes in standard orientation. So perhaps remove it later!

    //this is component to base transformation
    Eigen::Matrix3f pose_component=component_pose_matrix(component);

    // std::cout<<std::endl<<"pose component T: "<<std::endl<<pose_component.transpose()<<std::endl<<std::endl;
    // std::cout<<"pose gripper: "<<std::endl<<pose_gripper<<std::endl<<std::endl;

    Eigen::Matrix3f gripper_to_base=pose_component.transpose()*pose_gripper;
    // std::cout<<" --- gripper to base: "<<std::endl<<gripper_to_base<<std::endl;
    Eigen::Vector3f x_prime=gripper_to_base.block<3, 1>(0, 0);
    // std::cout<<"x prime: "<<x_prime.transpose()<<std::endl;
    // std::cout<<"given quaternion: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl<<"  xprime: "<<x_prime.transpose()<<std::endl;
    double angle=atan2(x_prime(2), x_prime(1));
    // double angle=atan2(x_prime(2), x_prime(1))-M_PI;
    if(inwards){
        angle=angle+M_PI;
    }
    // double angle=atan2(-x_prime(2), -x_prime(1));
    // std::cout<<"angle: "<<angle<<std::endl;
    int int_angle=filter_angle(angle);
    // int int_angle=filter_angle(angle + M_PI);
    // std::cout<<"int_angle: "<<int_angle<<std::endl;
    return int_angle;

}

Eigen::Matrix3f ExtendedDMG::component_pose_matrix(int component){
    Eigen::Matrix3f component_matrix;

    Eigen::Vector3f nx=component_to_average_normal.at(component);
    Eigen::Vector3f ny=get_orthogonal_axis(nx);
    Eigen::Vector3f nz=nx.cross(ny);

    component_matrix(0,0)=nx(0);
    component_matrix(1,0)=nx(1);
    component_matrix(2,0)=nx(2);

    component_matrix(0,1)=ny(0);
    component_matrix(1,1)=ny(1);
    component_matrix(2,1)=ny(2);

    component_matrix(0,2)=nz(0);
    component_matrix(1,2)=nz(1);
    component_matrix(2,2)=nz(2);

    return component_matrix;
}

int ExtendedDMG::filter_angle(double angle){
    if (angle<0){
        angle+=2*M_PI;
    }
    angle=angle*180.0/M_PI;
    // std::cout<<"angle degree: "<<angle<<std::endl;
    double round_angle=angle+angle_jump/2;
    int int_angle=floor(round_angle);
    int_angle=int_angle-(int_angle%angle_jump);
    // std::cout<<"int_angle: "<<int_angle<<std::endl;
    if(int_angle%angle_jump!=0){
        std::cout<<"ERROR IN THE ROUNDING TO "<<angle_jump<<std::endl;
    }
    if(int_angle==360){
        int_angle=0;
    }
    return int_angle;
}


std::vector<std::pair<int, int>> ExtendedDMG::get_opposite_finger_nodes(Eigen::Vector3f direction, std::pair<int, int> principal_node){
    std::vector<std::pair<int, int>> output_list;
    //the current considered point
    Eigen::Vector3f current_point; 
    // std::cout<<"here: 33333"<<std::endl;
    current_point<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(principal_node.first)).x, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(principal_node.first)).y, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(principal_node.first)).z;
    // std::cout<<"here: 44444"<<std::endl;

    
    //raise this point of 1cm for the start
    Eigen::Vector3f start=current_point-10.0*direction;
    //now something 50 cm away for the intersections
    Eigen::Vector3f end=current_point+500.0*direction; 

    //get all the intersections! Many possibilities for opposite fingers sometimes
    std::vector<Eigen::Vector3f> intersections=get_ray_intersections(start, end);
    // std::cout<<"here: 555555"<<std::endl;

    pcl::PointXYZRGBA input;
    int K = 1;//three nearest neighbours
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for(Eigen::Vector3f v: intersections){
        //get the closest supervoxel
        input.x=v(0);
        input.y=v(1);
        input.z=v(2);
        if(centroids_kdtree.nearestKSearch(input, K, pointIdxNKNSearch, pointNKNSquaredDistance)> 0){
            int supervoxel_idx=pc_to_supervoxel_idx.at(pointIdxNKNSearch[0]); //there is only one
            //TODO get the possible angular components and check if their intersection with the ranges in the principal finger is good
            // std::cout<<"here: 66666"<<std::endl;
            // std::cout<<"supervoxel: "<<supervoxel_idx<<std::endl;
            
            std::vector<int> angle_components=node_to_angle_components.at(supervoxel_idx);
            // std::cout<<"here: 77777"<<std::endl;

            for(int angle_c_idx: angle_components){
                // std::cout<<"adding node to list: "<<supervoxel_idx<<" "<<angle_c_idx<<std::endl;
                output_list.push_back(std::pair<int, int>(supervoxel_idx, angle_c_idx));
            }
        }
    }

    return output_list;

}

std::map<int, double> ExtendedDMG::weight_regrasping_area(Eigen::Vector3f release_contact_1, Eigen::Vector3f release_contact_2, Eigen::Vector3f regrasp_contact_1, Eigen::Vector3f regrasp_contact_2){
    //empty map
    std::map<int, double> regrasp_area_values;

    //loop through the grasp nodes
    for(std::map<int, std::set<uint32_t>>::iterator iter=connected_component_to_set_of_nodes.begin(); iter!=connected_component_to_set_of_nodes.end(); ++iter){
        int component=iter->first;
        //the direction is the one of the normal to the component
        Eigen::Vector3f component_normal=component_to_average_normal.at(component);
        std::set<uint32_t> nodes=iter->second;
        for(uint32_t n: nodes){
            //check if this node has already been visited. If yes skip
            if(regrasp_area_values.count(int(n))<1){
                //check if the grasp is possible (i.e. necessary opening is not too big)
                Eigen::Vector3f c; //current centroid
                c<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(n)).x, 
                    all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(n)).y, 
                    all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(n)).z;

                std::cout<<"normal at node "<<n<<": "<<component_normal.transpose()<<std::endl;
                std::cout<<"to the point: "<<c.transpose()<<" the opposite is: "<<std::endl;


                //check if it is possible to grasp at the first contact
                Eigen::Vector3f end_point_ray=c-1000.0*component_normal; //this point is the final point for the ray, from c1 outwards (opposite to line) of 1000 (1m)
                Eigen::Vector3f init_point_ray=c+1000.0*component_normal; //this point is the final point for the ray, from c1 outwards (opposite to line) of 1000 (1m)
                
                std::vector<Eigen::Vector3f> intersections=get_ray_intersections(init_point_ray, end_point_ray);
                if(intersections.size()>0){
                    for(int idx=0; idx<intersections.size(); idx++){
                        Eigen::Vector3f point_i=intersections[idx];
                        std::cout<<"    "<<point_i.transpose()<<std::endl;
                        double distance=(point_i-c).norm();
                        if(distance>0.5){
                            //in this case the intersection is not the current considered point!
                            if(distance<max_fingers_opening_mm){
                                double cumulative_d=0;
                                //then this node is good for being regrasped
                                if(regrasp_area_values.count(int(n))<1){
                                    //distance between this node and the grasp line 
                                    double node_distance_1=((c-release_contact_1).cross((c-release_contact_2))).norm()/(release_contact_2-release_contact_1).norm();
                                    double node_distance_2=((c-regrasp_contact_1).cross((c-regrasp_contact_2))).norm()/(regrasp_contact_2-regrasp_contact_1).norm();
                                    double d_value=node_distance_1+node_distance_2;
                                    double d_diff=fabs(node_distance_1-node_distance_2);
                                    //if one of the distances is too close, then put 0 as value
                                    if(node_distance_1<regrasp_distance_threshold||node_distance_2<regrasp_distance_threshold){
                                        d_value=0.0;
                                        d_diff=0.0;
                                    }
                                    double val=std::max(0.0, d_value-1.2*d_diff);
                                    regrasp_area_values.insert(std::pair<int, double>(n, val));
                                    cumulative_d=val;
                                    for(int angle_comp:node_to_angle_components.at(n)){
                                        regrasping_candidate_nodes[1].push_back(std::pair<int, int>(n, angle_comp));
                                    }
                                }
                                //check the closest node to the intersection
                                int opposite_node=get_supervoxel_index(point_i, false);
                                //sanity check on the opposite node: get the two normals and see if they are too far apart (in absolute value)
                                // std::cout<<"lalalalala"<<std::endl;
                                try{
                                    Eigen::Vector3f normal_opposite=component_to_average_normal.at(nodes_to_connected_component.at(opposite_node));
                                    Eigen::Vector3f normal_n=component_to_average_normal.at(nodes_to_connected_component.at(n));
                                    // std::cout<<"dfklvjhsiodgh"<<std::endl;
                                    //get the angle between the vectors
                                    double cos_angle=normal_n.dot(normal_opposite)/(normal_opposite.norm()*normal_n.norm());
                                    if((fabs(acos(cos_angle))<0.4) || (fabs(acos(cos_angle)-M_PI)<0.4)){
                                        std::cout<<"---------------   VALID OPPOSITE COMPONENT FOUND"<<std::endl;
                                        std::cout<<"opposite node: "<<opposite_node<<std::endl;
                                        if(regrasp_area_values.count(opposite_node)<1){
                                            std::cout<<"IN!"<<std::endl;
                                            double node_distance_1=((point_i-release_contact_1).cross((point_i-release_contact_2))).norm()/(release_contact_2-release_contact_1).norm();
                                            double node_distance_2=((point_i-regrasp_contact_1).cross((point_i-regrasp_contact_2))).norm()/(regrasp_contact_2-regrasp_contact_1).norm();
                                            double d_value=node_distance_1+node_distance_2;
                                            double d_diff=fabs(node_distance_1-node_distance_2);
                                            //if one of the distances is too close, then put 0 as value
                                            if(node_distance_1<regrasp_distance_threshold||node_distance_2<regrasp_distance_threshold){
                                                d_value=0.0;
                                                d_diff=0.0;
                                            }
                                            double val=std::max(0.0, d_value-1.2*d_diff);
                                            regrasp_area_values.insert(std::pair<int, double>(opposite_node, val));
                                            cumulative_d+=val;
                                            if(node_to_angle_components.find(opposite_node)!= node_to_angle_components.end()){
                                                for(int angle_comp:node_to_angle_components.at(opposite_node)){
                                                    regrasping_candidate_nodes[1].push_back(std::pair<int, int>(opposite_node, angle_comp));
                                                }
                                                //put these two points (without angle component for now) in the regrasp value map
                                                regrasp_poses_distance_values[std::pair<int, int>(n, opposite_node)]=cumulative_d;
                                            }
                                        }
                                        else if(regrasp_poses_distance_values.count(std::pair<int, int>(n, opposite_node))<1 && regrasp_poses_distance_values.count(std::pair<int, int>(opposite_node, n))<1){

                                            // std::cout<<"the node was already considered!"<<std::endl;
                                            // double node_distance_1=((point_i-release_contact_1).cross((point_i-release_contact_2))).norm()/(release_contact_2-release_contact_1).norm();
                                            // double node_distance_2=((point_i-regrasp_contact_1).cross((point_i-regrasp_contact_2))).norm()/(regrasp_contact_2-regrasp_contact_1).norm();
                                            // double d_value=node_distance_1+node_distance_2;
                                            // double d_diff=fabs(node_distance_1-node_distance_2);
                                            double val=regrasp_area_values.at(opposite_node);
                                            // std::cout<<"+++ diff: "<<d_diff<<"  val: "<<val<<std::endl;
                                            cumulative_d+=val;
                                            if(node_to_angle_components.find(opposite_node)!= node_to_angle_components.end()){
                                                for(int angle_comp:node_to_angle_components.at(opposite_node)){
                                                    regrasping_candidate_nodes[1].push_back(std::pair<int, int>(opposite_node, angle_comp));
                                                }
                                                //put these two points (without angle component for now) in the regrasp value map
                                                regrasp_poses_distance_values[std::pair<int, int>(n, opposite_node)]=cumulative_d;
                                            }
                                            
                                        }
                                    }
                                    //otherwise this node is not valid, but maybe the opposite component is
                                    else{
                                        std::cout<<"------- INVALID NODE"<<std::endl;
                                        std::cout<<"invalid node: "<<n<<std::endl;
                                        if(regrasp_area_values.count(int(n))<1){
                                            regrasp_area_values.insert(std::pair<int, double>(n, 0.0));
                                        }
                                    }
                                } catch(std::out_of_range& e){
                                    std::cout<<"Missing average normal for component. "<<e.what()<<std::endl;
                                }
                            }
                            else{
                                //this node is not valid and so is the opposite component
                                if(regrasp_area_values.count(int(n))<1){
                                    std::cout<<"invalid node: "<<n<<std::endl;
                                    regrasp_area_values.insert(std::pair<int, double>(n, 0.0));
                                }
                                int opposite_node=get_supervoxel_index(point_i, false);
                                if(regrasp_area_values.count(opposite_node)<1){
                                    std::cout<<"invalid opposite node: "<<opposite_node<<std::endl;
                                    regrasp_area_values.insert(std::pair<int, double>(opposite_node, 0.0));
                                }
                            }
                        }

                    }
                }
            }
        }
    }

    return regrasp_area_values;
        
}

// Eigen::Matrix3f axisAngleRotMat(Eigen::Vector3f axis, double angle){
//     Eigen::Matrix3f rrt=axis*axis.transpose();
//     Eigen::Matrix3f Sr;
//     Sr<<0, -axis(2), axis(1),
//         axis(2), 0, -axis(0),
//         -axis(1), axis(0), 0;
//     Eigen::Matrix3f R=rrt+(Eigen::Matrix3f::Identity()-rrt)*cos(angle)+Sr*sin(angle);
//     return R;
// }

void ExtendedDMG::visualize_results(){
    //first of all add the shape to the viewer
    //viewer->addModelFromPolyData(colorable_shape, "object");
    // std::cout<<"green spheres: "<<desired_pose_1.transpose()<<std::endl;
    // std::cout<<"green spheres: "<<desired_pose_2.transpose()<<std::endl;
    // std::cout<<"red spheres: "<<initial_pose_1.transpose()<<std::endl;
    // std::cout<<"red spheres: "<<initial_pose_2.transpose()<<std::endl;
    // std::cout<<"dark green spheres: "<<regrasp1_principal.transpose()<<std::endl;
    // std::cout<<"dark green spheres: "<<regrasp1_secondary.transpose()<<std::endl;
    //remove all the previously made spheres, if any
    viewer->removeShape("goal_node_1");
    viewer->removeShape("goal_node_2");
    viewer->removeShape("initial_node_1");
    viewer->removeShape("initial_node_2");
    viewer->removeShape("regrasp1_node_1");
    viewer->removeShape("regrasp1_node_2");

    std::cout<<" --- now the regrasp1_angle is: "<<regrasp1_angle<<std::endl;


    //now add a green sphere to the goal nodes
    regrasp_viewer->addSphere(pcl::PointXYZ(desired_pose_1(0, 0)*1000.0, desired_pose_1(1, 0)*1000.0, desired_pose_1(2, 0)*1000.0), 3, 0.0, 1.0, 0.0, "goal_node_1");
    regrasp_viewer->addSphere(pcl::PointXYZ(desired_pose_2(0, 0)*1000.0, desired_pose_2(1, 0)*1000.0, desired_pose_2(2, 0)*1000.0), 3, 0.0, 1.0, 0.0, "goal_node_2");
    //add red spheres to the initial nodes
    regrasp_viewer->addSphere(pcl::PointXYZ(initial_pose_1(0, 0)*1000.0, initial_pose_1(1, 0)*1000.0, initial_pose_1(2, 0)*1000.0), 3, 1.0, 0.0, 0.0, "initial_node_1");
    regrasp_viewer->addSphere(pcl::PointXYZ(initial_pose_2(0, 0)*1000.0, initial_pose_2(1, 0)*1000.0, initial_pose_2(2, 0)*1000.0), 3, 1.0, 0.0, 0.0, "initial_node_2");
    //add a dark green sphere to the 1st gripper regrasp
    regrasp_viewer->addSphere(pcl::PointXYZ(regrasp1_principal(0), regrasp1_principal(1), regrasp1_principal(2)), 3, 0.0, 0.4, 0.0, "regrasp1_node_1");
    regrasp_viewer->addSphere(pcl::PointXYZ(regrasp1_secondary(0), regrasp1_secondary(1), regrasp1_secondary(2)), 3, 0.0, 0.4, 0.0, "regrasp1_node_2");

    //add the two initial finger poses
    // std::cout<<"initial pose 1: "<<initial_pose_1.transpose()<<std::endl;
    // std::cout<<"initial pose 2: "<<initial_pose_2.transpose()<<std::endl;
    draw_finger("finger1_principal", initial_pose_1.block<3, 1>(0, 0)*1000.0, Eigen::Quaternionf(initial_pose_1(6, 0), initial_pose_1(3, 0), initial_pose_1(4, 0), initial_pose_1(5, 0)), 0, true);
    draw_finger("finger1_secondary", initial_pose_2.block<3, 1>(0, 0)*1000.0, Eigen::Quaternionf(initial_pose_2(6, 0), initial_pose_2(3, 0), initial_pose_2(4, 0), initial_pose_2(5, 0)), 0, true);
    //add the two desired finger poses
    // std::cout<<"desired pose 1: "<<desired_pose_1.transpose()<<std::endl;
    // std::cout<<"desired pose 2: "<<desired_pose_2.transpose()<<std::endl;
    draw_finger("finger3_principal", desired_pose_1.block<3, 1>(0, 0)*1000.0, Eigen::Quaternionf(desired_pose_1(6, 0), desired_pose_1(3, 0), desired_pose_1(4, 0), desired_pose_1(5, 0)), 2, true);
    draw_finger("finger3_secondary", desired_pose_2.block<3, 1>(0, 0)*1000.0, Eigen::Quaternionf(desired_pose_2(6, 0), desired_pose_2(3, 0), desired_pose_2(4, 0), desired_pose_2(5, 0)), 2, true);
    
    //add the regrasp pose (first, need to get the proper orientation!)
    std::cout<<std::endl<<std::endl<<"++++++++++++++++++++++++++++++++++"<<std::endl;
    std::cout<<"angle: "<<regrasp2_angle<<std::endl;
    


    //the angle must be converted into degrees
    Eigen::Quaternionf regrasp_orientation=angle_to_pose(regrasp2_angle, regrasp2_principal);
    // std::cout<<"reconstructed angle: "<<pose_to_angle(regrasp_orientation, nodes_to_connected_component.at(get_supervoxel_index(regrasp2_principal)));
    // std::cout<<std::endl<<std::endl<<"++++++++++++++++++++++++++++++++++"<<std::endl;
    // std::cout<<"regrasp pose 1: "<<regrasp2_principal.transpose()<<" "<<regrasp_orientation.x()<<" "<<regrasp_orientation.y()<<" "<<regrasp_orientation.z()<<" "<<regrasp_orientation.w()<<std::endl;
    // std::cout<<"regrasp pose 2: "<<regrasp2_secondary.transpose()<<" "<<regrasp_orientation.x()<<" "<<regrasp_orientation.y()<<" "<<regrasp_orientation.z()<<" "<<regrasp_orientation.w()<<std::endl;
    
    draw_finger("finger2_principal", regrasp2_principal, regrasp_orientation, 1, true);
    draw_finger("finger2_secondary", regrasp2_secondary, regrasp_orientation, 1, true);

    std::cout<<"first gripper regrasp angle [deg]: "<<regrasp1_angle<<std::endl; 

    Eigen::Quaternionf regrasp2_orientation=angle_to_pose(regrasp1_angle, regrasp1_principal);

    // //debug: print the frame rotated by this angle
    // Eigen::Vector3f nx=get_normal_at_contact(regrasp1_principal);
    // Eigen::Vector3f ny=get_orthogonal_axis(nx);
    // Eigen::Vector3f nz=nx.cross(ny);
    // //now rotate ny of angle around nx
    // Eigen::Matrix3f Rot=axisAngleRotMat(nx, regrasp1_angle);
    // Eigen::Vector3f ny_new=Rot*ny;
    // Eigen::Vector3f nz_new=Rot*nz;
    // Eigen::Matrix4f transf=Eigen::Matrix4f::Identity();
    // transf.block<3, 1>(0, 0)=ny_new;
    // transf.block<3, 1>(0, 1)=nz_new;
    // transf.block<3, 1>(0, 2)=nx;
    // transf.block<3, 1>(0, 3)=regrasp1_principal;
    // Eigen::Affine3f aff;
    // aff.matrix()=transf;
    // regrasp_viewer->addCoordinateSystem(5.0, aff, "checking the rotation around the normal");


    //draw also the second regrasp (for 1st gripper)
    draw_finger("finger4_principal", regrasp1_principal, regrasp2_orientation, 3, true);
    draw_finger("finger4_secondary", regrasp1_secondary, regrasp2_orientation, 3, true);

    //DEBUG
    std::cout<<std::endl<<" +++ initial pose: "<<master_initial_pose.transpose()<<std::endl;
    // std::cout<<"  component: "<<nodes_to_connected_component.at(get_supervoxel_index(master_initial_pose.block<3, 1>(0, 0)*1000.0))<<std::endl;
    Eigen::Quaternionf quatquat(master_initial_pose(6, 0), master_initial_pose(3, 0), master_initial_pose(4, 0), master_initial_pose(5, 0));
    int initial_angle_test=pose_to_angle(quatquat, nodes_to_connected_component.at(get_supervoxel_index(master_initial_pose.block<3, 1>(0, 0), false)));
    // std::cout<<std::endl<<" +++++ initial angle test: "<<initial_angle_test<<std::endl;

    // std::cout<<"  +++desired pose: "<<master_desired_pose.transpose()<<std::endl;
    // std::cout<<"  component: "<<nodes_to_connected_component.at(get_supervoxel_index(master_desired_pose.block<3, 1>(0, 0)*1000.0))<<std::endl;
    // Eigen::Quaternionf quatquat2(master_desired_pose(6, 0), master_desired_pose(3, 0), master_desired_pose(4, 0), master_desired_pose(5, 0));
    // int final_angle_test=pose_to_angle(quatquat2, nodes_to_connected_component.at(get_supervoxel_index(master_desired_pose.block<3, 1>(0, 0)*1000.0)));
    // std::cout<<std::endl<<" +++++ final angle test: "<<final_angle_test<<std::endl;

    //draw the path, if there is any, for the 1st finger regrasp
    draw_path(r_translations[2], "1st_finger_regrasp");

    //loop through the nodes and add spheres with a radius normalized according to the bounding box
    // pcl::PointXYZ max, min;
    // pcl::getMinMax3D(*object_shape, min, max);
    // double normalizing_factor=std::max(std::max(max.x-min.x, max.y-min.y), max.z-min.z);
    // double radius;
    // for(std::pair<int, int> n : regrasping_candidate_nodes[1]){
    //     //now check the distance and normalize it w.r.t. the maximum value of the bounding box
    //     radius=10.0*nodes_distances_from_regrasps.at(n.first)/normalizing_factor;
    //     //if this node is one of the two that are perfect candidates for regrasping, then put them in yellow
    //     std::cout<<"node: "<<n.first<<" is: "<<all_centroids_cloud->at(supervoxel_to_pc_idx.at(n.first)).x<<" "<<all_centroids_cloud->at(supervoxel_to_pc_idx.at(n.first)).y<<" "<<all_centroids_cloud->at(supervoxel_to_pc_idx.at(n.first)).z<<std::endl;
    //     if(n.first==regrasp2_node1||n.first==regrasp2_node2){
    //         std::cout<<"!!!!!!!!!! equivalent to pc: "<<supervoxel_to_pc_idx.at(n.first)<<std::endl;
    //         regrasp_viewer->addSphere(all_centroids_cloud->at(supervoxel_to_pc_idx.at(n.first)), radius, 1.0, 1.0, 0.0, "node_"+std::to_string(n.first));
    //         regrasp_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "node_"+std::to_string(n.first));
    //     }
    //     else{
    //         regrasp_viewer->addSphere(all_centroids_cloud->at(supervoxel_to_pc_idx.at(n.first)), radius, 0.0, 0.0, 1.0, "node_"+std::to_string(n.first));
    //         regrasp_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "node_"+std::to_string(n.first));        
    //     }
    // }

    //draw the fingers during the transition
    // std::cout<<"--------------------------------------------------------------------------"<<std::endl;
    // Eigen::Vector3f current_point=initial_pose_1.block<3, 1>(0, 0);
    // std::cout<<"new point: "<<current_point.transpose()<<std::endl;
    // double current_angle=double(initial_angle_test)*M_PI/180.0;
    // std::cout<<"new angle: "<<current_angle<<std::endl<<std::endl;
    // for(int i=1; i<r_translations[0].size()-1; i++){
    //     current_point(0)=r_translations[0][i].x;
    //     current_point(1)=r_translations[0][i].y;
    //     current_point(2)=r_translations[0][i].z;
    //     //draw before and after the rotation
    //     // draw_finger("f_path_before"+std::to_string(i), current_point*1000.0, angle_to_pose(current_angle, current_point), 1, false);
    //     std::cout<<"new point: "<<current_point.transpose()<<std::endl;
    //     current_angle=current_angle+r_rotations[0][i-1];
    //     std::cout<<"new angle: "<<current_angle<<std::endl<<std::endl;
    //     // draw_finger("f_path_after"+std::to_string(i), current_point*1000.0, angle_to_pose(current_angle, current_point), 1, false);
    // }

}

void ExtendedDMG::spin_viewer_once(){
    ShapeAnalyzer::spin_viewer_once();
    if(!regrasp_viewer->wasStopped()){
        regrasp_viewer->spinOnce(1);
    }
    else{
        regrasp_viewer->close();
    }
}

bool ExtendedDMG::is_normal_inwards(Eigen::Vector3f contact, Eigen::Vector3f normal){
    //first of all check if the contact is really on the surface (assume a tolerance of 1.5 cm)
    Eigen::Vector3f precise_contact=contact;
    std::vector<Eigen::Vector3f> intersections=get_ray_intersections(contact, contact + 15*normal);
    if(intersections.size()>0){
        precise_contact=intersections[0];
    }
    else{
        intersections=get_ray_intersections(contact, contact - 15*normal);
        if(intersections.size()>0){
            precise_contact=intersections[0];
        }
    }

    intersections=get_ray_intersections(contact, contact + 1000*normal);
    //if the intersections from the surface, along the normal, are even, the normal is outwards, otherwise inwards
    if(intersections.size()%2==0){
    // std::cout<<"the normal is outwards"<<std::endl;
        return false;
    }
    // std::cout<<"the normal is inwards"<<std::endl;
    return true;

}

geometry_msgs::Pose ExtendedDMG::get_regrasp_pose(int gripper){
    geometry_msgs::Pose regrasp_pose;

    //get the corresponding regrasp point and angle
    Eigen::Vector3f regrasp_point;
    double regrasp_angle;

    if(gripper==0){
        regrasp_point=regrasp1_principal;
        regrasp_angle=regrasp1_angle;
        std::cout<<"  +++GIVING REGRASP ANGLE: "<<regrasp_angle<<std::endl;
    }
    else{
        regrasp_point=regrasp2_principal;
        regrasp_angle=regrasp2_angle;
    }

    Eigen::Quaternion<float> q=angle_to_pose(regrasp_angle, regrasp_point);
    //fill the geometry msg with the values (from mm to m)
    regrasp_pose.position.x=regrasp_point(0)/1000.0;
    regrasp_pose.position.y=regrasp_point(1)/1000.0;
    regrasp_pose.position.z=regrasp_point(2)/1000.0;

    regrasp_pose.orientation.x=q.x();
    regrasp_pose.orientation.y=q.y();
    regrasp_pose.orientation.z=q.z();
    regrasp_pose.orientation.w=q.w();

    return regrasp_pose;
}

Eigen::Quaternion<float> ExtendedDMG::angle_to_pose(double angle, Eigen::Vector3f contact){
    //the nx axis corresponds to the z axis of the gripper (this could be inverted according to which finger is set as principal finger)
    Eigen::Vector3f nx=get_normal_at_contact(contact);
    Eigen::Vector3f ny=get_orthogonal_axis(nx);
    Eigen::Vector3f nz=nx.cross(ny);

    // std::cout<<"the angle "<<angle<<" i.e. "<<angle*180/M_PI<<"  becomes: "<<std::endl;
    bool inwards=is_normal_inwards(contact, nx);
    if(inwards){
        // std::cout<<"which is inwards"<<std::endl;
        angle=angle-M_PI;
        // angle=-angle;
        Eigen::Matrix3f R;
        R.block<3, 1>(0, 0)=nx;
        R.block<3, 1>(0, 1)=ny;
        R.block<3, 1>(0, 2)=nz;
        Eigen::Quaternionf q_y;
        q_y.x()=0;
        q_y.y()=1;
        q_y.z()=0;
        q_y.w()=0;
        Eigen::Matrix3f R_around_y=q_y.toRotationMatrix();
        R=R_around_y*R;
        nx=R.block<3, 1>(0, 0);
        ny=R.block<3, 1>(0, 1);
        nz=R.block<3, 1>(0, 2);

    }

    // std::cout<<"normal: "<<nx.transpose()<<std::endl;

    // // std::cout<<"ny ax : "<<ny.transpose()<<std::endl;
    // std::cout<<"IN angle to pose ---- angle: "<<angle<<std::endl;

    // //now rotate ny around nx of angle
    // Eigen::AngleAxis<float> aa(angle*M_PI/180.0, nx);
    // Eigen::Vector3f t_ny=aa*ny;

    // // std::cout<<"transformed: "<<t_ny.transpose()<<std::endl;

    // //get the third axis
    // Eigen::Vector3f t_nz=nx.cross(t_ny);

    // //fill a matrix
    // Eigen::Matrix3f component_matrix;
    // component_matrix(0,0)=nx(0);
    // component_matrix(1,0)=nx(1);
    // component_matrix(2,0)=nx(2);

    // component_matrix(0,1)=t_ny(0);
    // component_matrix(1,1)=t_ny(1);
    // component_matrix(2,1)=t_ny(2);

    // component_matrix(0,2)=t_nz(0);
    // component_matrix(1,2)=t_nz(1);
    // component_matrix(2,2)=t_nz(2);

    // //reorient it according to the fingers component
    // Eigen::Matrix3f component_to_finger=Eigen::Matrix3f::Zero();

    // component_to_finger(0, 1)=1;
    // component_to_finger(1, 2)=1;
    // component_to_finger(2, 0)=1;

    // //this I am not sure if it is correct
    // // bool inwards=is_normal_inwards(contact, nx);
    // // if(!inwards){
    // //     std::cout<<"the normal is outwards!"<<std::endl;
    // //     component_to_finger(0, 1)=-1;
    // //     component_to_finger(2, 0)=1;
    // // }

    // // Eigen::Matrix3f finger_pose=component_to_finger*component_matrix*component_to_finger.transpose();
    // Eigen::Matrix3f finger_pose=(component_to_finger*component_matrix.transpose()).transpose();

    // //this is component to base transformation
    // // Eigen::Matrix3f pose_component=component_pose_matrix(nodes_to_connected_component.at(get_supervoxel_index(contact)));

    // // std::cout<<"matrix generated: "<<std::endl<<component_matrix<<std::endl;

    // // component_matrix.transposeInPlace();

    // // std::cout<<"component_matrix: "<<std::endl<<pose_component<<std::endl;


    // //get the quaternion corresponding to this matrix
    // // Eigen::Quaternion<float> q(component_matrix);
    // Eigen::Quaternion<float> q(finger_pose);

    Eigen::Vector3f gripper_x;
    gripper_x<<0, -cos(angle), -sin(angle);
    // gripper_x<<0, cos(angle), sin(angle);
    // gripper_x<<0, cos(angle+M_PI), sin(angle+M_PI);
    Eigen::Vector3f gripper_z;
    gripper_z<<1, 0, 0; //because it is nx in the component
    Eigen::Vector3f gripper_y=gripper_z.cross(gripper_x);
    Eigen::Matrix3f gripper_pose_in_component;
    gripper_pose_in_component(0, 0)=gripper_x(0);
    gripper_pose_in_component(1, 0)=gripper_x(1);
    gripper_pose_in_component(2, 0)=gripper_x(2);

    gripper_pose_in_component(0, 1)=gripper_y(0);
    gripper_pose_in_component(1, 1)=gripper_y(1);
    gripper_pose_in_component(2, 1)=gripper_y(2);

    gripper_pose_in_component(0, 2)=gripper_z(0);
    gripper_pose_in_component(1, 2)=gripper_z(1);
    gripper_pose_in_component(2, 2)=gripper_z(2);

    Eigen::Matrix3f component_matrix;
    component_matrix(0,0)=nx(0);
    component_matrix(1,0)=nx(1);
    component_matrix(2,0)=nx(2);

    component_matrix(0,1)=ny(0);
    component_matrix(1,1)=ny(1);
    component_matrix(2,1)=ny(2);

    component_matrix(0,2)=nz(0);
    component_matrix(1,2)=nz(1);
    component_matrix(2,2)=nz(2);

    Eigen::Matrix3f gripper_pose=component_matrix*gripper_pose_in_component;
    // Eigen::Matrix3f gripper_pose=Eigen::Matrix3f::Identity(3, 3);

    Eigen::Quaternion<float> q(gripper_pose);
    // std::cout<<q.x()<<"  "<<q.y()<<"  "<<q.z()<<"  "<<q.w()<<std::endl;
    return q;

}

double min_vertices_distance(std::vector<Eigen::Vector3f> rectangle1, std::vector<Eigen::Vector3f> rectangle2, std::vector<Eigen::Vector3f> rectangle3){
    double min_clearence=DBL_MAX;
    double min_d1=DBL_MAX;
    double min_d2=DBL_MAX;
    for(uint i=0; i<rectangle3.size(); i++){
        //the rectangles all have 4 elements
        for(uint j=0; j<rectangle1.size(); j++){
            double d1=(rectangle3[i]-rectangle1[j]).norm();
            // std::cout<<d1<<std::endl;
            //min distance of rectancle 3 from rectangle 1
            if(d1<min_d1){
                min_d1=d1;
            }
            double d2=(rectangle3[i]-rectangle2[j]).norm();
            // std::cout<<d2<<std::endl;
            //min distance of rectangle 3 from rectangle 2
            if(d2<min_d2){
                min_d2=d2;
            }
        }
    }
    min_clearence=std::min(min_d1, min_d2);
    return min_clearence;
}

int ExtendedDMG::get_collision_free_regrasp_angle(Eigen::Vector3f contact1_principal, Eigen::Vector3f contact1_secondary, Eigen::Vector3f point1_principal, Eigen::Vector3f point1_secondary, int grasping_angle1, Eigen::Vector3f point2_principal, Eigen::Vector3f point2_secondary, int &grasping_angle2){
    int best_possible_angle=-1;

    //store the set of valid angles for the secondary finger
    std::set<int> secondary_regrasp_valid_angles=possible_angles.at(get_supervoxel_index(contact1_secondary, false));
    // std::cout<<"valid secondary angles: :::::::";
    // for(auto ang : secondary_regrasp_valid_angles){
    //     std::cout<<" "<<ang;
    // }
    std::cout<<std::endl;
    //store also the secondary component
    int secondary_regrasping_cc=nodes_to_connected_component.at(get_supervoxel_index(contact1_secondary, false));
    int principal_regrasping_cc=nodes_to_connected_component.at(get_supervoxel_index(contact1_principal, false));


    //create the first rectangle in 3D:
    Eigen::Vector3f v11=point1_principal;
    Eigen::Vector3f v12=point1_secondary;
    //for the other 2 vertices, get the normal to the surface at the 1st contact point and check the 0 angle direction
    Eigen::Vector3f normal=get_normal_at_contact(v11);
    bool inwards=is_normal_inwards(v11, normal);
    Eigen::Vector3f zero_axis=get_zero_angle_direction(v11);
    //now rotate this axis of the grasping angle around the normal
    std::cout<<"first grasping angle: "<<grasping_angle1<<std::endl;
    Eigen::Matrix3f R_axis_angle=axis_angle_matrix(normal, grasping_angle1*M_PI/180.0);
    if(inwards){
        R_axis_angle=axis_angle_matrix(normal, grasping_angle1+M_PI);
    }
    Eigen::Vector3f direction=R_axis_angle*zero_axis; //double check if it is correct
    std::cout<<"first direction: "<<direction.transpose()<<std::endl;
    //now get the two other vertices of the rectangle, from the first two along this direction at distance of finger length
    Eigen::Vector3f v13=v12+l_finger*direction;
    Eigen::Vector3f v14=v11+l_finger*direction;
    std::vector<Eigen::Vector3f> rectangle1;
    rectangle1.push_back(v11);
    rectangle1.push_back(v12);
    rectangle1.push_back(v13);
    rectangle1.push_back(v14);

    //get a plane that divides the space into two: one area is the one containing the grasping fingers.
    //the point defining this plane is the point1_principal
    //the normal to the plane is the direction of the rectangle
    Eigen::Vector4f plane;
    plane<<direction(0), direction(1), direction(2), -direction.dot(point1_principal);
    Eigen::Vector3f finger_direction1=direction;
    finger_direction1.normalize();
    Eigen::Vector4f control_point1;
    control_point1.block<3, 1>(0, 0)=point1_principal+finger_direction1;
    control_point1(3)=1;
    // std::cout<<"direction: "<<direction.transpose()<<std::endl;
    // std::cout<<"CONTROL sign: "<<plane.dot(control_point1)<<std::endl;

    //the possible angles of the first gripper:
    std::set<int> regrasp_gripper1_angles=possible_angles.at(get_supervoxel_index(point2_principal, false));
    int current_angle=grasping_angle2;

    regrasp_gripper1_angles.erase(current_angle);
    double min_clearence=DBL_MAX;

    while(regrasp_gripper1_angles.size()>0){
        //create the second rectangle in 3D:
        Eigen::Vector3f v21=point2_principal;
        Eigen::Vector3f v22=point2_secondary;
        //obtain the other 2 vertices with the same procedure as before:
        normal=get_normal_at_contact(v21);
        inwards=is_normal_inwards(v21, normal);
        zero_axis=get_zero_angle_direction(v21);
        std::cout<<"desired angle: "<<current_angle<<std::endl;
        R_axis_angle=axis_angle_matrix(normal, current_angle*M_PI/180.0);
        if(inwards){
            R_axis_angle=axis_angle_matrix(-normal, current_angle*M_PI/180.0-M_PI);

        }
        direction=R_axis_angle*zero_axis; //double check if it is correct
        std::cout<<"direction: "<<direction.transpose()<<std::endl;
        Eigen::Vector3f v23=v22+l_finger*direction;
        Eigen::Vector3f v24=v21+l_finger*direction;
        std::vector<Eigen::Vector3f> rectangle2;
        rectangle2.push_back(v21);
        rectangle2.push_back(v22);
        rectangle2.push_back(v23);
        rectangle2.push_back(v24);

        std::cout<<"FIRST RECTANGLE: "<<std::endl<<v11.transpose()<<std::endl<<v12.transpose()<<std::endl<<v13.transpose()<<std::endl<<v14.transpose()<<std::endl;
        std::cout<<"SECOND RECTANGLE: "<<std::endl<<v21.transpose()<<std::endl<<v22.transpose()<<std::endl<<v23.transpose()<<std::endl<<v24.transpose()<<std::endl;

        //Now, loop through all the possible candidate regrasping angles to check if some are collision-free
        std::set<int> all_angles=possible_angles.at(get_supervoxel_index(contact1_principal, false));
        // std::cout<<"valid principal angles: :::::::";
        // for(auto ang : all_angles){
        //     std::cout<<" "<<ang;
        // }
        //the first two verticese will always be the same
        Eigen::Vector3f v31=contact1_principal;
        Eigen::Vector3f v32=contact1_secondary;
        normal=get_normal_at_contact(v31);
        inwards=is_normal_inwards(v31, normal);
        zero_axis=get_zero_angle_direction(v31);
        Eigen::Vector3f v33, v34;
        std::vector<Eigen::Vector3f> rectangle3;
        rectangle3.push_back(v31);
        rectangle3.push_back(v32);
        rectangle3.push_back(v33);
        rectangle3.push_back(v34);
        float max_clearence=0;
        min_clearence=DBL_MAX;
        // std::cout<<"ALL ANGLES: "<<all_angles.size()<<std::endl;
        for(int alpha : all_angles){
            // std::cout<<std::endl;
            // std::cout<<" ... testing: "<<alpha<<std::endl;

            //check if this angle is valid in the opposite component. If not, move forward
            Eigen::Quaternionf current_pose=angle_to_pose(M_PI*float(alpha)/180.0, contact1_principal);
            // std::cout<<"pose: "<<current_pose.x()<<" "<<current_pose.y()<<" "<<current_pose.z()<<" "<<current_pose.w()<<std::endl;
            int opposite_angle=pose_to_angle(current_pose, principal_regrasping_cc);
            // int opposite_angle=pose_to_angle(current_pose, secondary_regrasping_cc);
            // draw_finger("finger_secondary"+std::to_string(alpha), contact1_secondary, current_pose, 0);

            // std::cout<<"opposite_angle: "<<opposite_angle<<std::endl;
            if(secondary_regrasp_valid_angles.count(opposite_angle)>0){
                // std::cout<<"VALID!"<<std::endl;


            //the second vertices depend on the current angle. Obtained as before
                R_axis_angle=axis_angle_matrix(normal, M_PI*float(alpha)/180.0);
                if(inwards){
                    // R_axis_angle=axis_angle_matrix(normal, M_PI*float(alpha)/180.0+M_PI);
                }
                direction=R_axis_angle*zero_axis;
                v33=v32+l_finger*direction;
                v34=v31+l_finger*direction;
                rectangle3[2]=v33;
                rectangle3[3]=v34;

                //check first if this angle is not in the same direction of the first gripper
                //this is done by ensuring that the two "finger vectors" are pointing in different directions of the plane (starting from the point on the plane)
                Eigen::Vector3f finger_direction2=direction;
                finger_direction2.normalize();
                Eigen::Vector4f control_point2;
                control_point2.block<3, 1>(0, 0)=point1_principal+finger_direction2;
                control_point2(3)=1;
                // std::cout<<"direction: "<<direction.transpose()<<std::endl;
                // std::cout<<"result sign sign: "<<plane.dot(control_point2)<<std::endl;
                //the two control points should have opposite signs, i.e. be on opposite sides of the plane
                //it is -0.2 instead of 0 to account for numerical imprecisions
                if((plane.dot(control_point1))*(plane.dot(control_point2))<-0.01){

                    // draw_finger("finger_testing"+std::to_string(alpha), contact1_principal, current_pose, 0);
                    bool collision1=false;
                    bool collision2=false;
                    //test if it is in collision with the first rectangle:
                    collision1=are_rectangles_intersecting(rectangle1, rectangle3);
                    if(!collision1){
                        collision2=are_rectangles_intersecting(rectangle2, rectangle3);
                    }

                    //if both rectangles are collision-free, then we are happy and this angle is a good candidate angle!
                    if(!(collision1||collision2)){
                        //check the clearence (somehow)
                        float clearence11=std::min((v33-v13).norm(), (v33-v14).norm()); 
                        float clearence12=std::min((v34-v13).norm(), (v34-v14).norm());
                        float clearence1=std::min(clearence11, clearence12); 
                        float clearence21=std::min((v33-v23).norm(), (v33-v24).norm()); 
                        float clearence22=std::min((v34-v23).norm(), (v34-v24).norm());
                        float clearence2=std::min(clearence21, clearence22);
                        // double min_clearence_local=std::min(clearence1, clearence2);
                        float clearence=clearence1+clearence2 - std::max(clearence1, clearence2);
                        //get the minimum clearence with ALL the vertices
                        double min_clearence_all_vertices=min_vertices_distance(rectangle1, rectangle2, rectangle3);
                        // std::cout<<std::endl<<"min tot clearence: "<<min_clearence_all_vertices<<std::endl<<std::endl;

                        // std::cout<<"  --clearence: "<<clearence<<std::endl;
                        if(clearence>max_clearence){
                            std::cout<<"CURRENT RECTANGLE: "<<std::endl<<v31.transpose()<<std::endl<<v32.transpose()<<std::endl<<v33.transpose()<<std::endl<<v34.transpose()<<std::endl;

                            max_clearence=clearence;
                            best_possible_angle=alpha;
                            std::cout<<"FOUND BEST POSSIBLE ANGLE  -- "<<alpha<<std::endl;
                            if(min_clearence_all_vertices<min_clearence){
                                min_clearence=min_clearence_all_vertices;
                            }
                        } 

                    }
                }
            }

        }
        if(min_clearence>min_clearence_threshold){
            break;
        }
        //get a new element of the set of angles (only if it is valid)
        bool valid=false;
        while(!valid&&regrasp_gripper1_angles.size()>0){
            current_angle=*regrasp_gripper1_angles.begin();
            regrasp_gripper1_angles.erase(current_angle);
            Eigen::Quaternionf current_pose=angle_to_pose(M_PI*float(current_angle)/180.0, point2_principal);
            int current_secondary_node=get_supervoxel_index(point2_secondary, false);
            int current_secondary_component=nodes_to_connected_component.at(current_secondary_node);
            int regrasp1_angle_secondary=pose_to_angle(current_pose, current_secondary_component);
            if(possible_angles.at(current_secondary_node).find(regrasp1_angle_secondary)!=possible_angles.at(current_secondary_node).end()){
                valid=true;
            }
        }
    }
    grasping_angle2=current_angle;
    std::cout<<std::endl<<"++++++++ MIN CLEARENCE: "<<min_clearence<<std::endl;
    if(min_clearence<min_clearence_threshold){
        std::cout<<"The current configuration is not far enough from the gripper..."<<std::endl;
        return -1;
    }
    //TODO check if moving the second regrasping angle improves the situation
    return best_possible_angle;
}

void ExtendedDMG::draw_finger(std::string name, Eigen::Vector3f position, Eigen::Quaternion<float> orientation, int color_profile, bool is_regrasp_viewer){
    if(is_regrasp_viewer){
        regrasp_viewer->removeShape(name);
    }
    else{
        viewer->removeShape(name);
    }
    Eigen::Matrix3f cube_rot_matrix=orientation.toRotationMatrix();
    Eigen::Vector3f cube_pos=position+cube_rot_matrix*Eigen::Vector3f(-l_finger/2+1.5, 0, 0);
    if(is_regrasp_viewer){
        regrasp_viewer->addCube(cube_pos, orientation, l_finger, 6, 6, name);
    }
    else{
        viewer->addCube(cube_pos, orientation, l_finger, 6, 6, name);
    }
    double r, g, b;
    switch(color_profile){
        case 0:
            r=1.0;
            g=0.0;
            b=0.0;
            break;
        case 1:
            r=0.0;
            g=0.0;
            b=1.0;
            break;
        case 2:
            r=0.0;
            g=1.0;
            b=0.0;
            break;
        case 3:
            r=0.0;
            g=0.4;
            b=0.0;
            break;
    }

    if(is_regrasp_viewer){
        regrasp_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name);
        regrasp_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
    }
    else{
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
    }

    Eigen::Matrix4f transf=Eigen::Matrix4f::Identity();
    transf.block<3, 3>(0, 0)=orientation.toRotationMatrix();
    transf.block<3,1>(0, 3)=cube_pos;
    Eigen::Affine3f aff;
    aff.matrix()=transf;
    if(is_regrasp_viewer){
        // regrasp_viewer->addCoordinateSystem(3.0, aff, "frame_"+name);
    }
    //no coordinate systems in the first viewer

}

void ExtendedDMG::draw_path(std::vector<geometry_msgs::Point> path, std::string sequence){
    //remove previous path (not done exactly correctly... store the previous number somewhere):
    int max_line_id=std::max(r_translations[0].size(), std::max(r_translations[1].size(), r_translations[2].size()))-1;
    for(int i=0; i<=max_line_id; i++){
        viewer->removeShape(sequence+std::to_string(i));
    }
    if(path.size()<2){
        return;
    }
    std::cout<<" +++++++++++++++++ Drawing path!"<<std::endl;
    pcl::PointXYZRGBA p1, p2;
    p1.x=path[0].x*1000.0;
    p1.y=path[0].y*1000.0;
    p1.z=path[0].z*1000.0;

    for(int i=1; i<path.size(); i++){
        p2.x=path[i].x*1000.0;
        p2.y=path[i].y*1000.0;
        p2.z=path[i].z*1000.0;
        std::cout<<"point1: "<<p1.x<<" "<<p1.y<<" "<<p1.z<<std::endl;
        std::cout<<"point2: "<<p2.x<<" "<<p2.y<<" "<<p2.z<<std::endl;
        regrasp_viewer->addLine(p1, p2, 0, 0.4, 0, sequence+std::to_string(i));
        regrasp_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 15, sequence+std::to_string(i));
        p1=p2;
    }   

}






/**
    Returns the intersections between a segment and a cylinder
    @param segment_start the first point of the segment
    @param segment_end the final point of the segment
    @cylinder_start the base point of the cylinder
    @cylinder_end the top point of the cylinder
    @radius the radius of the cylinder
    @return a vector with the intersections, if any 
*/
// std::vector<Eigen::Vector3f> get_segment_cylinder_intersection(Eigen::Vector3f segment_start, Eigen::Vector3f segemnt_end, Eigen::Vector3f cylinder_start, Eigen::Vector3f cylinder_end, double radius){
//     //TODO first check if the segment is parallel to the cylinder, and if yes see if the distance is within the radius or not


//     //this is taken from https://stackoverflow.com/questions/4078401/trying-to-optimize-line-vs-cylinder-intersection
//     std::vector<Eigen::Vector3f> intersections;
//     double cxmin, cymin, czmin, cxmax, cymax, czmax;
    
//     if(cylinder_start(2)<cylinder_end(2)){
//         czmin=cylinder_start(2)-radius;
//         czmax=cylinder_end(2)+radius;
//     }
//     else{
//         czmin=cylinder_end(2)-radius;
//         czmax=cylinder_start(2)+radius;
//     }
//     if(cylinder_start(1)<cylinder_end(1)){
//         cymin=cylinder_start(1)-radius;
//         cymax=cylinder_end(1)+radius;
//     }
//     else{
//         cymin=cylinder_end(1)-radius;
//         cymax=cylinder_start(1)+radius;
//     }
//     if(cylinder_start(0)<cylinder_end(0)){
//         cxmin=cylinder_start(0)-radius;
//         cxmax=cylinder_end(0)+radius;
//     }
//     else{
//         cxmin=cylinder_end(0)-radius;
//         cxmax=cylinder_start(0)+radius;
//     }

//     Eigen::Vector3f cylinder_AB=cylinder_end-cylinder_start;
//     Eigen::Vector3f A0=segment_start-cylinder_start;
//     Eigen::Vector3f A0xAB=A0.cross(cylinder_AB);
//     Eigen::Vector3f dir=segment_end-segment_start;
//     Eigen::Vector3f VxAB=dir.cross(cylinder_AB);

//     double ab2=cylinder_AB.dot(cylinder_AB);
//     double a=VxAB.dot(VxAB);
//     double b=2*VxAB.dot(AOxAB);
//     double c=AOxAB.dot(AOxAB)-(radius*radius*ab2);
//     double d=b*b-4*a*c;

//     if(d<0){
//         return intersections; //empty
//     }

// }



/**
    Obtains the rotation matrix of the rotation around the axis of a given angle
    @param axis the axis around which the rotation is done
    @param theta the angle of the rotation
    @return the rotation matrix
*/
Eigen::Matrix3f axis_angle_matrix(Eigen::Vector3f axis, double theta){
    // Eigen::Matrix3f R;
    // double x=axis(0);
    // double y=axis(1);
    // double z=axis(2);

    // double cos_th=cos(theta);
    // double sin_th=sin(theta);

    // R<< cos_th + x*x * (1-cos_th), x*y* (1-cos_th) - z*sin_th, x*z* (1-cos_th) + y*sin_th,
    //     y*x* (1-cos_th) + z*sin_th, cos_th + y*y* (1-cos_th), y*z* (1-cos_th) - x*sin_th,
    //     z*x* (1-cos_th) - y*sin_th, z*y* (1-cos_th) + x*sin_th, cos_th + z*z* (1-cos_th);

    // return R;

    Eigen::Matrix3f rrt=axis*axis.transpose();
    Eigen::Matrix3f Sr;
    Sr<<0, -axis(2), axis(1),
        axis(2), 0, -axis(0),
        -axis(1), axis(0), 0;
    Eigen::Matrix3f R=rrt+(Eigen::Matrix3f::Identity()-rrt)*cos(theta)+Sr*sin(theta);
    return R;
}


/**
    Checks the intersection between a segment and a plane
    @param start_point the start point of the segment
    @param end_point the end point of the segment
    @param plane_normal the normal to the plane
    @param plane_point a point on the plane 
    @return the intersection point, -1 if they don't intersect, 0 if they intersect in 1 point, 1 if the line lies on the plane
*/
std::pair<Eigen::Vector3f, int> line_plane_intersection(Eigen::Vector3f start_point, Eigen::Vector3f end_point, Eigen::Vector3f plane_normal, Eigen::Vector3f plane_point){
    //the line lies on the plane if it is normal to the plane normal
    if(fabs((end_point-start_point).dot(plane_normal))<0.000000001){
        return std::pair<Eigen::Vector3f, int>(Eigen::Vector3f(), 1);
    }

    //get the signed distance between the two segment points and the plane
    double start_dist=plane_normal.dot(start_point-plane_point);
    double end_dist=plane_normal.dot(end_point-plane_point);
    //check if they have the same sign. If yes, there is no intersection
    if(start_dist*end_dist>0){
        return std::pair<Eigen::Vector3f, int>(Eigen::Vector3f(), -1);
    }

    //get the point on the line
    double t=start_dist/(start_dist-end_dist);
    Eigen::Vector3f point=t*(end_point-start_point);
    return std::pair<Eigen::Vector3f, int>(point, 0);

}

/**
    Checks if a segment intersects a rectangle
    @param start_point the start point of the segment
    @param end_point the end point of the segment
    @param rectangle a vector with the 4 vertices of the rectangle
    @return true if they intersect, false otherwise
*/
bool segment_rectangle_intersecting(Eigen::Vector3f start_point, Eigen::Vector3f end_point, std::vector<Eigen::Vector3f> rectangle){
    //find the normal vector to the plane of the rectangle
    Eigen::Vector3f A, B, C, D; //three point of the rectangle
    A=rectangle[0];
    B=rectangle[1];
    C=rectangle[2];
    D=rectangle[3];
    Eigen::Vector3f n=(B-A).cross(C-A);
    n.normalize();
    std::pair<Eigen::Vector3f, int> intersection=line_plane_intersection(start_point, end_point, n, A);
    if(intersection.second==-1){
        //there is no intersection
        return false;
    }
    else{
        //get a bounding box:
        double maxx, maxy, maxz, minx, miny, minz;
        double epsilon=0.001;

        maxx=std::max(A(0), std::max(B(0), std::max(C(0), D(0))));
        maxy=std::max(A(1), std::max(B(1), std::max(C(1), D(1))));
        maxz=std::max(A(2), std::max(B(2), std::max(C(2), D(2))));

        minx=std::min(A(0), std::max(B(0), std::max(C(0), D(0))));
        miny=std::min(A(1), std::max(B(1), std::max(C(1), D(1))));
        minz=std::min(A(2), std::max(B(2), std::max(C(2), D(2))));
        if(intersection.second==0){
        //check if the point is inside the rectangle:
            Eigen::Vector3f point=intersection.first;

            if(point(0)>=minx&&point(0)<=maxx&&
                point(1)>=miny&&point(1)<=maxy&&
                point(2)>=minz&&point(2)<=maxz){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            //check if the points are outside in the same direction
            if((start_point(0)>maxx && end_point(0)>maxx)||
                (start_point(1)>maxy && end_point(1)>maxy)||
                (start_point(2)>maxz && end_point(2)>maxz)||
                (start_point(0)<minx && end_point(0)<minx)||
                (start_point(1)<miny && end_point(1)<miny)||
                (start_point(2)<minz && end_point(2)<minz)){
                return false;
            }
            else{
                return true;
            }

        }
    }

}

/**
    Checks if two rectangles are intersecting
    @param rectangle_1 a vector containing the 4 vertices of the 1st rectangle (in connected order)
    @param rectangle_2 a vector containing the 4 vertices of the 2nd rectangle (in connected order)
    @return true if the rectangles intersect, false otherwise
*/
bool are_rectangles_intersecting(std::vector<Eigen::Vector3f> rectangle_1, std::vector<Eigen::Vector3f> rectangle_2){
    //check if any of the 4 segments of the 1st rectangle intersects the 2nd rectangle
    bool intersects=false;
    Eigen::Vector3f start, end;
    for(int i=0; i<rectangle_1.size(); i++){
        start=rectangle_1[i-1];
        end=rectangle_1[i];
        intersects=segment_rectangle_intersecting(start, end, rectangle_2);
        if(intersects){
            return true;
        }
    }
    //last segment:
    start=rectangle_1[rectangle_1.size()-1];
    end=rectangle_1[0];
    intersects=segment_rectangle_intersecting(start, end, rectangle_2);
    if(intersects){
        return true;
    }
    //now check if any of the 4 segments of the 2n rectangle intersects the 1st rectangle
    for(int i=0; i<rectangle_2.size(); i++){
        start=rectangle_2[i-1];
        end=rectangle_2[i];
        intersects=segment_rectangle_intersecting(start, end, rectangle_1);
        if(intersects){
            return true;
        }
    }
    //last segment:
    start=rectangle_2[rectangle_2.size()-1];
    end=rectangle_2[0];
    intersects=segment_rectangle_intersecting(start, end, rectangle_1);
    if(intersects){
        return true;
    }

    return false;
}


