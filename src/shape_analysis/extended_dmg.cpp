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

    int principal_desired_angle=pose_to_angle(q, initial_contact_cc);

    //get also the angle of the secondary finger at the desired pose
    if(finger_id==1){
        initial_contact_cc=nodes_to_connected_component.at(initial_centroid_idx_2);
        q=Eigen::Quaternion<float>(initial_pose_2(6), initial_pose_2(3), initial_pose_2(4), initial_pose_2(5));
    }
    else{
        initial_contact_cc=nodes_to_connected_component.at(initial_centroid_idx_1);
        q=Eigen::Quaternion<float>(initial_pose_1(6), initial_pose_1(3), initial_pose_1(4), initial_pose_1(5));
    }
    int secondary_desired_angle=pose_to_angle(q, initial_contact_cc);

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
    int regrasp1_angle=-1;
    if(direct_regrasp_1&& !angle_in_collision){
        //this is the best case
        r_rotations[2]=std::vector<double>();
        r_finger_distances[2]=std::vector<double>();
        r_translations[2]=std::vector<geometry_msgs::Point>();
        //the regrasp of the 1st gripper is exactly the desired grasp
        regrasp1_principal=contact_point1;
        std::cout<<"DIRECT REGRASP: "<<regrasp1_principal.transpose();
        regrasp1_secondary=contact_point2;
        regrasp1_angle=principal_desired_angle;
        //add it to the list
        int regrasp1_node=get_supervoxel_index(regrasp1_principal);
        std::cout<<"   node: "<<regrasp1_node<<std::endl;
        int ang_component=node_angle_to_angle_component.at(std::pair<int, int>(regrasp1_node, regrasp1_angle));
        regrasping_candidate_nodes[0].push_back(std::pair<int, int>(regrasp1_node, ang_component));
    }
    else{
        std::cout<<"MORE WORK MUST BE DONE! the 1st gripper cannot directly regrasp!"<<std::endl;
        //call this for the first gripper
        find_available_regrasping_points(contact_point1, contact_point2, principal_desired_angle, secondary_desired_angle, 0);

        std::cout<<"Found available regrasping points for the 1st gripper!"<<std::endl;

        //the first gripper must have a determined regrasp point, and so does the second gripper! for now it is only an area......

        //for now use the first node available among the regrasp candidates 
        //TODO: use all the candidates when it comes to find the 2nd gripper's regrasp
        pcl::PointXYZRGBA p=all_centroids_cloud->at(supervoxel_to_pc_idx.at(regrasping_candidate_nodes[0][0].first));
        regrasp1_principal<<p.x, p.y, p.z;
        //get the first available angle:
        regrasp1_angle=*(possible_angles.at(regrasping_candidate_nodes[0][0].first).begin());
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


    std::cout<<"going to the second gripper processing..."<<std::endl;
    std::cout<<"angles: "<<principal_desired_angle<<"    "<<secondary_desired_angle<<std::endl;    
    

    //now we have a regrasping area (or points) for the first gripper. We have to obtain one for the second gripper.
    //We assign to each node in the graph a weight wich depends on the sum of the distances from the release grasp and the 1st regrasp of the
    //first gripper
    
    //let's assume for now that the release contacts are the initial contacts
    Eigen::Vector3f release_contact_1, release_contact_2; //these two are kept in mm
    release_contact_1=1000.0*initial_pose_1.block<3,1>(0, 0);
    release_contact_2=1000.0*initial_pose_2.block<3,1>(0, 0);
    Eigen::Quaternion<float> quat(initial_pose_1(6), initial_pose_1(3), initial_pose_1(4), initial_pose_1(5));
    int release_angle=pose_to_angle(quat, nodes_to_connected_component.at(get_supervoxel_index(release_contact_1)));



    //we also must check if it is possible to release the grasp at the initial location, or if additional motions are required
    //This is a TODO for later, because for now one can assume that the object has just been grasped there, so release is always possible
    //it can be generalized "very easily" though
    std::cout<<"weighting regrasp area"<<std::endl;

    //now, we loop for all the possible contact points until we find a valid solution!
    bool valid_configuration=false;
    //get the extended target nodes (the final goal)
    int ang_component1=node_angle_to_angle_component.at(std::pair<int, int>(get_supervoxel_index(contact_point1), principal_desired_angle));
    std::pair<int, int> regrasp_node_goal1=std::pair<int, int>(get_supervoxel_index(contact_point1), ang_component1);
    int ang_component2=node_angle_to_angle_component.at(std::pair<int, int>(get_supervoxel_index(contact_point2), secondary_desired_angle));
    std::pair<int, int> regrasp_node_goal2=std::pair<int, int>(get_supervoxel_index(contact_point2), ang_component2);

    for(int idx=0; idx<regrasping_candidate_nodes[0].size() && !valid_configuration; idx++){
        std::cout<<"Iteration: "<<idx<<std::endl;
        //current values:
        pcl::PointXYZRGBA p=all_centroids_cloud->at(supervoxel_to_pc_idx.at(regrasping_candidate_nodes[0][idx].first));
        regrasp1_principal<<p.x, p.y, p.z;
        std::cout<<"CURRENT NODE: "<<regrasping_candidate_nodes[0][idx].first<<"   i.e.   "<<regrasp1_principal.transpose()<<std::endl;
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


        //second gripper regrasping area:
        nodes_distances_from_regrasps=weight_regrasping_area(release_contact_1, release_contact_2, regrasp1_principal, regrasp1_secondary);
        std::cout<<"--done!"<<std::endl;

        //check if there is at least one possible pose! If not, movements of the fingers must be considered (TODO). For now, return -1
        if(regrasp_poses_distance_values.size()<1){
            std::cout<<"No valid regrasp poses are directly achievable. More in-hand motions are required."<<std::endl;
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

        //store the values in eigen vectors:
        regrasp2_principal(0)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node1]).x;
        regrasp2_principal(1)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node1]).y;
        regrasp2_principal(2)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node1]).z;

        regrasp2_secondary(0)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node2]).x;
        regrasp2_secondary(1)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node2]).y;
        regrasp2_secondary(2)=all_centroids_cloud->at(supervoxel_to_pc_idx[regrasp2_node2]).z;

        //loop over all the possible angles (this could take time)
        for(std::set<int>::iterator it=node_component_to_angles_subset.at(regrasping_candidate_nodes[0][idx]).begin(); it!=possible_angles.at(regrasping_candidate_nodes[0][idx].first).end(); it++){
            regrasp1_angle=*it;
            //check if in this regrasp point the gripper is free of collisions
            int free_regrasp_angle=get_collision_free_regrasp_angle(regrasp2_principal, regrasp2_secondary, release_contact_1, release_contact_2, release_angle, regrasp1_principal, regrasp1_secondary, regrasp1_angle);
            if(free_regrasp_angle>=0){
                //there is a good collision-free angle!
                regrasp2_angle=M_PI*float(free_regrasp_angle/180.0);
                std::cout<<"second gripper regrasp angle: "<<regrasp2_angle<<std::endl;
                valid_configuration=true;
                break;
            }
            else{
                //A lot of TODOS
                //for now skip
                //std::cout<<"The candidate helper regrasp is not free of collisions. More in-hand motions are required"<<std::endl;
                // 1) move the 1st gripper regrasp angle (can be done in big intervals or randomly...)
            }
        }

    }

    //if after all this, no success, oh well...
    if(!valid_configuration){
        return -1;
    }

    //get the path from of the regrasping thing. Only if it is not direct regrasp
    if(!direct_regrasp_1||angle_in_collision){
        int regrasp1_secondary_node=get_supervoxel_index(regrasp1_secondary);
        int regrasp1_angle_secondary= pose_to_angle(angle_to_pose(M_PI*float(regrasp1_angle)/180.0, regrasp1_principal), nodes_to_connected_component.at(regrasp1_secondary_node));
        std::pair<std::stack<std::pair<int, int>>, std::stack<int>> regrasp_path=ShapeAnalyzer::get_extended_path(extended_refined_adjacency, get_supervoxel_index(regrasp1_principal), regrasp_node_goal1.first, contact_point2-contact_point1, std::pair<int, int>(regrasp1_secondary_node, regrasp1_angle_secondary), regrasp_node_goal2, regrasp1_angle, principal_desired_angle);
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
        compute_extended_angle_sequence(vector_regrasp_path, finger_id, regrasp1_angle, principal_desired_angle);
        r_rotations[2]=angle_sequence;

        //for now do not care about the distances
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
    int supervoxel_idx=get_supervoxel_index(contact);
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
    std::cout<<"here in the processing: 1"<<std::endl;
    Eigen::Vector3f line=secondary_contact-principal_contact;

    //get the closest node to the principal contact point
    int principal_supervoxel_idx=get_supervoxel_index(principal_contact);
    //get the index of the angle component
    std::cout<<"principal angle: "<<principal_angle<<std::endl;
    int principal_angle_component=node_angle_to_angle_component.at(std::pair<int, int>(principal_supervoxel_idx, principal_angle));
    std::cout<<"here in the processing: 2"<<std::endl;

    //get the extended connected component
    int principal_cc=extended_nodes_to_connected_component.at(std::pair<int, int>(principal_supervoxel_idx, principal_angle_component));
    std::cout<<"here in the processing: 3"<<std::endl;


    //do the same to get the secondary connected component
    int secondary_supervoxel_idx=get_supervoxel_index(secondary_contact);

    int secondary_angle_component=node_angle_to_angle_component.at(std::pair<int, int>(secondary_supervoxel_idx, secondary_angle));
    int secondary_cc=extended_nodes_to_connected_component.at(std::pair<int, int>(secondary_supervoxel_idx, secondary_angle_component));
    std::cout<<"here in the processing: 4"<<std::endl;


    //now to a BFS starting from the given contact, until we find points from which it is possible to regrasp
    std::queue<std::pair<int, int>> Q;
    std::set<std::pair<int, int>> visited_nodes;
    std::pair<int, int> n_init=std::pair<int, int>(principal_supervoxel_idx, principal_angle_component);
    Q.push(n_init);
    visited_nodes.insert(n_init);
    std::cout<<"here in the processing: 5"<<std::endl;

    while(Q.size()>0){
        std::pair<int, int> n=Q.front();
        Q.pop(); //remove the first element, that is now n
        //visit all the children (neighbor of n)
        std::multimap<std::pair<int, int>, std::pair<int, int>>::iterator label_itr=extended_refined_adjacency.begin();
        for ( ; label_itr!=extended_refined_adjacency.end();) {
            std::pair<int, int> child=label_itr->first;
            //check if this child has already been explored
            if(visited_nodes.find(child)==visited_nodes.end()){
                visited_nodes.insert(child);
                //now add this to the queue only if the secondary finger there can be in a valid configuration
                //this can also be added only up to a certain distance from the contact, to keep the regrasping area limited (when possible)
                std::list<std::pair<int, int>> secondary_nodes=get_opposite_finger_nodes(line, n);
                for(std::pair<int, int>& sec_n: secondary_nodes){
                    //check if this is in the correct connected component
                    int node_cc=extended_nodes_to_connected_component.at(sec_n);
                    if(node_cc==secondary_cc){
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
                        regrasping_candidate_nodes[gripper_id].push_back(child);
                    }
                }
            }
        }
    }
    std::cout<<"LOOP ENDED"<<std::endl<<std::endl;
}

int ExtendedDMG::get_supervoxel_index(Eigen::Vector3f contact){
    //put the contact in pcl point
    pcl::PointXYZRGBA input;
    input.x=contact(0);
    input.y=contact(1);
    input.z=contact(2);

    //now start searching for the nearest neighbor
    int K=1;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    double min_dist=DBL_MAX;
    int min_dist_idx=-1;
    if (centroids_kdtree.nearestKSearch(input, K, pointIdxNKNSearch, pointNKNSquaredDistance)> 0){
        return int(pc_to_supervoxel_idx.at(pointIdxNKNSearch[0])); //the first and only found index is the nearest neighbor
    }
    else{
        std::cout<<"No neighbour found."<<std::endl;
        return -1;
    }
}

Eigen::Vector3f ExtendedDMG::get_normal_at_contact(Eigen::Vector3f contact){
    //get the index of the supervoxel
    int idx=get_supervoxel_index(contact);
    //get the normal of that component
    Eigen::Vector3f normal=component_to_average_normal.at(nodes_to_connected_component.at(idx));
    return normal;
}

Eigen::Vector3f ExtendedDMG::get_zero_angle_direction(Eigen::Vector3f contact){
    Eigen::Vector3f nx=component_to_average_normal.at(nodes_to_connected_component.at(get_supervoxel_index(contact)));
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
    Eigen::Matrix3f pose_component=component_pose_matrix(component);

    Eigen::Matrix3f gripper_to_component=pose_component.transpose()*pose_gripper;
    Eigen::Vector3f x_prime=gripper_to_component.block<3, 1>(0, 0);
    double angle=atan2(-x_prime(2), -x_prime(1));
    std::cout<<"angle: "<<angle<<std::endl;
    int int_angle=filter_angle(angle);
    std::cout<<"int_angle: "<<int_angle<<std::endl;
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
    std::cout<<"angle degree: "<<angle<<std::endl;
    double round_angle=angle+angle_jump/2;
    int int_angle=floor(round_angle);
    int_angle=int_angle-(int_angle%angle_jump);
    std::cout<<"int_angle: "<<int_angle<<std::endl;
    if(int_angle%angle_jump!=0){
        std::cout<<"ERROR IN THE ROUNDING TO "<<angle_jump<<std::endl;
    }
    if(int_angle==360){
        int_angle=0;
    }
    return int_angle;
}


std::list<std::pair<int, int>> ExtendedDMG::get_opposite_finger_nodes(Eigen::Vector3f direction, std::pair<int, int> principal_node){
    std::list<std::pair<int, int>> output_list;
    //the current considered point
    Eigen::Vector3f current_point; 
    current_point<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(principal_node.first)).x, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(principal_node.first)).y, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(principal_node.first)).z;
    
    //scale it to mm
    current_point=1000.0*current_point;
    //raise this point of 1cm for the start
    Eigen::Vector3f start=current_point-10.0*direction;
    //now something 50 cm away for the intersections
    Eigen::Vector3f end=current_point+500.0*direction; 

    //get all the intersections! Many possibilities for opposite fingers sometimes
    std::vector<Eigen::Vector3f> intersections=get_ray_intersections(start, end);
    pcl::PointXYZRGBA input;
    int K = 1;//three nearest neighbours
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    for(Eigen::Vector3f& v: intersections){
        //get the closest supervoxel
        input.x=v(0);
        input.y=v(1);
        input.z=v(2);
        if(centroids_kdtree.nearestKSearch(input, K, pointIdxNKNSearch, pointNKNSquaredDistance)> 0){
            int supervoxel_idx=pointIdxNKNSearch[0]; //there is only one
            //TODO get the possible angular components and check if their intersection with the ranges in the principal finger is good
            std::vector<int> angle_components=node_to_angle_components.at(supervoxel_idx);
            for(int &angle_c_idx: angle_components){
                output_list.push_back(std::pair<int, int>(supervoxel_idx, angle_c_idx));
            }
        }
    }

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


                //check if it is possible to grasp at the first contact
                Eigen::Vector3f end_point_ray=c-1000.0*component_normal; //this point is the final point for the ray, from c1 outwards (opposite to line) of 1000 (1m)
                Eigen::Vector3f init_point_ray=c+1000.0*component_normal; //this point is the final point for the ray, from c1 outwards (opposite to line) of 1000 (1m)
                
                std::vector<Eigen::Vector3f> intersections=get_ray_intersections(init_point_ray, end_point_ray);
                if(intersections.size()>0){
                    for(int idx=0; idx<intersections.size(); idx++){
                        Eigen::Vector3f point_i=intersections[idx];
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
                                int opposite_node=get_supervoxel_index(point_i);
                                if(regrasp_area_values.count(opposite_node)<1){
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
                                    for(int angle_comp:node_to_angle_components.at(opposite_node)){
                                        regrasping_candidate_nodes[1].push_back(std::pair<int, int>(opposite_node, angle_comp));
                                    }
                                    //put these two points (without angle component for now) in the regrasp value map
                                    regrasp_poses_distance_values[std::pair<int, int>(n, opposite_node)]=cumulative_d;
                                }
                            }
                            else{
                                //this node is not valid and so is the opposite component
                                if(regrasp_area_values.count(int(n))<1){
                                    regrasp_area_values.insert(std::pair<int, double>(n, 0.0));
                                }
                                int opposite_node=get_supervoxel_index(point_i);
                                if(regrasp_area_values.count(opposite_node)<1){
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

void ExtendedDMG::visualize_results(){
    //first of all add the shape to the viewer
    //viewer->addModelFromPolyData(colorable_shape, "object");
    std::cout<<"green spheres: "<<desired_pose_1.transpose()<<std::endl;
    std::cout<<"green spheres: "<<desired_pose_2.transpose()<<std::endl;
    std::cout<<"red spheres: "<<initial_pose_1.transpose()<<std::endl;
    std::cout<<"red spheres: "<<initial_pose_2.transpose()<<std::endl;
    std::cout<<"dark green spheres: "<<regrasp1_principal.transpose()<<std::endl;
    std::cout<<"dark green spheres: "<<regrasp2_principal.transpose()<<std::endl;
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
    std::cout<<"initial pose 1: "<<initial_pose_1.transpose()<<std::endl;
    std::cout<<"initial pose 2: "<<initial_pose_2.transpose()<<std::endl;
    draw_finger("finger1_principal", initial_pose_1.block<3, 1>(0, 0)*1000.0, Eigen::Quaternionf(initial_pose_1(6, 0), initial_pose_1(3, 0), initial_pose_1(4, 0), initial_pose_1(5, 0)), 0);
    draw_finger("finger1_secondary", initial_pose_2.block<3, 1>(0, 0)*1000.0, Eigen::Quaternionf(initial_pose_2(6, 0), initial_pose_2(3, 0), initial_pose_2(4, 0), initial_pose_2(5, 0)), 0);
    //add the two desired finger poses
    std::cout<<"desired pose 1: "<<desired_pose_1.transpose()<<std::endl;
    std::cout<<"desired pose 2: "<<desired_pose_2.transpose()<<std::endl;
    draw_finger("finger3_principal", desired_pose_1.block<3, 1>(0, 0)*1000.0, Eigen::Quaternionf(desired_pose_1(6, 0), desired_pose_1(3, 0), desired_pose_1(4, 0), desired_pose_1(5, 0)), 2);
    draw_finger("finger3_secondary", desired_pose_2.block<3, 1>(0, 0)*1000.0, Eigen::Quaternionf(desired_pose_2(6, 0), desired_pose_2(3, 0), desired_pose_2(4, 0), desired_pose_2(5, 0)), 2);
    //add the regrasp pose (first, need to get the proper orientation!)
    Eigen::Quaternionf regrasp_orientation=angle_to_pose(regrasp2_angle, regrasp2_principal);
    std::cout<<"regrasp pose 1: "<<regrasp2_principal.transpose()<<" "<<regrasp_orientation.x()<<" "<<regrasp_orientation.y()<<" "<<regrasp_orientation.z()<<" "<<regrasp_orientation.w()<<std::endl;
    std::cout<<"regrasp pose 2: "<<regrasp2_secondary.transpose()<<" "<<regrasp_orientation.x()<<" "<<regrasp_orientation.y()<<" "<<regrasp_orientation.z()<<" "<<regrasp_orientation.w()<<std::endl;
    draw_finger("finger2_principal", regrasp2_principal, regrasp_orientation, 1);
    draw_finger("finger2_secondary", regrasp2_secondary, regrasp_orientation, 1);

    //loop through the nodes and add spheres with a radius normalized according to the bounding box
    pcl::PointXYZ max, min;
    pcl::getMinMax3D(*object_shape, min, max);
    double normalizing_factor=std::max(std::max(max.x-min.x, max.y-min.y), max.z-min.z);
    double radius;
    for(std::pair<int, int> n:regrasping_candidate_nodes[1]){
        //now check the distance and normalize it w.r.t. the maximum value of the bounding box
        radius=10.0*nodes_distances_from_regrasps.at(n.first)/normalizing_factor;
        //if this node is one of the two that are perfect candidates for regrasping, then put them in yellow
        if(n.first==regrasp2_node1||n.first==regrasp2_node2){
            regrasp_viewer->addSphere(all_centroids_cloud->at(supervoxel_to_pc_idx.at(n.first)), radius, 1.0, 1.0, 0.0, "node_"+std::to_string(n.first));
        }
        else{
            regrasp_viewer->addSphere(all_centroids_cloud->at(supervoxel_to_pc_idx.at(n.first)), radius, 0.0, 0.0, 1.0, "node_"+std::to_string(n.first));        
        }
    }

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
    std::cout<<"the normal is outwards"<<std::endl;
        return false;
    }
    std::cout<<"the normal is inwards"<<std::endl;
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
    //the nx axis corresponds to the y axis of the gripper (this could be inverted according to which finger is set as principal finger)
    Eigen::Vector3f nx=get_normal_at_contact(contact);
    Eigen::Vector3f ny=get_orthogonal_axis(nx);

    //now rotate ny around nx of -angle
    Eigen::AngleAxis<float> aa(-angle*M_PI/180.0, nx);
    Eigen::Vector3f t_ny=aa*ny;

    //get the third axis
    Eigen::Vector3f t_nz=nx.cross(t_ny);

    //fill a matrix
    Eigen::Matrix3f component_matrix;
    component_matrix(0,0)=nx(0);
    component_matrix(1,0)=nx(1);
    component_matrix(2,0)=nx(2);

    component_matrix(0,1)=t_ny(0);
    component_matrix(1,1)=t_ny(1);
    component_matrix(2,1)=t_ny(2);

    component_matrix(0,2)=t_nz(0);
    component_matrix(1,2)=t_nz(1);
    component_matrix(2,2)=t_nz(2);

    component_matrix.transposeInPlace();

    //get the quaternion corresponding to this matrix
    Eigen::Quaternion<float> q(component_matrix);
    return q;

}

int ExtendedDMG::get_collision_free_regrasp_angle(Eigen::Vector3f contact1_principal, Eigen::Vector3f contact1_secondary, Eigen::Vector3f point1_principal, Eigen::Vector3f point1_secondary, int grasping_angle1, Eigen::Vector3f point2_principal, Eigen::Vector3f point2_secondary, int &grasping_angle2){
    int best_possible_angle=-1;

    //create the first rectangle in 3D:
    Eigen::Vector3f v11=point1_principal;
    Eigen::Vector3f v12=point1_secondary;
    //for the other 2 vertices, get the normal to the surface at the 1st contact point and check the 0 angle direction
    Eigen::Vector3f normal=get_normal_at_contact(v11);
    Eigen::Vector3f zero_axis=get_zero_angle_direction(v11);
    //now rotate this axis of the grasping angle around the normal
    Eigen::Matrix3f R_axis_angle=axis_angle_matrix(normal, grasping_angle1);
    Eigen::Vector3f direction=R_axis_angle*zero_axis; //double check if it is correct
    //now get the two other vertices of the rectangle, from the first two along this direction at distance of finger length
    Eigen::Vector3f v13=v12+l_finger*direction;
    Eigen::Vector3f v14=v11+l_finger*direction;
    std::vector<Eigen::Vector3f> rectangle1;
    rectangle1.push_back(v11);
    rectangle1.push_back(v12);
    rectangle1.push_back(v13);
    rectangle1.push_back(v14);

    //create the second rectangle in 3D:
    Eigen::Vector3f v21=point2_principal;
    Eigen::Vector3f v22=point2_secondary;
    //obtain the other 2 vertices with the same procedure as before:
    normal=get_normal_at_contact(v21);
    zero_axis=get_zero_angle_direction(v21);
    R_axis_angle=axis_angle_matrix(normal, grasping_angle1);
    direction=R_axis_angle*zero_axis; //double check if it is correct
    Eigen::Vector3f v23=v22+l_finger*direction;
    Eigen::Vector3f v24=v21+l_finger*direction;
    std::vector<Eigen::Vector3f> rectangle2;
    rectangle2.push_back(v21);
    rectangle2.push_back(v22);
    rectangle2.push_back(v23);
    rectangle2.push_back(v24);

    //Now, loop through all the possible candidate regrasping angles to check if some are collision-free
    std::set<int> all_angles=possible_angles.at(get_supervoxel_index(contact1_principal));
    //the first two verticese will always be the same
    Eigen::Vector3f v31=contact1_principal;
    Eigen::Vector3f v32=contact1_secondary;
    normal=get_normal_at_contact(v31);
    zero_axis=get_zero_angle_direction(v31);
    Eigen::Vector3f v33, v34;
    std::vector<Eigen::Vector3f> rectangle3;
    rectangle3.push_back(v31);
    rectangle3.push_back(v32);
    rectangle3.push_back(v33);
    rectangle3.push_back(v34);
    float max_clearence=0;
    // std::cout<<"ALL ANGLES: "<<all_angles.size()<<std::endl;
    for(int alpha : all_angles){
        // std::cout<<" ... testing: "<<alpha;
        //the second vertices depend on the current angle. Obtained as before
        R_axis_angle=axis_angle_matrix(normal, M_PI*float(alpha)/180.0);
        direction=R_axis_angle*zero_axis;
        v33=v32+l_finger*direction;
        v34=v31+l_finger*direction;
        rectangle3[2]=v33;
        rectangle3[3]=v34;

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
            float clearence=clearence1+clearence2 - std::max(clearence1, clearence2);
            // std::cout<<"  --clearence: "<<clearence<<std::endl;
            if(clearence>max_clearence){
                max_clearence=clearence;
                best_possible_angle=alpha;
            } 

        }

    }
    //TODO check if moving the second regrasping angle improves the situation
    return best_possible_angle;
}

void ExtendedDMG::draw_finger(std::string name, Eigen::Vector3f position, Eigen::Quaternion<float> orientation, int color_profile){
    regrasp_viewer->removeShape(name);
    Eigen::Matrix3f cube_rot_matrix=orientation.toRotationMatrix();
    Eigen::Vector3f cube_pos=position+cube_rot_matrix*Eigen::Vector3f(-l_finger/2+1.5, 0, 0);
    regrasp_viewer->addCube(cube_pos, orientation, l_finger, 6, 6, name);
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
    }

    regrasp_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name);
    regrasp_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);

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
    Eigen::Matrix3f R;
    double x=axis(0);
    double y=axis(1);
    double z=axis(2);

    double cos_th=cos(theta);
    double sin_th=sin(theta);

    R<< cos_th + x*x * (1-cos_th), x*y* (1-cos_th) - z*sin_th, x*z* (1-cos_th) + y*sin_th,
        y*x* (1-cos_th) + z*sin_th, cos_th + y*y* (1-cos_th), y*z* (1-cos_th) - x*sin_th,
        z*x* (1-cos_th) - y*sin_th, z*y* (1-cos_th) + x*sin_th, cos_th + z*z* (1-cos_th);

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
