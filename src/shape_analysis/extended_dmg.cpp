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



    //if it is possible to directly regrasp, then we are happy and we can fill the last sequence with empty vectors
    //otherwise, we should propagate backwards in the DMG to find a proper regrasping area
    if(direct_regrasp_1){
        std::cout<<"checking if the desired angle is in collision"<<std::endl;
        //collision check with the desired angle: check if the desired angle is reachable
        angle_in_collision=is_in_collision(contact_point1, double(principal_desired_angle)*M_PI/180.0);
        std::cout<<"is in collision: "<<angle_in_collision<<std::endl;
        if (!angle_in_collision){
            //check if the slave finger is also not in collision
            std::cout<<"checking if secondary angle is also not in collisiton"<<std::endl;
            angle_in_collision=is_in_collision(contact_point1, double(secondary_desired_angle)*M_PI/180.0);
            std::cout<<"is in collision: "<<angle_in_collision<<std::endl;

        }

        if(!angle_in_collision){
            //this is the best case
            r_rotations[2]=std::vector<double>();
            r_finger_distances[2]=std::vector<double>();
            r_translations[2]=std::vector<geometry_msgs::Point>();
            //the regrasp of the 1st gripper is exactly the desired grasp
            regrasp1_principal=contact_point1;
            regrasp1_secondary=contact_point2;
        }
        else{
            std::cout<<"MORE WORK MUST BE DONE! the 1st gripper cannot directly regrasp!"<<std::endl;
            //call this for the first gripper
            find_available_regrasping_points(contact_point1, contact_point2, principal_desired_angle, secondary_desired_angle, 0);

            std::cout<<"Found available regrasping points for the 1st gripper!"<<std::endl;

            //the first gripper must have a determined regrasp point, and so does the second gripper! for now it is only an area......

            //for now use the first node available among the regrasp candidates
            pcl::PointXYZRGBA p=all_centroids_cloud->at(regrasping_candidate_nodes[0][0].first);
            regrasp1_principal<<p.x, p.y, p.z;
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



    //we also must check if it is possible to release the grasp at the initial location, or if additional motions are required
    //This is a TODO for later, because for now one can assume that the object has just been grasped there, so release is always possible
    //it can be generalized "very easily" though
    std::cout<<"weighting regrasp area"<<std::endl;

    //second gripper regrasping area:
    nodes_distances_from_regrasps=weight_regrasping_area(release_contact_1, release_contact_2, regrasp1_principal, regrasp1_secondary);
    std::cout<<"--done!"<<std::endl;

    //check if there is at least one possible pose! If not, movements of the fingers must be considered. For now, return -1
    if(regrasp_poses_distance_values.size()<1){
        std::cout<<"No valid regrasp poses are directly achievable. More in-hand motions are required."<<std::endl;
        return -1;
    }
    //now get the regrasp pose with max value (las element in the map)
    std::map<std::pair<int, int>, double>::reverse_iterator r_it=regrasp_poses_distance_values.rbegin();

    //TODO additional checks for the angle are needed, but for now just get the position, not the orientation
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
                                    //if one of the distances is too close, then put 0 as value
                                    if(node_distance_1<regrasp_distance_threshold||node_distance_2<regrasp_distance_threshold){
                                        d_value=0.0;
                                    }

                                    regrasp_area_values.insert(std::pair<int, double>(n, d_value));
                                    cumulative_d=d_value;
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
                                    //if one of the distances is too close, then put 0 as value
                                    if(node_distance_1<regrasp_distance_threshold||node_distance_2<regrasp_distance_threshold){
                                        d_value=0.0;
                                    }

                                    regrasp_area_values.insert(std::pair<int, double>(opposite_node, d_value));
                                    cumulative_d+=d_value;
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

    //get the quaternion corresponding to this matrix
    Eigen::Quaternion<float> q(component_matrix);
    return q;

}


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

