/**
 *  extended_dmg.cpp
 *
 *  Created on: March 14 2018
 *      Author: Silvia Cruciani
*/

#include "shape_analysis/extended_dmg.hpp"

using namespace shape_analysis;

//helper functions 
Eigen::Matrix3f axis_angle_matrix(Eigen::Vector3f axis, double theta);

ExtendedDMG::ExtendedDMG(ros::NodeHandle n) : ShapeAnalyzer(){
    r_translations=std::vector<std::vector<geometry_msgs::Point>>(3);
    r_rotations=std::vector<std::vector<double>>(3);
    r_finger_distances=std::vector<std::vector<double>>(3);
    ray_tracing_service_name="ray_tracing";
    angle_collision_service_name="collision_check";
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
        //collision check with the desired angle: check if the desired angle is reachable
        angle_in_collision=is_in_collision(contact_point1, double(principal_desired_angle)*M_PI/180.0);
        if (!angle_in_collision){
            //check if the slave finger is also not in collision
            angle_in_collision=is_in_collision(contact_point1, double(secondary_desired_angle)*M_PI/180.0);
        }

        if(!angle_in_collision){
            //this is the best case
            r_rotations[2]=std::vector<double>();
            r_finger_distances[2]=std::vector<double>();
            r_translations[2]=std::vector<geometry_msgs::Point>();
            //the regrasp of the 1st gripper is exactly the desired grasp
            regrasp1_principal=contact_point1;
            regrasp1_secondary=contact_point2;
            return 1;
        }
    }
    
    //call this for the first gripper
    find_available_regrasping_points(contact_point1, contact_point2, principal_desired_angle, secondary_desired_angle, 0);

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

bool ExtendedDMG::is_in_collision(Eigen::Vector3f contact, double angle){
    //get the normal to the surface at the contact point
    Eigen::Vector3f normal=get_normal_at_contact(contact);
    //get two points on the vertical line, raised a bit to not be in collision with the contact surface
    Eigen::Vector3f p1, p2, p3, p4;
    double delta1=2.0; //2mm
    double delta2=12.0;
    p1=contact+delta1*normal;
    p2=contact+delta2*normal;

    //now get the other two points given the angle
    Eigen::Vector3f zero_axis=get_zero_angle_direction(contact);
    //rotate this axis of angle around the normal
    Eigen::Matrix3f R_axis_angle=axis_angle_matrix(normal, angle);
    Eigen::Vector3f direction=R_axis_angle*zero_axis; //double check if it is correct
    p3=contact+l_finger*direction + delta2*normal;
    p4=contact+l_finger*direction + delta1*normal;

    //now do the service call!
    geometry_msgs::Point point1, point2, point3, point4;

    point1.x=p1(0);
    point1.y=p1(1);
    point1.z=p1(2);

    point2.x=p2(0);
    point2.y=p2(1);
    point2.z=p2(2);

    point3.x=p3(0);
    point3.y=p3(1);
    point3.z=p3(2);

    point4.x=p4(0);
    point4.y=p4(1);
    point4.z=p4(2);

    //get the mesh name from the object's name
    std::string name=object_name.substr(0, object_name.find("_"))+".stl";

    CollisionCheck srv;
    srv.request.mesh_name=name;
    srv.request.v1=point1;
    srv.request.v2=point2;
    srv.request.v3=point3;
    srv.request.v4=point4;

    //call the server
    if(angle_collision_client.call(srv)){
        return srv.response.collision;
    }

    ROS_ERROR("Failed to call the service %s", angle_collision_service_name.c_str());

    //assume collision
    return true;
}

void ExtendedDMG::find_available_regrasping_points(Eigen::Vector3f principal_contact, Eigen::Vector3f secondary_contact, int principal_angle, int secondary_angle, int gripper_id){
    //get the grasp line first (to check for the secondary finger following the principal finger)
    Eigen::Vector3f line=secondary_contact-principal_contact;

    //get the closest node to the principal contact point
    int principal_supervoxel_idx=get_supervoxel_index(principal_contact);
    //get the index of the angle component
    int principal_angle_component=node_angle_to_angle_component.at(std::pair<int, int>(principal_supervoxel_idx, principal_angle));
    //get the extended connected component
    int principal_cc=extended_nodes_to_connected_component.at(std::pair<int, int>(principal_supervoxel_idx, principal_angle));

    //do the same to get the secondary connected component
    int secondary_supervoxel_idx=get_supervoxel_index(secondary_contact);
    int secondary_angle_component=node_angle_to_angle_component.at(std::pair<int, int>(secondary_supervoxel_idx, secondary_angle));
    int secondary_cc=extended_nodes_to_connected_component.at(std::pair<int, int>(secondary_supervoxel_idx, secondary_angle));

    //now to a BFS starting from the given contact, until we find points from which it is possible to regrasp
    std::queue<std::pair<int, int>> Q;
    std::set<std::pair<int, int>> visited_nodes;
    std::pair<int, int> n_init=std::pair<int, int>(principal_supervoxel_idx, principal_angle);
    Q.push(n_init);
    visited_nodes.insert(n_init);
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
    Eigen::Vector3f normal=component_to_average_normal.at(idx);
    return normal;
}

Eigen::Vector3f ExtendedDMG::get_zero_angle_direction(Eigen::Vector3f contact){
    Eigen::Vector3f nx=component_to_average_normal.at(get_supervoxel_index(contact));
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
    int int_angle=filter_angle(angle);

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
    double round_angle=angle+angle_jump/2;
    int int_angle=floor(round_angle);
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

