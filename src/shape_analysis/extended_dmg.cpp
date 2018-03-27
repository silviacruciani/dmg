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
        //collision check with the desired angle: check if the desired angle is reachable



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

void ExtendedDMG::find_available_regrasping_points(Eigen::Vector3f principal_contact, Eigen::Vector3f secondary_contact, bool desired_angle){
    //get the grasp line first (to check for the secondary finger following the principal finger)
    Eigen::Vector3f line=secondary_contact-principal_contact;
    //get the closest node to the principal contact point
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
        return pointIdxNKNSearch[0]; //the first and only found index is the nearest neighbor
    }
    else{
        std::cout<<"No neighbour found."<<std::endl;
        return -1;
    }
}

Eigen::Vector3f ExtendedDMG::get_normal_at_contact(Eigen::Vector3f contact){
    //get the index of the supervoxel
    int idx=get_supervoxel_index(contact);
    //get the normal of that supervoxel
    pcl::Normal n=all_normals_clouds->at(idx);
    //convert it into a Vector3f
    Eigen::Vector3f normal(n.normal_x, n.normal_y, n.normal_z);
    return normal;
}

Eigen::Vector3f ExtendedDMG::get_zero_angle_direction(Eigen::Vector3f contact){
    Eigen::Vector3f nx=component_to_average_normal.at(get_supervoxel_index(contact));
    Eigen::Vector3f ny; //ny is the axis at which the angle is 0, for all the nodes
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