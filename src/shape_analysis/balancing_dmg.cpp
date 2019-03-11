/**
 *  balancing_dmg.cpp
 *
 *  Created on: March 7 2019
 *      Author: Silvia Cruciani
*/

#include "shape_analysis/balancing_dmg.hpp"
#include <cstdlib> 
#include <ctime>
#include <fstream> 

using namespace shape_analysis;

BalancingDMG::BalancingDMG(){

}

BalancingDMG::~BalancingDMG(){

}

Eigen::Vector3f zeroAngleDirection(Eigen::Vector3f nx){
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

Eigen::Matrix3f axisAngle(Eigen::Vector3f axis, double angle){
    Eigen::Matrix3f rrt=axis*axis.transpose();
    Eigen::Matrix3f Sr;
    Sr<<0, -axis(2), axis(1),
        axis(2), 0, -axis(0),
        -axis(1), axis(0), 0;
    Eigen::Matrix3f R=rrt+(Eigen::Matrix3f::Identity()-rrt)*cos(angle)+Sr*sin(angle);
    return R;
}

void BalancingDMG::drawAllFingers(){
    std::cout<<"drawing all the fingers..."<<std::endl;
    // std::cout<<"num_extended_connected_components: "<<num_extended_connected_components<<std::endl;
    srand(time(0));
    //draw the fingers for each connected component
    for(int i=0; i<num_extended_connected_components; i++){
        // std::cout<<"i: "<<i<<std::endl;
        if(extended_connected_component_to_set_of_nodes_angle.count(i)>0){
            // std::cout<<"here"<<std::endl;
            std::set<std::pair<int, int>> component_nodes=extended_connected_component_to_set_of_nodes_angle.at(i);
            // std::cout<<"woops"<<std::endl;
            //only draw if the component has more than one node
            if(component_nodes.size()>1){
                //get the component's reference frame
                Eigen::Vector3f nx=component_to_average_normal.at(nodes_to_connected_component.at(component_nodes.begin()->first));
                Eigen::Vector3f ny=zeroAngleDirection(nx);
                Eigen::Vector3f nz=nx.cross(ny);
                for(std::pair<int, int> n : component_nodes){
                    // std::cout<<"node: "<<n.first<<" "<<n.second<<std::endl;
                    std::set<int> angles=node_component_to_angles_subset.at(n);
                    int r, g, b;
                    r=rand()%100;
                    g=rand()%100;
                    b=rand()%100;
                    // std::cout<<r<<" "<<g<<" "<<b<<std::endl;
                    for(int angle : angles){
                        //now, get the position and the 3D orientation
                        Eigen::Matrix3f pose=axisAngle(nx, double(angle)*M_PI/180.0);
                        Eigen::Vector3f position;
                        position(0)=all_centroids_cloud->at((supervoxel_to_pc_idx.at(n.first))).x;
                        position(1)=all_centroids_cloud->at((supervoxel_to_pc_idx.at(n.first))).y;
                        position(2)=all_centroids_cloud->at((supervoxel_to_pc_idx.at(n.first))).z;
                        Eigen::Quaternion<float> q(pose);
                        draw_finger("node"+std::to_string(n.first)+"_"+std::to_string(angle), position, q, double(r)/100.0, double(g)/100.0, double(b)/100.0);
                    }
                }
            }
        }
    }

}

void BalancingDMG::saveDMGComponents(std::string file_path, std::string file_name){
    //we need to save: 1- the adjacency list 2- the node to angle set 3- the node to component -4 the component to average normal -5 node to position
    std::string adjacency_file=file_path+"graph_"+file_name;
    std::string node_angle_file=file_path+"node_angle_"+file_name;
    std::string node_component_file=file_path+"node_component_"+file_name;
    std::string component_normal_file=file_path+"component_normal_"+file_name;
    std::string node_position_file=file_path+"node_position_"+file_name;
    std::string node_angle_angle_component_file=file_path+"node_angle_angle_component_"+file_name;

    std::ofstream adj_f, na_f, nc_f, cn_f, np_f, nac_f;
    adj_f.open(adjacency_file);
    na_f.open(node_angle_file);
    nc_f.open(node_component_file);
    cn_f.open(component_normal_file);
    np_f.open(node_position_file);
    nac_f.open(node_angle_angle_component_file);

    for(int i=0; i<num_extended_connected_components; i++){
        if(extended_connected_component_to_set_of_nodes_angle.count(i)>0){
            //get all the nodes in the component
            std::set<std::pair<int, int>> component_nodes=extended_connected_component_to_set_of_nodes_angle.at(i);
            if(component_nodes.size()>1){
                //print the average normal
                Eigen::Vector3f nx=component_to_average_normal.at(nodes_to_connected_component.at(component_nodes.begin()->first));
                cn_f<<i<<" "<<nx(0)<<" "<<nx(1)<<" "<<nx(2)<<std::endl;
                for(std::pair<int, int> n : component_nodes){
                    //print the component of this node
                    nc_f<<n.first<<" "<<n.second<<" "<<i<<std::endl;
                    //print the position of this node
                    Eigen::Vector3f position;
                    position(0)=all_centroids_cloud->at((supervoxel_to_pc_idx.at(n.first))).x;
                    position(1)=all_centroids_cloud->at((supervoxel_to_pc_idx.at(n.first))).y;
                    position(2)=all_centroids_cloud->at((supervoxel_to_pc_idx.at(n.first))).z;
                    position=position/1000.0;
                    np_f<<n.first<<" "<<n.second<<" "<<position(0)<<" "<<position(1)<<" "<<position(2)<<std::endl;
                    //print all the angles of this node
                    na_f<<n.first<<" "<<n.second;
                    std::set<int> angles=node_component_to_angles_subset.at(n);
                    for(int angle : angles){
                        na_f<<" "<<angle;
                        int angle_component=node_angle_to_angle_component.at(std::pair<int, int>(n.first, angle));
                        nac_f<<n.first<<" "<<angle<<" "<<angle_component<<std::endl;
                    }
                    na_f<<std::endl;
                    //print all the neighbors of this node
                    adj_f<<n.first<<" "<<n.second;
                    std::multimap<std::pair<int, int>, std::pair<int, int>>::iterator adjacent_itr=extended_refined_adjacency.equal_range(n).first;
                    for ( ; adjacent_itr!=extended_refined_adjacency.equal_range(n).second; adjacent_itr++){
                        //if this element is still in Q
                        adj_f<<" "<<adjacent_itr->second.first<<" "<<adjacent_itr->second.second;
                    }
                    adj_f<<std::endl;
                }

            }
        }
    }

    //close all the files
    adj_f.close();
    na_f.close();
    nc_f.close();
    cn_f.close();
    np_f.close();
    nac_f.close();
    std::cout<<"the DMG was saved into files"<<std::endl;

}

void BalancingDMG::draw_finger(std::string name, Eigen::Vector3f position, Eigen::Quaternion<float> orientation, double r, double g, double b){
    viewer->removeShape(name);
    // std::cout<<"drawing: "<<name<<std::endl;
    Eigen::Matrix3f cube_rot_matrix=orientation.toRotationMatrix();
    Eigen::Vector3f cube_pos=position+cube_rot_matrix*Eigen::Vector3f(-l_finger/2+1.5, 0, 0);
    viewer->addCube(cube_pos, orientation, l_finger, 6, 6, name, v2);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name, v2);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name, v2);

    //if the coordinate system should be visualized, uncomment the last row
    Eigen::Matrix4f transf=Eigen::Matrix4f::Identity();
    transf.block<3, 3>(0, 0)=orientation.toRotationMatrix();
    transf.block<3,1>(0, 3)=cube_pos;
    Eigen::Affine3f aff;
    aff.matrix()=transf;
    //viewer->addCoordinateSystem(3.0, aff, "frame_"+name);
}