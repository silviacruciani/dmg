/*
 *  shape_analyzer.cpp
 *
 *  Created on: October 24 2017
 *      Author: Silvia Cruciani
*/

#include "shape_analysis/shape_analyzer.hpp"
#include <iostream>
#include <math.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>


using namespace shape_analysis;

ShapeAnalyzer::ShapeAnalyzer(){
    //object_shape=pcl::PointCloud<pcl::PointXYZ>::Ptr();
    //set the default parameters
    disable_transform=true; //the transformation has to be disabled for organized pointclouds
    voxel_resolution=1.0;
    seed_resolution=20.0;
    color_importance=0.0;
    spatial_importance=0.9;
    normal_importance=1.0;
    refinement_iterations=8;
    angle_jump=5;
}

ShapeAnalyzer::~ShapeAnalyzer(){

}

void ShapeAnalyzer::set_object_from_pointcloud(std::string file_name){
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    if ( pcl::io::loadPCDFile(file_name, *in_cloud)<0){
        std::cout<<"Error loading model cloud."<<std::endl;
    }
    object_shape=in_cloud;
    //print the dimensions:
    pcl::PointXYZ max, min;
    pcl::getMinMax3D(*object_shape, min, max);
    std::cout<<"min: "<<min.x<<" "<<min.y<<" "<<min.z<<std::endl;
    std::cout<<"max: "<<max.x<<" "<<max.y<<" "<<max.z<<std::endl;
    object_center<<(max.x - min.x)/2.0, (max.y - min.y)/2.0, (max.z - min.z)/2.0;
    //visualizer to be used for debug purposes
    viewer=new pcl::visualization::PCLVisualizer ("3D Viewer");
    int v(0);
    v1=v;
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addCoordinateSystem(15.0);
    viewer->initCameraParameters();
    viewer->addText ("supervoxels and adjacency", 10, 10, "v1 text", v1);
    //viewer for debug
    //viewer->addPointCloud<pcl::PointXYZ> (object_shape, "object");
    //use a separate thread for the viewer (non blocking)
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object");
    //spin_viewer();
}

void ShapeAnalyzer::set_object_from_mesh(std::string file_name){

}

void ShapeAnalyzer::subdivide_object(double subcube_dimension){
    //get starting point from somewhere (now default 0, 0, 0)
    
    //get bounding box so that we do not divide forever
    pcl::PointXYZ max, min;
    pcl::getMinMax3D(*object_shape, min, max);
    //filter x

    //filter y

    //filter z

}

void ShapeAnalyzer::spin_viewer(){
    while(!viewer->wasStopped()){
        viewer->spinOnce(1);
    }
    std::cout<<"viewer closed"<<std::endl;
    viewer->close();

}

void ShapeAnalyzer::spin_viewer_once(){
    if(!viewer->wasStopped()){
        viewer->spinOnce(1);
    }
    else{
        viewer->close();
    }

}

void ShapeAnalyzer::get_supervoxels(){
    //use the supervoxels
    pcl::SupervoxelClustering<pcl::PointXYZRGBA> super(voxel_resolution, seed_resolution);
    if(disable_transform){
        super.setUseSingleCameraTransform (false);
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud=boost::shared_ptr <pcl::PointCloud<pcl::PointXYZRGBA>> (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::copyPointCloud(*object_shape,*cloud);
        

    super.setInputCloud(cloud);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);

    pcl::console::print_highlight ("Extracting supervoxels!\n");
    super.extract (supervoxel_clusters);
    pcl::console::print_info ("Found %d supervoxels. Refining...\n", supervoxel_clusters.size ());
    super.refineSupervoxels(refinement_iterations, supervoxel_clusters);
    voxel_centroid_cloud=super.getVoxelCentroidCloud();
    std::cout<<"VoxelCentroidCloud size: "<<voxel_centroid_cloud->points.size()<<std::endl;

    viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "voxel centroids");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

    labeled_voxel_cloud = super.getLabeledVoxelCloud ();
    viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels", v1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "labeled voxels");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

    //get the normals
    sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
    std::cout<<"VoxelNormalCloud size: "<<sv_normal_cloud->points.size()<<std::endl;
    viewer->addPointCloudNormals<pcl::PointNormal> (sv_normal_cloud,1, 5.0, "supervoxel_normals", v1);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr centroids_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_cloud(new pcl::PointCloud<pcl::Normal>);
    //iterate over the supervoxels to get the REAL centroids cloud (and a normal cloud that follows the same order)
    int pc_idx=0;
    for (auto const& x : supervoxel_clusters){
        //std::cout<<x.first<<" "<<pc_idx<<std::endl;
        pcl::PointXYZRGBA newpoint;
        newpoint.x=x.second->centroid_.x;
        newpoint.y=x.second->centroid_.y;
        newpoint.z=x.second->centroid_.z;
        newpoint.r=x.second->centroid_.r;
        newpoint.g=x.second->centroid_.g;
        newpoint.b=x.second->centroid_.b;
        newpoint.a=x.second->centroid_.a;
        centroids_cloud->push_back(newpoint);
        //normals
        pcl::Normal norm;
        norm.normal_x=x.second->normal_.normal_x;
        norm.normal_y=x.second->normal_.normal_y;
        norm.normal_z=x.second->normal_.normal_z;
        normals_cloud->push_back(norm);


        //necessary mapping from index in the point cloud to supervoxel label:
        pc_to_supervoxel_idx.insert(std::pair<int, uint32_t>(pc_idx, x.first));
        supervoxel_to_pc_idx.insert(std::pair<uint32_t, int>(x.first, pc_idx));
        centroids_ids.push_back(x.first);
        pc_idx++;
        //std::cout<<"CentroidsCloud size: "<<centroids_cloud->points.size()<<std::endl;
    }

    all_centroids_cloud=centroids_cloud;
    all_normals_clouds=normals_cloud;
    //generate the tree for nearest neighbor search
    centroids_kdtree.setInputCloud(all_centroids_cloud);

    pcl::console::print_highlight ("Getting supervoxel adjacency\n");
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
    for ( ; label_itr != supervoxel_adjacency.end ();) {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //if(supervoxel_label==27){
            //viewer->addSphere(supervoxel_clusters.at(supervoxel_label)->centroid_, 1, 0.0, 1.0, 0.0, std::to_string(supervoxel_label));
        //}
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; adjacent_itr++){
          pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
          adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
          //if(supervoxel_label==27){
            //std::cout<<"-----adjacent label: "<<adjacent_itr->second<<std::endl;
            //viewer->addSphere(neighbor_supervoxel->centroid_, 1, 1.0, 0.0, 0.0, std::to_string(adjacent_itr->second));
          //}
        }
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
        addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer, v1);
        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }

    //spin_viewer();
}

void ShapeAnalyzer::addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
                                  pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer* viewer, int viewport){

    vtkSmartPointer<vtkPoints> points=vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells=vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyLine> polyLine=vtkSmartPointer<vtkPolyLine>::New();

    //Iterate through all adjacent points, and add a center point to adjacent point pair
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
    for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr){
        points->InsertNextPoint (supervoxel_center.data);
        points->InsertNextPoint (adjacent_itr->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    // Add the points to the dataset
    polyData->SetPoints (points);
    polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
    for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++){
        polyLine->GetPointIds ()->SetId (i,i);
    }
    cells->InsertNextCell (polyLine);
    // Add the lines to the dataset
    polyData->SetLines (cells);
    viewer->addModelFromPolyData (polyData,supervoxel_name, viewport);
}

//bool ShapeAnalyzer::return_manipulation_sequence(){
    //get the pose of the two fingers of the gripper on the object (in the object frame)
    //coordinates to be used: x,y,z,yaw? normal? axis angle maybe?
//}

int ShapeAnalyzer::connect_centroid_to_contact(geometry_msgs::Point p, geometry_msgs::Quaternion q, std::string id, bool render_sphere){
    //convert the input into pcl point (from meters to millimeters in the object scale)
    pcl::PointXYZRGBA input;
    input.x=p.x*1000.0;
    input.y=p.y*1000.0;
    input.z=p.z*1000.0;

    //std::cout<<"connecting contact point: "<<id<<std::endl;

    //draw a sphere at the contact point
    //viewer->addSphere (input, 3, 1.0, 0.0, 0.0, "contact point");
    
    //now start searching for the K nearest neighbors. (then take the closest one if the normal matches)
    int K = 3;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    double min_dist=DBL_MAX;
    int min_dist_idx=-1;
    if (centroids_kdtree.nearestKSearch(input, K, pointIdxNKNSearch, pointNKNSquaredDistance)> 0){
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
          /*std::cout << "    "  <<   all_centroids_cloud->points[ pointIdxNKNSearch[i] ].x 
                    << " " << all_centroids_cloud->points[ pointIdxNKNSearch[i] ].y 
                    << " " << all_centroids_cloud->points[ pointIdxNKNSearch[i] ].z 
                    << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;*/
            //only based on distance now. later on normal too.
            if(pointNKNSquaredDistance[i]<min_dist){
                min_dist=pointNKNSquaredDistance[i];
                min_dist_idx=i;
            }
        
        }
        if(render_sphere){
            //std::cout<<"Rendering: "<<id<<std::endl;
            viewer->removeShape(id);
            viewer->addSphere(all_centroids_cloud->points[pointIdxNKNSearch[min_dist_idx]], 1, 0.0, 0.0, 1.0, id);
        }
    }
    else{
        std::cout<<"No neighbour found."<<std::endl;
        return -1;
    }

    //connect this into the label for the supervoxel:
    //std::cout<<"Nearest neighbour index: "<<pointIdxNKNSearch[min_dist_idx]<<std::endl;
    //std::cout<<"Corresponds to index: "<<pc_to_supervoxel_idx.at(pointIdxNKNSearch[min_dist_idx])<<std::endl;
    return int(pc_to_supervoxel_idx.at(pointIdxNKNSearch[min_dist_idx]));

}

void ShapeAnalyzer::set_initial_contact(geometry_msgs::Point p, geometry_msgs::Quaternion q, int finger_id){
    std::cout<<"set initial contact: "<<finger_id<<std::endl;
    pcl::PointXYZRGBA input;
    input.x=p.x*1000.0;
    input.y=p.y*1000.0;
    input.z=p.z*1000.0;
    //later quaternion


    if(finger_id==1){
        initial_centroid_idx_1=connect_centroid_to_contact(p, q, "initial centroid "+std::to_string(finger_id), true);
        initial_pose_1<<p.x, p.y, p.z, q.x, q.y, q.z, q.w;
    }
    else{
        initial_centroid_idx_2=connect_centroid_to_contact(p, q, "initial centroid "+std::to_string(finger_id), true);
        initial_pose_2<<p.x, p.y, p.z, q.x, q.y, q.z, q.w;
    }
    viewer->removeShape("initial contact "+std::to_string(finger_id));
    Eigen::Vector3f cube_pos(input.x, input.y, input.z);
    Eigen::Quaternionf cube_or(q.w, q.x, q.y, q.z);
    Eigen::Matrix3f cube_rot_matrix=cube_or.toRotationMatrix();
    //std::cout<<"matrix: "<<std::endl<<cube_rot_matrix<<std::endl;
    cube_pos=cube_pos+cube_rot_matrix*Eigen::Vector3f(-l_finger/2+1.5, 0, 0);
    viewer->addCube(cube_pos, cube_or, l_finger, 3, 3, "initial contact "+std::to_string(finger_id));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "initial contact "+std::to_string(finger_id));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "initial contact "+std::to_string(finger_id));

}

void ShapeAnalyzer::set_desired_contact(geometry_msgs::Point p, geometry_msgs::Quaternion q, int finger_id){
    pcl::PointXYZRGBA input;
    input.x=p.x*1000.0;
    input.y=p.y*1000.0;
    input.z=p.z*1000.0;
    //later quaternion

    if(finger_id==1){
        desired_centroid_idx_1=connect_centroid_to_contact(p, q, "desired centroid "+std::to_string(finger_id), true);
        desired_pose_1<<p.x, p.y, p.z, q.x, q.y, q.z, q.w;
    }
    else{
        desired_centroid_idx_2=connect_centroid_to_contact(p, q, "desired centroid "+std::to_string(finger_id), true);
        desired_pose_2<<p.x, p.y, p.z, q.x, q.y, q.z, q.w;
    }
    viewer->removeShape("desired contact "+std::to_string(finger_id));
    Eigen::Vector3f cube_pos(input.x, input.y, input.z);
    Eigen::Quaternionf cube_or(q.w, q.x, q.y, q.z);
    Eigen::Matrix3f cube_rot_matrix=cube_or.toRotationMatrix();
    cube_pos=cube_pos+cube_rot_matrix*Eigen::Vector3f(-l_finger/2+1.5, 0, 0);
    viewer->addCube(cube_pos, cube_or, l_finger, 3, 3, "desired contact "+std::to_string(finger_id));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "desired contact "+std::to_string(finger_id));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "desired contact "+std::to_string(finger_id));

}

std::stack<int> ShapeAnalyzer::get_path(std::multimap<uint32_t,uint32_t> graph, int init, int goal, Eigen::Vector3f grasp_line, int opposite_component){
    std::cout<<"computing path from: "<<init<<" to "<<goal<<std::endl;

    //check if a path exists. i.e. the elements are in the connencted component:
    if(nodes_to_connected_component.at(init)!=nodes_to_connected_component.at(goal)){
        std::cout<<"The nodes are not connected. Regrasping is needed."<<std::endl;
        std::stack<int> empty;
        return empty;
    }

    //check the normal to the opposite_component
    Eigen::Vector3f plane_normal=component_to_average_normal.at(nodes_to_connected_component.at(opposite_component));
    Eigen::Vector3f l0, p0;
    l0<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(init))).x, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(init))).y, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(init))).z;

    p0<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(opposite_component))).x, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(opposite_component))).y, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(opposite_component))).z;

    //insert the first distance
    double distance_between_contacts=(p0-l0).norm();
    node_to_slave_distance.insert(std::pair<uint32_t, double>(uint32_t(init), distance_between_contacts));    

    //do graph search here
    //create the set of keys (vertices)
    std::set<uint32_t> Q;
    std::vector<double> dist, dist_faked;
    std::vector<uint32_t> prev;
    for(uint32_t i=0; i<all_centroids_cloud->points.size(); i++){
        Q.insert(i);
        dist.push_back(DBL_MAX);
        dist_faked.push_back(DBL_MAX);
        prev.push_back(all_centroids_cloud->points.size()+1);
    }
    dist[init]=0;
    dist_faked[init]=0;

    //std::cout<<"data structures initialized for graph search."<<std::endl;

    int u;
    std::stack<int> S;
    //iterate over the vertices
    while(Q.size()>0){
        //get element with minimum distance
        std::vector<double>::iterator it=std::min_element(std::begin(dist_faked), std::end(dist_faked)); 
        u=std::distance(std::begin(dist_faked), it);
        //remove this element from Q
        Q.erase(u);
        dist_faked[u]=DBL_MAX; //this is to look for shortest distance of elements still in Q
        //std::cout<<"current vertex: "<<u<<std::endl;
        //check if the element found is the goal
        if(u!=goal){
            //loop over all the neighbours of u
            std::multimap<uint32_t,uint32_t>::iterator adjacent_itr=graph.equal_range(uint32_t(u)).first;
            for ( ; adjacent_itr!=graph.equal_range(uint32_t(u)).second; adjacent_itr++){
                //if this element is still in Q
                //std::cout<<"current adjacent element "<<adjacent_itr->second<<std::endl;
                if(Q.find(int(adjacent_itr->second))!=Q.end()){
                    //compute the intersection between the opposite plane and the grasp line(assumed constant!)
                    //l0 is the current considered point
                    l0<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->second)).x, 
                        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->second)).y, 
                        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->second)).z;
                    double alpha=-(l0-p0).dot(plane_normal)/(grasp_line.dot(plane_normal));
                    Eigen::Vector3f intersection_point=alpha*grasp_line+l0;
                    //now find the nearest centroid to this point, and check if it belongs to the same connected component(be sure to check that this node IS associaded to a component first)
                    int K = 3;//three nearest neighbours
                    std::vector<int> pointIdxNKNSearch(K);
                    std::vector<float> pointNKNSquaredDistance(K);
                    //improve this distance according to the change in normal (TO DO)
                    double slave_dist=100;
                    pcl::PointXYZRGBA input;
                    input.x=intersection_point(0);
                    input.y=intersection_point(1);
                    input.z=intersection_point(2);

                    //now add the distance between the current contact point and the obtained component to the map between distances and nodes (along the normal)
                    distance_between_contacts=(l0-intersection_point).norm();
                    node_to_slave_distance.insert(std::pair<uint32_t, double>(adjacent_itr->second, distance_between_contacts));

                    //std::cout<<"Checking the opposite component."<<std::endl;

                    if (centroids_kdtree.nearestKSearch(input, K, pointIdxNKNSearch, pointNKNSquaredDistance)> 0){
                        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                            //search for one component in the default connected component
                            if(nodes_to_connected_component.at(pc_to_supervoxel_idx.at(pointIdxNKNSearch[i]))==nodes_to_connected_component.at(opposite_component)){
                                slave_dist=0;
                                //std::cout<<"The slave contact is in the connected component."<<std::endl;
                                break;
                            }

                        }
                    }
                    else{
                        //there is something very wrong
                        std::cout<<"ERROR: cold not associate slave contact to centroid"<<std::endl;
                    }

                    //compute the real distance between the nodes now
                    Eigen::Vector3f source_point;
                    source_point<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->first)).x, 
                        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->first)).y, 
                        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->first)).z;

                    double node_dist=(l0-source_point).norm();

                    //the distance can be modified by the opposite finger if it is not in the same connected component
                    double alt=dist[u]+node_dist+slave_dist; 
                    if(alt<dist[int(adjacent_itr->second)]){
                        //a new shortest path has been found
                        dist[int(adjacent_itr->second)]=alt;
                        dist_faked[int(adjacent_itr->second)]=alt;
                        prev[int(adjacent_itr->second)]=u;
                    }
                }
            }

        }
        else{
            //std::cout<<"Found Path!"<<std::endl;
            //do something
            while(prev[u]!=all_centroids_cloud->points.size()+1){
                //std::cout<<"u: "<<u<<std::endl;
                S.push(u);
                u=prev[u];
            }
            S.push(u);
            break; 
        }
    }
    if(Q.size()==0){
        std::cout<<"No path has been found"<<std::endl;
        return S;
    }

    std::cout<<"Found path of length: "<<S.size()-1<<std::endl;
    return S;
}

void ShapeAnalyzer::compute_path(int finger_id){
    //first of all, clear the previously drawn path and delete the previous translation sequence
    int max_line_id=translation_sequence.size()-1;
    translation_sequence = std::vector<geometry_msgs::Point>();
    angle_sequence=std::vector<double>();
    distance_variations=std::vector<double>();
    for(int i=0; i<=max_line_id; i++){
        viewer->removeShape("line "+std::to_string(i));
    }

    //compute the line between the two contact points;
    Eigen::Vector3f grasp_point1, grasp_point2, grasp_line;
    grasp_point1<<initial_pose_1(0, 0), initial_pose_1(1, 0), initial_pose_1(2, 0);
    grasp_point2<<initial_pose_2(0, 0), initial_pose_2(1, 0), initial_pose_2(2, 0);
    std::cout<<"grasp points: "<<std::endl<<grasp_point1.transpose()<<std::endl<<grasp_point2.transpose()<<std::endl;
    grasp_line=grasp_point2-grasp_point1;
    grasp_line.normalize();
    std::cout<<"initial grasp line: "<<grasp_line.transpose()<<std::endl;
    //check if the desired contact is valid
    grasp_point1=Eigen::Vector3f(desired_pose_1(0, 0), desired_pose_1(1, 0), desired_pose_1(2, 0));
    grasp_point2=Eigen::Vector3f(desired_pose_2(0, 0), desired_pose_2(1, 0), desired_pose_2(2, 0));
    std::cout<<"desired points: "<<std::endl<<grasp_point1.transpose()<<std::endl<<grasp_point2.transpose()<<std::endl;
    Eigen::Vector3f desired_grasp_line=grasp_point2-grasp_point1;
    desired_grasp_line.normalize();
    std::cout<<"desired grasp line: "<<desired_grasp_line.transpose()<<std::endl;
    std::cout<<"diff vector: "<<(grasp_line-desired_grasp_line).transpose()<<std::endl;
    std::cout<<"diff norm: "<<(grasp_line-desired_grasp_line).norm()<<std::endl;
    if(fabs((grasp_line-desired_grasp_line).norm())>0.3){
        std::cout<<"Wrong final pose. Rotation around different axes could be required."<<std::endl;
        return;
    }

    std::stack<int> S;
    std::vector<int> path;
    if(finger_id==1){
        S=get_path(refined_adjacency, initial_centroid_idx_1, desired_centroid_idx_1, grasp_line, initial_centroid_idx_2);
    }
    else{
        S=get_path(refined_adjacency, initial_centroid_idx_2, desired_centroid_idx_2, -grasp_line, initial_centroid_idx_1);
    }

    if(S.size()<1){
        return;
    }

    //then draw the lines to highligt the path and store the points to keep track of the necessary translations
    //get the first point
    int idx;
    pcl::PointXYZRGBA point1;
    pcl::PointXYZRGBA point2;
    idx=S.top();
    S.pop();
    path.push_back(idx);
    point1=supervoxel_clusters.at(idx)->centroid_;

    //variables to store the translation sequence
    geometry_msgs::Point p1, p1Scaled;
    p1.x=point1.x;
    p1.y=point1.y;
    p1.z=point1.z;
    p1Scaled.x=point1.x/1000.0;
    p1Scaled.y=point1.y/1000.0;
    p1Scaled.z=point1.z/1000.0;
    translation_sequence.push_back(p1Scaled);

    //get the new elements and draw the line
    int count=0;
    while(S.size()>0){
        idx=S.top();
        S.pop();
        path.push_back(idx);
        point2=supervoxel_clusters.at(idx)->centroid_;
        //store this in the translation sequence
        geometry_msgs::Point p2, p2Scaled;
        p2.x=point2.x;
        p2.y=point2.y;
        p2.z=point2.z;
        p2Scaled.x=point2.x/1000.0;
        p2Scaled.y=point2.y/1000.0;
        p2Scaled.z=point2.z/1000.0;
        translation_sequence.push_back(p2Scaled);
        viewer->addLine(point1, point2, 0, 1, 0, "line "+std::to_string(count));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line "+std::to_string(count));
        count++;
        //std::cout<<"line drawn: "<<count<<std::endl;
        point1=point2;
    }
    //std::cout<<"TRANSLATION SEQUENCE::::::"<<translation_sequence.size()<<std::endl;  

    //compute angle sequence
    compute_angle_sequence(path, finger_id); 
    //compute distance sequence
    compute_contact_distances(path); 
}

void ShapeAnalyzer::set_finger_length(double l){
    l_finger=l*1000.0;//in mm
}

void ShapeAnalyzer::refine_adjacency(){
    //create another viewport
    int v(0);
    v2=v;
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0, 0, 0, v2);
    viewer->addText ("refined adjacency", 10, 10, "v2 text", v2);
    //scan the adjacency list to check for the normals
    std::multimap<uint32_t,uint32_t>::iterator label_itr=supervoxel_adjacency.begin();
    for ( ; label_itr!=supervoxel_adjacency.end();) {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //std::cout<<"before refining, current: "<<supervoxel_label<<std::endl;
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel=supervoxel_clusters.at (supervoxel_label);
        //now get the normal of this supervoxel
        pcl::Normal supervoxel_normal=supervoxel->normal_;
        //std::cout<<"current normal: "<<supervoxel_normal.normal_x<<" "<<supervoxel_normal.normal_y<<" "<<supervoxel_normal.normal_z<<std::endl<<std::endl;

        //Now we need to iterate through the adjacent supervoxels to check their normals
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers; //this will be used for debug
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr=supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range(supervoxel_label).second; adjacent_itr++){
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel=supervoxel_clusters.at(adjacent_itr->second);
            pcl::Normal adjacent_normal=neighbor_supervoxel->normal_;
            //std::cout<<"adjacent normal: "<<adjacent_normal.normal_x<<" "<<adjacent_normal.normal_y<<" "<<adjacent_normal.normal_z<<std::endl;
            //now check if these two normals have a similar direction
            if(fabs((supervoxel_normal.normal_x-adjacent_normal.normal_x)*(supervoxel_normal.normal_x-adjacent_normal.normal_x)+
                (supervoxel_normal.normal_y-adjacent_normal.normal_y)*(supervoxel_normal.normal_y-adjacent_normal.normal_y)+
                (supervoxel_normal.normal_z-adjacent_normal.normal_z)*(supervoxel_normal.normal_z-adjacent_normal.normal_z))<0.07){ //check what value to put there
                //std::cout<<"1 is here"<<std::endl;

                //add this to the visualization debug
                adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
                //add this to the refined adjacency
                refined_adjacency.insert(std::pair<uint32_t, uint32_t>(supervoxel_label, adjacent_itr->second));
            }
            //check if the normal is pointing in the opposite direction (inwards instead of outwards)
            else if(fabs((supervoxel_normal.normal_x+adjacent_normal.normal_x)*(supervoxel_normal.normal_x+adjacent_normal.normal_x)+
                (supervoxel_normal.normal_y+adjacent_normal.normal_y)*(supervoxel_normal.normal_y+adjacent_normal.normal_y)+
                (supervoxel_normal.normal_z+adjacent_normal.normal_z)*(supervoxel_normal.normal_z+adjacent_normal.normal_z))<0.07){
                //std::cout<<"1 is in there"<<std::endl;

                //add this to the visualization debug
                adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
                //add this to the refined adjacency
                refined_adjacency.insert(std::pair<uint32_t, uint32_t>(supervoxel_label, adjacent_itr->second));
            }
            else if(!(refined_adjacency.find(supervoxel_label)!=refined_adjacency.end())){
                //otherwise this node is just alone
                refined_adjacency.insert(std::pair<uint32_t, uint32_t>(supervoxel_label, supervoxel_label));
            }
        }
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "refined_supervoxel_" << supervoxel_label;
        //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
        addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str(), viewer, v2);
        //Move iterator forward to next label
        label_itr=supervoxel_adjacency.upper_bound(supervoxel_label);
    }

    //analyze the refined adjacency to generate a map of the connected components (BFS)
    std::set<uint32_t> examined_nodes;
    label_itr=refined_adjacency.begin();
    int component_id=-1;
    for ( ; label_itr!=refined_adjacency.end();) {
        //std::cout<<"++++++++++++++++++++++++++++++"<<std::endl;
        //get the supervoxel id
        uint32_t supervoxel_label = label_itr->first;
        //std::cout<<"Current ITERATOR: "<<supervoxel_label<<std::endl;
        if(!(examined_nodes.find(supervoxel_label)!=examined_nodes.end())){
            //this node had not been analyzed before! It means it is a new connected component
            component_id++;
            int num_components=0;
            Eigen::Vector3f sum_normal;
            sum_normal<<0.0, 0.0, 0.0;

            Eigen::Vector3f reference_normal;
            reference_normal<<supervoxel_clusters.at(supervoxel_label)->normal_.normal_x,
                        supervoxel_clusters.at(supervoxel_label)->normal_.normal_y,
                        supervoxel_clusters.at(supervoxel_label)->normal_.normal_z;
            //std::cout<<"reference normal: "<<reference_normal.transpose()<<std::endl;

            //now start a dfs visit of all the nodes that can be reached from here!
            std::set<uint32_t> discovered_nodes;
            std::stack<uint32_t> S;
            S.push(supervoxel_label);
            while (!S.empty()){
                uint32_t v=S.top();
                S.pop();
                if(discovered_nodes.find(v)==discovered_nodes.end()){
                    discovered_nodes.insert(v); //for the local bfs
                    examined_nodes.insert(v); //for the global analysis

                    //std::cout<<"inserting component of: "<<v<<std::endl;

                    nodes_to_connected_component.insert(std::pair<uint32_t, int>(v, component_id));
                    Eigen::Vector3f normal;
                    normal<<supervoxel_clusters.at(v)->normal_.normal_x,
                        supervoxel_clusters.at(v)->normal_.normal_y,
                        supervoxel_clusters.at(v)->normal_.normal_z;

                    //check if the normal has not a huge discordance:
                    //if((reference_normal-normal).squaredNorm()<0.07){
                    if((reference_normal-normal).squaredNorm()<=(reference_normal+normal).squaredNorm()){
                        //average the normal
                        sum_normal=sum_normal+normal;
                        num_components++;
                        //std::cout<<"normals+: "<<normal.transpose()<<std::endl;
                        //std::cout<<"    diff+: "<<(reference_normal-normal).squaredNorm()<<std::endl;
                        //std::cout<<"    sum: "<<sum_normal.transpose()<<std::endl;
                    }
                    else{
                        //average the normal but change sign
                        sum_normal=sum_normal-normal;
                        num_components++;
                        //std::cout<<"normals-: "<<normal.transpose()<<std::endl;
                        //std::cout<<"    diff+: "<<(reference_normal-normal).squaredNorm()<<std::endl;
                        //std::cout<<"    diff-: "<<(reference_normal+normal).squaredNorm()<<std::endl;
                        //std::cout<<"    sum: "<<sum_normal.transpose()<<std::endl;
                    }                    

                    //now get all the adjacent nodes
                    std::multimap<uint32_t,uint32_t>::iterator adjacent_itr=refined_adjacency.equal_range(v).first;
                    for ( ; adjacent_itr!=refined_adjacency.equal_range(v).second; adjacent_itr++){
                        S.push(adjacent_itr->second);
                    }
                }
            }

            //now get the average normal
            Eigen::Vector3f average_normal=sum_normal/num_components;
            average_normal.normalize();
            //std::cout<<"----average:  "<<average_normal.transpose()<<std::endl;
            component_to_average_normal.insert(std::pair<int, Eigen::Vector3f>(component_id, average_normal));
            connected_component_to_set_of_nodes.insert(std::pair<int, std::set<uint32_t>>(component_id, discovered_nodes));
        }

        //move the iterator to the next label
        label_itr=refined_adjacency.upper_bound(supervoxel_label);
    }

    std::cout<<"Found connected components: "<<component_id+1<<std::endl;


    //now that the adjacency has been already reduced a lot, check for the possible orientations of the gripper
    //for each connected component, whose normals are similar, define an absolute orientation which is the 0 deg orientation
    //remember that some normals have opposite sign!

    //discretization step of 5 degrees:
    std::set<int> all_angles;
    for(int i=0; i<360; i+=angle_jump){
        all_angles.insert(i);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Vector3f x, y, z;
    x<<1, 0, 0;
    y<<0, 1, 0;
    z<<0, 0, 1;

    pcl::PassThrough<pcl::PointXYZ> pass;

    bool done=false;

    for(int component=0; component<=component_id; component++){
        //std::cout<<"component: "<<component<<std::endl;
        
        //get the average normal of the component:
        Eigen::Vector3f nx, ny, nz;
        nx=component_to_average_normal.at(component);
        //std::cout<<"========AVERAGE NORMAL: "<<nx.transpose()<<std::endl;
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
        nz=nx.cross(ny);

        //ny is the axis at which the angle is 0, for all the nodes

        //great. Now we can create a matrix to transform the pointcloud with this new reference frame
        //Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f R, R_transpose;
        R(0,0)=nx.dot(x);
        R(0,1)=nx.dot(y);
        R(0,2)=nx.dot(z);

        R(1,0)=ny.dot(x);
        R(1,1)=ny.dot(y);
        R(1,2)=ny.dot(z);

        R(2,0)=nz.dot(x);
        R(2,1)=nz.dot(y);
        R(2,2)=nz.dot(z);

        R_transpose=R.transpose();

        //check if nx is pointing inwards or outwards with respect to the object's center:

        /*transform.block<3,3>(0,0)=R;*/
        //for the translation, cicle through all the nodes in the component
        std::vector<uint32_t> nodes(connected_component_to_set_of_nodes.at(component).begin(), connected_component_to_set_of_nodes.at(component).end());
        for(int idx=0; idx<nodes.size(); idx++){
            //std::cout<<"idx: "<<idx<<std::endl;
            //std::cout<<"    node: "<<nodes[idx]<<std::endl;
            
            int pc_idx=supervoxel_to_pc_idx.at(nodes[idx]);
            pcl::PointXYZRGBA c=all_centroids_cloud->at(pc_idx);
            //std::cout<<"===point normal: "<<supervoxel_clusters.at(nodes[idx])->normal_.normal_x<<" "<<supervoxel_clusters.at(nodes[idx])->normal_.normal_y<< " "<<supervoxel_clusters.at(nodes[idx])->normal_.normal_z<<std::endl;
            //transform.block<3,1>(0,3)=-R*Eigen::Vector3f(c.x, c.y, c.z);
            //now we can transform the pointcloud and filter it

            //pcl::transformPointCloud (*object_shape, *transformed_cloud, transform);

            //now cut the pointcloud
            /*pass.setInputCloud (transformed_cloud);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (-1, 1); //2 mm total 
            pass.filter(*cloud_filtered);

            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("y");
            pass.setFilterLimits (-l_finger-0.5, l_finger+0.5);
            pass.filter(*transformed_cloud);

            pass.setInputCloud(transformed_cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits (-l_finger-0.5, l_finger+0.5); 
            pass.filter(*cloud_filtered);*/

            //use kdltree to create a sphere with all the neighbours and their respective distances
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud (object_shape);
            pcl::PointXYZ searchPoint;
            //searchPoint.x=searchPoint.y=searchPoint.z=0.0;
            searchPoint.x=c.x;
            searchPoint.y=c.y;
            searchPoint.z=c.z;
            //vector for the transform
            Eigen::Vector3f translation=-R*Eigen::Vector3f(c.x,c.y,c.z);
            std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
            std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

            std::set<int> angles_per_node(all_angles);

            //variables used to check the direction of the normal:
            Eigen::Vector3f t_vector;
            t_vector<<c.x, c.y, c.z;
            double squared_sphere_radius=(-object_center + t_vector).squaredNorm();
            //check the direction of the normal nx
            Eigen::Vector3f control_vector=t_vector+0.5*nx;
            bool inwards_normal=true;
            if(false){
            //if(nodes[idx] == 77){
                std::cout<<"sphere radius: "<<squared_sphere_radius<<std::endl;
                std::cout<<"t_vector: "<<t_vector.transpose()<<std::endl;
                std::cout<<"obj center: "<<object_center.transpose()<<std::endl;
                std::cout<<"nx: "<<nx.transpose()<<std::endl;
                std::cout<<"control_vector: "<<control_vector.transpose()<<std::endl;
                std::cout<<"diff_vector squared norm: "<<(control_vector-object_center).squaredNorm()<<std::endl;
            }
            if((control_vector(0)-object_center(0))*(control_vector(0)-object_center(0))+(control_vector(1)-object_center(1))*(control_vector(1)-object_center(1))+
                (control_vector(2)-object_center(2))*(control_vector(2)-object_center(2)) > squared_sphere_radius){
                //if the control vector has one component too close to the object center, do additional checks.
                inwards_normal=false;
            }

            //std::cout<<"        impossible angles: ";

            if(kdtree.radiusSearch(searchPoint, l_finger+3.0, pointIdxRadiusSearch, pointRadiusSquaredDistance)>0){
                pcl::PointCloud<pcl::PointXYZ>::Ptr neighbour_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                //waste time in additional checks (do this only if the node is somehow not in the bounding box:
                int inwards_count=0;
                int outwards_count=0;
                bool found_y_neg_i=false;
                bool found_z_neg_i=false;
                bool found_y_pos_i=false;
                bool found_z_pos_i=false;
                bool found_y_neg_o=false;
                bool found_z_neg_o=false;
                bool found_y_pos_o=false;
                bool found_z_pos_o=false;

                for(int i=0; i<pointIdxRadiusSearch.size(); i++){
                    Eigen::Vector3f point;
                    point<<object_shape->at(pointIdxRadiusSearch[i]).x, object_shape->at(pointIdxRadiusSearch[i]).y, object_shape->at(pointIdxRadiusSearch[i]).z;
                    Eigen::Vector3f transformed_point=R*point+translation; 

                    //the x axis corresponds to the normal. the ny axis corresponds to the 0 angle
                    //check the x component: (for now assume nx is always pointing inwards from the object's center)
                    if(transformed_point(0)<-3.0 && transformed_point(0)>-5.0){
                        outwards_count++;
                        if (transformed_point(1)<0){
                            found_y_neg_o=true;
                        }
                        else if(transformed_point(1)>0){
                            found_y_pos_o=true;
                        }
                        if(transformed_point(2)<0){
                            found_z_neg_o=true;
                        }
                        else if(transformed_point(2)>0){
                            found_z_pos_o=true;
                        }
                    }
                    else if(transformed_point(0)>3.0 && transformed_point(0)<5.0){
                        inwards_count++;
                        if (transformed_point(1)<0){
                            found_y_neg_i=true;
                        }
                        else if(transformed_point(1)>0){
                            found_y_pos_i=true;
                        }
                        if(transformed_point(2)<0){
                            found_z_neg_i=true;
                        }
                        else if(transformed_point(2)>0){
                            found_z_pos_i=true;
                        }
                    }
                }
                //inwards_normal=(inwards_count>outwards_count);
                //if(nodes[idx] == 77){
                if(false){
                //if(component == 18){
                    std::cout<<"found inwards pos, neg: "<<found_y_neg_i<<" "<<found_y_pos_i<<" "<<found_z_neg_i<<" "<<found_z_pos_i<<std::endl;
                    std::cout<<"INWARDS COUNT: "<<inwards_count<<std::endl;
                    std::cout<<"found outwards pos, neg: "<<found_y_neg_o<<" "<<found_y_pos_o<<" "<<found_z_neg_o<<" "<<found_z_pos_o<<std::endl;
                    std::cout<<"OUTWARDS COUNT: "<<outwards_count<<std::endl;
                }
                if (!(found_y_neg_o && found_y_pos_o && found_z_neg_o && found_z_pos_o && found_y_neg_i && found_y_pos_i && found_z_neg_i && found_z_pos_i)){
                    if(found_y_neg_o && found_y_pos_o && found_z_neg_o && found_z_pos_o){
                        inwards_normal=false;
                    } 
                    else if(found_y_neg_i && found_y_pos_i && found_z_neg_i && found_z_pos_i){
                        inwards_normal=true;
                    }
                }
                else if(inwards_count > outwards_count){
                    inwards_normal=true;
                }
                
                for(int i=0; i<pointIdxRadiusSearch.size(); i++){
                    //check if the distance is larger
                    if(pointRadiusSquaredDistance[i]>=(l_finger-2.0)*(l_finger-2.0)){
                        //create a 3D vector:
                        Eigen::Vector3f point;
                        point<<object_shape->at(pointIdxRadiusSearch[i]).x, object_shape->at(pointIdxRadiusSearch[i]).y, object_shape->at(pointIdxRadiusSearch[i]).z;
                        Eigen::Vector3f transformed_point=R*point+translation;
                        //use the coordinates of the point in the normal reference frame to compute the elevation
                        //double elevation=asin(transformed_point(0)/sqrt(pointRadiusSquaredDistance[i]));
                        //now check if the elevation is in the defined interval:
                        //if(fabs(elevation)<0.1){ //10 degrees total
                            //std::cout<<"point: "<<transformed_point.transpose()<<std::endl;

                        //check the x coordinate to filter out some of these
                        if(fabs(transformed_point(0))<l_finger/4.0){
                            //add the point to the poincloud for debug

    
                            //neighbour_cloud->push_back(object_shape->at(pointIdxRadiusSearch[i]));

                            //check the point and associate it with the corresponding angle
                            //the x axis corresponds to the normal. the ny axis corresponds to the 0 angle
                            double angle=atan2(transformed_point(2), transformed_point(1));
                            //move it between 0 and 2 pi
                            if(angle<0){
                                angle=angle+2*M_PI;
                            }
                            //convert the angle into degrees:
                            angle=angle*180.0/M_PI;
                            //std::cout<<"    angle: "<<angle<<std::endl;
                            //approximate the angle in multiples of 5
                            double round_angle=angle+angle_jump/2;
                            //round_angle-=floor(round_angle)%5;
                            //convert in int
                            int int_round_angle=floor(round_angle);
                            int int_angle=int_round_angle-(int_round_angle%angle_jump);
                            if(int_angle%angle_jump!=0){
                                std::cout<<"ERROR IN THE ROUNDING TO "<<angle_jump<<std::endl;
                            }
                            //convert from 360 to 0:
                            if(int_round_angle==360){
                                int_round_angle=0;
                            }
                            if (angles_per_node.find(int_angle)!=angles_per_node.end()){
                                if(false){
                                    viewer->addSphere(object_shape->at(pointIdxRadiusSearch[i]), 1, 0.0, 0.0, 0.6, "anglesphere"+std::to_string(int_angle)+"_"+std::to_string(nodes[idx]), v2);
                                }
                                angles_per_node.erase((int_angle)); 
                                //std::cout<<int_angle<<", ";
                            }
                        }
                    }
                    else {//if (pointRadiusSquaredDistance[i]>=100.0){
                        //these are probable collisions not due to the length of the finger, but because of the object's shape
                        //check the elevation of the transformed point. If it is too big (tolerance of 0.2 radians to remove those on the surface?)
                        //create a 3D vector:
                        Eigen::Vector3f point;
                        point<<object_shape->at(pointIdxRadiusSearch[i]).x, object_shape->at(pointIdxRadiusSearch[i]).y, object_shape->at(pointIdxRadiusSearch[i]).z;
                        Eigen::Vector3f transformed_point=R*point+translation; 

                        //the x axis corresponds to the normal. the ny axis corresponds to the 0 angle
                        //check the x component: (for now assume nx is always pointing inwards from the object's center)
                        if( (transformed_point(0)<-3.0 && transformed_point(0)>-5.0 && inwards_normal) ||
                            (transformed_point(0)>3.0 && transformed_point(0)<5.0 && !inwards_normal) )
                            {
                            neighbour_cloud->push_back(object_shape->at(pointIdxRadiusSearch[i]));
                            double angle=atan2(transformed_point(2), transformed_point(1));
                            //move it between 0 and 2 pi
                            if(angle<0){
                                angle=angle+2*M_PI;
                            }
                            //convert the angle into degrees:
                            angle=angle*180.0/M_PI;
                            //std::cout<<"    angle: "<<angle<<std::endl;
                            //approximate the angle in multiples of 5
                            double round_angle=angle+angle_jump/2;
                            //round_angle-=floor(round_angle)%5;
                            //convert in int
                            int int_round_angle=floor(round_angle);
                            int int_angle=int_round_angle-(int_round_angle%angle_jump);
                            if(int_angle%5!=0){
                                std::cout<<"ERROR IN THE ROUNDING TO "<<angle_jump<<std::endl;
                            }
                            //convert from 360 to 0:
                            if(int_round_angle==360){
                                int_round_angle=0;
                            }
                            if (angles_per_node.find(int_angle)!=angles_per_node.end()){
                                if(false){
                                    viewer->addSphere(object_shape->at(pointIdxRadiusSearch[i]), 1, 0.0, 0.0, 0.6, "anglesphere"+std::to_string(int_angle)+"_"+std::to_string(nodes[idx]), v2);
                                }
                                angles_per_node.erase((int_angle)); 
                                //std::cout<<int_angle<<", ";
                            }
                        }
                    }
                }
                //if(idx == 1 && component== 0){
                //if(false){
                if(nodes[idx] == 53 || nodes[idx] == 58 || nodes[idx] == 55 || nodes[idx] ==114){
                //if(component == 18){
                    std::cout<<"==========idx: "<<idx<<std::endl;
                    std::cout<<"=================== node: "<<nodes[idx]<<std::endl;
                    std::cout<<"inwards normal: "<<inwards_normal<<std::endl;
                    //add the neighbor pointcloud to the viewer
                    std::stringstream ss;
                    ss<<"neighbor_" <<nodes[idx];
                    //random color
                    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> hc(neighbour_cloud);
                    viewer->addPointCloud (neighbour_cloud, hc, ss.str(), v2);
                    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, ss.str());
                    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, ss.str());
                    viewer->addSphere(supervoxel_clusters.at(nodes[idx])->centroid_, 1, 0.0, 0.0, 1.0, "sphere"+std::to_string(nodes[idx]), v2);
                    Eigen::Matrix4f transf=Eigen::Matrix4f::Identity();
                    transf.block<3, 3>(0, 0)=R_transpose;
                    transf.block<3,1>(0, 3)=Eigen::Vector3f(c.x,c.y,c.z);
                    Eigen::Affine3f aff;
                    aff.matrix()=transf;
                    viewer->addCoordinateSystem(3.0, aff, "normals ");
                    //done=false;
                }
            }
            //std::cout<<std::endl;

            //now add this mapping between the supervoxel and the possible angles
            possible_angles.insert(std::pair<uint32_t, std::set<int>>(nodes[idx], angles_per_node));
            //generate the angle components data structure that will be used in Dijikstra
            generate_angles_components_structures(int(nodes[idx]));
        }
        //std::cout<<std::endl;

    }
    //now further refine the adjacency by removing all the impossible connections and impossible vertices
    std::cout<<"Refining the adjacency list according to the possible angles"<<std::endl;
    //at the same time, generate the map from node to angle to component and the map from node to angle to subset of angles in the same component
    //loop on all the connected components
    std::multimap<uint32_t, uint32_t> angles_refined_adjacency;
    std::multimap<std::pair<uint32_t, int>, std::pair<uint32_t, int>> angles_extended_adjacency;
    for(int component=0; component<=component_id; component++){
        //get all the nodes in the component:
        std::set<uint32_t> nodes_set=connected_component_to_set_of_nodes.at(component);
        //now loop on all the nodes and check if their connections (or the node itself) have to be removed from the connected component
        for(std::set<uint32_t>::iterator it=nodes_set.begin(); it!=nodes_set.end(); it++){
            std::set<int> node_angles=possible_angles.at(*it);
            //if this node has no possible angles, it will be removed from the connected component and not added to the new adjacency map
            uint32_t node=*it;
            //debug prints:
            if(node==53||node==55||node==58||node==114)
                std::cout<<"node: "<<node<<"   set size: "<<node_angles.size()<<std::endl;
            if(node_angles.size()<1){
                connected_component_to_set_of_nodes.at(component).erase(node);
                nodes_to_connected_component.erase(node);
            }
            else{
                //generate_angles_components_structrures(*it);
                pcl::PointXYZRGBA node_center=supervoxel_clusters.at(node)->centroid_;
                pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
                //the list of all angle components
                //std::cout<<"node 1 to component: "<<node<<std::endl;
                std::vector<int> current_node_angle_components=node_to_angle_components.at(node);
                //std::cout<<"OK"<<std::endl;
                //now loop over all the components
                std::multimap<uint32_t,uint32_t>::iterator adjacent_itr=refined_adjacency.equal_range(node).first;
                for ( ; adjacent_itr!=refined_adjacency.equal_range(node).second; adjacent_itr++){
                    if(node==53||node==55||node==58||node==114)
                        std::cout<<"   adjacent node: "<<adjacent_itr->second<<" set size: "<<possible_angles.at(adjacent_itr->second).size()<<std::endl;
                    //get the intersection between the two 
                    std::set<int> angles_intersection;
                    std::set_intersection(node_angles.begin(), node_angles.end(), possible_angles.at(adjacent_itr->second).begin(), possible_angles.at(adjacent_itr->second).end(), std::inserter(angles_intersection, angles_intersection.begin()));
                    if(node==53||node==55||node==58||node==114)
                        std::cout<<"   intersection size: "<<angles_intersection.size()<<std::endl;
                    if(angles_intersection.size()>0){
                        angles_refined_adjacency.insert(std::pair<uint32_t, uint32_t>(node, adjacent_itr->second));
                        adjacent_supervoxel_centers.push_back(supervoxel_clusters.at(adjacent_itr->second)->centroid_);
                    
                        //do the same for the extended adjacency: loop over all the angles component of the current node and all the ones of the adjacent nodes
                        //remember to always check if a key exists before looping through this map
                        //the list of the components of angles of the adjacent node
                        //std::cout<<"node 2 to component: "<<adjacent_itr->second<<std::endl;
                        std::vector<int> adjacent_node_angle_components=node_to_angle_components.at(adjacent_itr->second);
                        //std::cout<<"OK"<<std::endl;
                        for(int ac_idx=0; ac_idx<current_node_angle_components.size(); ac_idx++){
                            std::set<int> node_component_set=node_component_to_angles_subset.at(std::pair<int, int>(node, ac_idx));
                            if(node==53||node==55||node==58||node==114)
                                std::cout<<"        pair 1: "<<node<<" "<<ac_idx<<"   size: "<<node_component_set.size()<<std::endl;
                            //std::cout<<"OK"<<std::endl;
                            for(int ac_jdx=0; ac_jdx<adjacent_node_angle_components.size(); ac_jdx++){
                                std::set<int> adjacent_component_set=node_component_to_angles_subset.at(std::pair<int, int>(adjacent_itr->second, ac_jdx));
                                if(node==53||node==55||node==58||node==114)
                                    std::cout<<"          pair 2: "<<adjacent_itr->second<<" "<<ac_jdx<<"   "<<adjacent_component_set.size()<<std::endl;
                                //std::cout<<"OK"<<std::endl;
                                std::set<int> component_angles_intersection;
                                std::set_intersection(node_component_set.begin(), node_component_set.end(), adjacent_component_set.begin(), adjacent_component_set.end(), std::inserter(component_angles_intersection, component_angles_intersection.begin()));
                                //if the components have an intersection, add this connection to the graph!
                                if(node==53||node==55||node==58||node==114)
                                    std::cout<<"               intersection size: "<<component_angles_intersection.size()<<std::endl;
                                if(component_angles_intersection.size()>0){
                                    extended_refined_adjacency.insert(std::pair<std::pair<uint32_t, int>, std::pair<uint32_t, int>>(std::pair<uint32_t, int>(node, ac_idx), std::pair<uint32_t, int>(adjacent_itr->second, ac_jdx)));
                                }
                            }

                        }

                    }

                }

                //now draw this poligon(different color to compare)
                std::stringstream ss;
                ss << "angles_refined_supervoxel_" << node;
                //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
                addSupervoxelConnectionsToViewer(node_center, adjacent_supervoxel_centers, ss.str(), viewer, v2);
                //now change the color of the new shape
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, ss.str(), v2);
            }
        }
    }
    //assign the new adjacency
    refined_adjacency=angles_refined_adjacency;

    //now visualize the new connections with the angles

    //generate the connected components of the further refinef adjacency.
    std::cout<<"Extended refinement..."<<std::endl;
    generate_connected_components_list_of_extended_refined_adjacency();    
}

void ShapeAnalyzer::compute_angle_sequence(std::vector<int> path, int finger_id){
    //get the desired angle from the desired pose (angle between the axiz z ("projected" on ny and nz) around nx, with ny being the 0 angle)
    //obtain the rotation matrix to transform the pose into the nx ny nz reference frame:
    Eigen::Matrix3f desired_gripper_to_base, initial_gripper_to_base, desired_component_to_base, initial_component_to_base;
    Eigen::Matrix3f slave_desired_gripper_to_base, slave_initial_gripper_to_base, slave_desired_component_to_base, slave_initial_component_to_base;
    Eigen::Quaternion<float> q1, q2, slave_q1, slave_q2;

    //1) matrix from base frame to nx ny nz
    //get the component of the initial contact
    uint32_t initial_contact_index, slave_initial_contact_index;
    if(finger_id==1){
        initial_contact_index=uint32_t(initial_centroid_idx_1);
        slave_initial_contact_index=uint32_t(initial_centroid_idx_2);
    }
    else{
        initial_contact_index=uint32_t(initial_centroid_idx_2);
        slave_initial_contact_index=uint32_t(initial_centroid_idx_1);
    }

    int initial_contact_connected_component=nodes_to_connected_component.at(initial_contact_index);
    int slave_initial_contact_connected_component=nodes_to_connected_component.at(slave_initial_contact_index);
    Eigen::Vector3f component_normal=component_to_average_normal.at(initial_contact_connected_component);
    Eigen::Vector3f slave_component_normal=component_to_average_normal.at(slave_initial_contact_connected_component);
    Eigen::Vector3f nx, ny, nz;
    Eigen::Vector3f slave_nx, slave_ny, slave_nz;
    nx=component_normal;
    slave_nx=slave_component_normal;
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
    nz=nx.cross(ny);

    if(fabs(slave_nx(0))>0.00000001){
        slave_ny(1)=0;
        slave_ny(2)=sqrt(slave_nx(0)*slave_nx(0)/(slave_nx(0)*slave_nx(0)+slave_nx(2)*slave_nx(2)));
        slave_ny(0)=-slave_nx(2)*slave_ny(2)/slave_nx(0);
    }
    else if(fabs(nx(1))>0.00000001){
        slave_ny(2)=0;
        slave_ny(0)=sqrt(slave_nx(1)*slave_nx(1)/(slave_nx(1)*slave_nx(1)+slave_nx(0)*slave_nx(0)));
        slave_ny(1)=-slave_ny(0)*slave_nx(0)/slave_nx(1);
    }
    else{
        slave_ny(0)=0;
        slave_ny(1)=sqrt(slave_nx(2)*slave_nx(2)/(slave_nx(1)*slave_nx(1)+slave_nx(2)*slave_nx(2)));
        slave_ny(2)=-slave_nx(1)*slave_ny(1)/slave_nx(2);
    }
    slave_nz=slave_nx.cross(slave_ny);

    Eigen::Matrix3f component_to_base, slave_component_to_base;
    component_to_base(0,0)=nx(0);
    component_to_base(1,0)=nx(1);
    component_to_base(2,0)=nx(2);

    component_to_base(0,1)=ny(0);
    component_to_base(1,1)=ny(1);
    component_to_base(2,1)=ny(2);

    component_to_base(0,2)=nz(0);
    component_to_base(1,2)=nz(1);
    component_to_base(2,2)=nz(2);

    slave_component_to_base(0,0)=slave_nx(0);
    slave_component_to_base(1,0)=slave_nx(1);
    slave_component_to_base(2,0)=slave_nx(2);

    slave_component_to_base(0,1)=slave_ny(0);
    slave_component_to_base(1,1)=slave_ny(1);
    slave_component_to_base(2,1)=slave_ny(2);

    slave_component_to_base(0,2)=slave_nz(0);
    slave_component_to_base(1,2)=slave_nz(1);
    slave_component_to_base(2,2)=slave_nz(2);


    //2) matrix from base to gripper pose
    uint32_t desired_contact_index, slave_desired_contact_index;
    if(finger_id==1){
        desired_contact_index=uint32_t(desired_centroid_idx_1);
        slave_desired_contact_index=uint32_t(desired_centroid_idx_2);
        q1=Eigen::Quaternion<float>(initial_pose_1(6), initial_pose_1(3), initial_pose_1(4), initial_pose_1(5)); //Eigen has the w before in the quaternion
        q2=Eigen::Quaternion<float>(desired_pose_1(6), desired_pose_1(3), desired_pose_1(4), desired_pose_1(5));
        slave_q1=Eigen::Quaternion<float>(initial_pose_2(6), initial_pose_2(3), initial_pose_2(4), initial_pose_2(5)); //Eigen has the w before in the quaternion
        slave_q2=Eigen::Quaternion<float>(desired_pose_2(6), desired_pose_2(3), desired_pose_2(4), desired_pose_2(5));
    }
    else{
        desired_contact_index=uint32_t(desired_centroid_idx_2);
        slave_desired_contact_index=uint32_t(desired_centroid_idx_1);
        q1=Eigen::Quaternion<float>(initial_pose_2(6), initial_pose_2(3), initial_pose_2(4), initial_pose_2(5));
        q2=Eigen::Quaternion<float>(desired_pose_2(6), desired_pose_2(3), desired_pose_2(4), desired_pose_2(5));
        slave_q1=Eigen::Quaternion<float>(initial_pose_1(6), initial_pose_1(3), initial_pose_1(4), initial_pose_1(5)); //Eigen has the w before in the quaternion
        slave_q2=Eigen::Quaternion<float>(desired_pose_1(6), desired_pose_1(3), desired_pose_1(4), desired_pose_1(5));
    }


    initial_gripper_to_base=q1.toRotationMatrix();
    desired_gripper_to_base=q2.toRotationMatrix();

    slave_initial_gripper_to_base=slave_q1.toRotationMatrix();
    slave_desired_gripper_to_base=slave_q2.toRotationMatrix();

    std::cout<<"slave initial gripper to base: "<<std::endl<<slave_initial_gripper_to_base<<std::endl<<std::endl;
    std::cout<<"slave desired gripper to base: "<<std::endl<<slave_desired_gripper_to_base<<std::endl<<std::endl;
    std::cout<<"slave component to base: "<<std::endl<<slave_component_to_base<<std::endl<<std::endl;
    std::cout<<"slave component to base transpose: "<<std::endl<<slave_component_to_base.transpose()<<std::endl<<std::endl;

    //now get the vector Z' of the gripper expressed in the component's reference frame
    Eigen::Matrix3f initial_gripper_to_component=component_to_base.transpose()*initial_gripper_to_base;
    Eigen::Matrix3f desired_gripper_to_component=component_to_base.transpose()*desired_gripper_to_base;

    Eigen::Matrix3f slave_initial_gripper_to_component=slave_component_to_base.transpose()*slave_initial_gripper_to_base;
    Eigen::Matrix3f slave_desired_gripper_to_component=slave_component_to_base.transpose()*slave_desired_gripper_to_base;

    std::cout<<"initial gripper to component: "<<std::endl<<initial_gripper_to_component<<std::endl<<std::endl;
    std::cout<<"desired gripper to component: "<<std::endl<<desired_gripper_to_component<<std::endl<<std::endl;

    //the first column of the matrix is the x vector of the gripper. this one has to be inverted and then the angle w.r.t. the y axis of the gripper can be found
    Eigen::Vector3f x_prime=initial_gripper_to_component.block<3, 1>(0, 0);
    //std::cout<<"x_prime init: "<<x_prime.transpose()<<std::endl;
    double initial_angle=atan2(-x_prime(2), -x_prime(1));
    if (initial_angle<0){
        initial_angle+=2*M_PI;
    }
    x_prime=desired_gripper_to_component.block<3, 1>(0, 0);
    //std::cout<<"x_prime des: "<<x_prime.transpose()<<std::endl;
    double desired_angle=atan2(-x_prime(2), -x_prime(1));
    if (desired_angle<0){
        desired_angle+=2*M_PI;
    }

    //convert this angle into degrees and with intervals of 5 (angle_jump variable)
    initial_angle=initial_angle*180.0/M_PI;
    desired_angle=desired_angle*180.0/M_PI;
    double round_angle=initial_angle+angle_jump/2;
    //convert in int
    int int_initial_angle=floor(round_angle);
    //convert from 360 to 0:
    int_initial_angle=int_initial_angle-(int_initial_angle%angle_jump);
    if(int_initial_angle==360){
        int_initial_angle=0;
    }
    //same for desired angle
    round_angle=desired_angle+angle_jump/2;
    int int_desired_angle=floor(round_angle);
    int_desired_angle=int_desired_angle-(int_desired_angle%angle_jump);
    if(int_desired_angle==360){
        int_desired_angle=0;
    }

    std::cout<<"Initial angle: "<<int_initial_angle<<std::endl;
    std::cout<<"Desired angle: "<<int_desired_angle<<std::endl;

    //do the same thing with the slave contact:
    Eigen::Vector3f slave_x_prime=slave_initial_gripper_to_component.block<3, 1>(0, 0);
    //std::cout<<"x_prime init: "<<x_prime.transpose()<<std::endl;
    double slave_initial_angle=atan2(-slave_x_prime(2), -slave_x_prime(1));
    if (slave_initial_angle<0){
        slave_initial_angle+=2*M_PI;
    }
    slave_x_prime=slave_desired_gripper_to_component.block<3, 1>(0, 0);
    //std::cout<<"x_prime des: "<<x_prime.transpose()<<std::endl;
    double slave_desired_angle=atan2(-slave_x_prime(2), -slave_x_prime(1));
    if (slave_desired_angle<0){
        slave_desired_angle+=2*M_PI;
    }

    //convert this angle into degrees and with intervals of 5
    slave_initial_angle=slave_initial_angle*180.0/M_PI;
    slave_desired_angle=slave_desired_angle*180.0/M_PI;
    double slave_round_angle=slave_initial_angle+angle_jump/2;
    //convert in int
    int slave_int_initial_angle=floor(slave_round_angle);
    //convert from 360 to 0:
    slave_int_initial_angle=slave_int_initial_angle-(slave_int_initial_angle%angle_jump);
    if(slave_int_initial_angle==360){
        slave_int_initial_angle=0;
    }
    //same for desired angle
    slave_round_angle=slave_desired_angle+angle_jump/2;
    int slave_int_desired_angle=floor(slave_round_angle);
    slave_int_desired_angle=slave_int_desired_angle-(slave_int_desired_angle%angle_jump);
    if(slave_int_desired_angle==360){
        slave_int_desired_angle=0;
    }

    std::cout<<"Slave Initial angle: "<<slave_int_initial_angle<<std::endl;
    std::cout<<std::endl;

    int index=path.size()-1;
    //std::cout<<"Index: "<<index<<std::endl;
    //now cicle backwards the nodes to compute the sequence of angles
    std::set<int> current_node_angles=possible_angles.at(desired_contact_index);
    std::set<int> next_node_angles;
    int current_angle=int_desired_angle;
    std::cout<<"master desired angle: "<<current_angle<<std::endl;
    std::cout<<"master desired contact idx: "<<desired_contact_index<<std::endl;
    //first of all check if it is possible. If it is not, well... the list of angles will be empty and the task is impossible to solve
    if(current_node_angles.find(current_angle)==current_node_angles.end()){
        std::cout<<"The desired angle cannot be achieved."<<std::endl;
        return;
    }
    //check also for the slave finger final pose:
    std::set<int> slave_current_node_angles=possible_angles.at(slave_desired_contact_index);
    int slave_current_angle=slave_int_desired_angle;
    std::cout<<"slave desired angle: "<<slave_current_angle<<std::endl;
    std::cout<<"slave desired contact idx: "<<slave_desired_contact_index<<std::endl;
    if(slave_current_node_angles.find(slave_current_angle)==slave_current_node_angles.end()){
        std::cout<<"The desired angle cannot be achieved."<<std::endl;
        return;
    }

    //convert the angle back to double
    angle_sequence.resize(path.size());
    angle_sequence[index]=double(desired_angle)*M_PI/180.0;
    index=index-1;
    //std::cout<<"Index: "<<index<<std::endl;
    while(index>=0){
        next_node_angles=current_node_angles;
        current_node_angles=possible_angles.at(uint32_t(path[index]));
        std::set<int> intersection;
        std::set_intersection(current_node_angles.begin(), current_node_angles.end(), next_node_angles.begin(), next_node_angles.end(), std::inserter(intersection, intersection.begin()));
        //check if the current angle is in the intersection (i.e. the translation can be with the gripper at this angle) 
        if(intersection.find(current_angle)!=intersection.end()){
            angle_sequence[index]=double(current_angle)*M_PI/180.0;
        }
        else{
            if(intersection.size()==0){
                std::cout<<"error"<<std::endl;
                std::cout<<"The intersection is empty! The adjacency is wrong."<<std::endl;
            }
            //in this case the element is random. Choose one that is closer to the one we already have
            current_angle=*intersection.begin();
            angle_sequence[index]=double(current_angle)*M_PI/180.0;
        }
        index=index-1;
        //std::cout<<"Index: "<<index<<std::endl;
    }

    //now change the list so that it becomes a list of deltas
    double old_angle=initial_angle*M_PI/180; //this is the initial angle
    for(int i=0; i<angle_sequence.size(); i++){
        angle_sequence[i]=angle_sequence[i]-old_angle;
        old_angle=old_angle+angle_sequence[i];//uptade to the next angle
    }


}

void ShapeAnalyzer::compute_contact_distances(std::vector<int> path){
    //delete the old distance if any
    distance_variations.clear();
    //std::cout<<"node: "<<path[0]<<std::endl;
    double previous_distance=node_to_slave_distance.at(uint32_t(path[0]))/1000.0; //convert into meters from mm
    //std::cout<<"OK"<<std::endl;
    double current_distance;
    //loop through the path and create the sequence of distance variations
    for(int i=1; i<path.size(); i++){
        //std::cout<<"node: "<<path[i]<<std::endl;
        current_distance=node_to_slave_distance.at(uint32_t(path[i]))/1000.0;
        //std::cout<<"OK"<<std::endl;
        distance_variations.push_back(current_distance - previous_distance);
        previous_distance=current_distance;
    }

}

void ShapeAnalyzer::set_supervoxel_parameters(float voxel_res, float seed_res, float color_imp, float spatial_imp, float normal_imp, bool disable_transf, int refinement_it){
    voxel_resolution=voxel_res;
    seed_resolution=seed_res;
    color_importance=color_imp;
    spatial_importance=spatial_imp;
    normal_importance=normal_imp;
    disable_transform=disable_transf;
    refinement_iterations=refinement_it;
}

std::vector<geometry_msgs::Point> ShapeAnalyzer::get_translation_sequence(){
    return translation_sequence;
}

std::vector<double> ShapeAnalyzer::get_angle_sequence(){
    return angle_sequence;
}

std::vector<double> ShapeAnalyzer::get_distance_sequence(){
    return distance_variations;
}

void ShapeAnalyzer::generate_angles_components_structures(int node_id){
    //std::cout<<"n: "<<node_id<<" ";

    //loop through the angles in the possible angle set
    std::set<int> node_angles=possible_angles.at(node_id);
    if(node_id == 114)
        std::cout<<" +++++ 114 set size: "<<possible_angles.size()<<std::endl;
    std::vector<int> all_angle_components;
    //the std::set is ordered from low to high (lucky us)
    //start the first component, proceed until there is a jump of more than 5 degrees (angle_jump), then start a new component
    std::set<int> component_angles_subset;
    int angle_component=0;
    int prev_angle=0;
    int first_angle;
    std::set<int>::iterator it=node_angles.begin();
    //add this angle to the structure
    component_angles_subset.insert(*it);
    node_angle_to_angle_component.insert(std::pair<std::pair<int, int>, int>(std::pair<int, int>(node_id, *it), angle_component));
    prev_angle=*it;
    first_angle=*it;
    //if it only has one angle, only one component and return
    if(node_angles.size()<2){
        //node_angle_to_connected_angles_subset.insert(std::pair<std::pair<int, int>, std::set<int>*>(std::pair<int, int>(node_id, angle_component), component_angles_subset));
        all_angle_components.push_back(angle_component);
        node_component_to_angles_subset.insert(std::pair<std::pair<int, int>, std::set<int>>(std::pair<int, int>(node_id, angle_component), component_angles_subset));
        node_to_angle_components.insert(std::pair<int, std::vector<int>>(node_id, all_angle_components));
        return;
    }
    it++;
    //std::cout<<"components: "<<angle_component<<" ";
    //std::cout<<"----------debug"<<std::endl;
    for(; it!=node_angles.end(); it++){
        //std::cout<<*it<<" ";
        //check if there has been a bigger jump than angle_jump (all the angles are positive and as said the set is ordered)
        if((*it-prev_angle)>angle_jump){
            //add the angles obtained so far to the structure
            //node_angle_to_connected_angles_subset.insert(std::pair<std::pair<int, int>, std::set<int>*>(std::pair<int, int>(node_id, angle_component), component_angles_subset));
            all_angle_components.push_back(angle_component);
            if(node_id == 114)
                std::cout<<"++++++ 114 inserting component: "<<angle_component<<" of size: "<<component_angles_subset.size()<<std::endl;
            node_component_to_angles_subset.insert(std::pair<std::pair<int, int>, std::set<int>>(std::pair<int, int>(node_id, angle_component), component_angles_subset));
            //advance the component
            angle_component++;
            //std::cout<<angle_component<<" ";
            //clear the subset angles and add the current angle to it
            component_angles_subset=std::set<int>();    
        }
        //insert the angle in the set and store the prev value
        node_angle_to_angle_component.insert(std::pair<std::pair<int, int>, int>(std::pair<int, int>(node_id, *it), angle_component));
        component_angles_subset.insert(*it);
        prev_angle=*it;
    }
    
    //check if the last component is connected to the first one (last angle 360-angle jump and first angle 0)
    int last_angle=prev_angle;
    if(angle_component>0 &&first_angle==0 && last_angle==(360 - angle_jump)){
        if(node_id == 114)
            std::cout<<"merging"<<std::endl;
        //instead of adding this additional component, merge the current set with the first one
        std::set<int> first_subset=node_component_to_angles_subset.at(std::pair<int, int>(node_id, 0));
        std::set<int> union_set;
        std::set_union(first_subset.begin(), first_subset.end(), component_angles_subset.begin(), component_angles_subset.end(), std::inserter(union_set, union_set.begin()));
        if(node_id == 114)
            std::cout<<"    inserting component: "<<angle_component<<" of size: "<<union_set.size()<<std::endl;
        node_component_to_angles_subset.erase(std::pair<int, int>(node_id, 0));
        node_component_to_angles_subset.insert(std::pair<std::pair<int, int>, std::set<int>>(std::pair<int, int>(node_id, 0), union_set));
        //change all the nodes in the set into the first component
        for(std::set<int>::iterator angles_iterator=component_angles_subset.begin(); angles_iterator!=component_angles_subset.end(); angles_iterator++){
            node_angle_to_angle_component.at(std::pair<int, int>(node_id, *angles_iterator))=0;
        }
        node_to_angle_components.insert(std::pair<int, std::vector<int>>(node_id, all_angle_components));
        return;

    }
    //std::cout<<std::endl;
    //add the last component
    //node_angle_to_connected_angles_subset.insert(std::pair<std::pair<int, int>, std::set<int>>(std::pair<int, int>(node_id, angle_component), component_angles_subset));
    all_angle_components.push_back(angle_component);
    if(node_id == 114)
        std::cout<<"    +++inserting component: "<<angle_component<<" of size: "<<component_angles_subset.size()<<std::endl;
    node_component_to_angles_subset.insert(std::pair<std::pair<int, int>, std::set<int>>(std::pair<int, int>(node_id, angle_component), component_angles_subset));

    node_to_angle_components.insert(std::pair<int, std::vector<int>>(node_id, all_angle_components));

    return;

}

void ShapeAnalyzer::generate_connected_components_list_of_extended_refined_adjacency(){
//analyze the refined adjacency to generate a map of the connected components (BFS)
    std::set<std::pair<int, int>> examined_nodes;
    //std::cout<<"000000"<<std::endl;
    std::multimap<std::pair<int, int>, std::pair<int, int>>::iterator label_itr=extended_refined_adjacency.begin();
    int component_id=-1;
    //std::cout<<"NOOOOOODES::: "<<std::endl;
    for ( ; label_itr!=extended_refined_adjacency.end();) {
        //get the extended node "id" id
        //std::cout<<"BBBBBBB"<<std::endl;

        std::pair<int, int> node_label = label_itr->first;
        if(!(examined_nodes.find(node_label)!=examined_nodes.end())){
            //this node had not been analyzed before! It means it is a new connected component
            component_id++;
            int num_components=0;
            std::set<std::pair<int, int>> discovered_nodes;
            std::stack<std::pair<int, int>> S;
            S.push(node_label);
            while (!S.empty()){
                //std::cout<<"CCCCCCC"<<std::endl;
                std::pair<int, int> v=S.top();
                S.pop();
                if(discovered_nodes.find(v)==discovered_nodes.end()){
                    discovered_nodes.insert(v); //for the local bfs
                    examined_nodes.insert(v); //for the global analysis

                    extended_nodes_to_connected_component.insert(std::pair<std::pair<int, int>, int>(v, component_id));

                    //now get all the adjacent nodes
                    //std::cout<<v.first<<" "<<v.second<<std::endl;
                    //check if this node has anything adjacent first of all!
                    if(extended_refined_adjacency.count(v)>0){
                        std::multimap<std::pair<int, int>, std::pair<int, int>>::iterator adjacent_itr=extended_refined_adjacency.equal_range(v).first;
                        for ( ; adjacent_itr!=extended_refined_adjacency.equal_range(v).second; adjacent_itr++){
                            S.push(adjacent_itr->second);
                        }
                    }
                }
            }

            extended_connected_component_to_set_of_nodes_angle.insert(std::pair<int, std::set<std::pair<int, int>>>(component_id, discovered_nodes));
        }

        //move the iterator to the next label
        label_itr=extended_refined_adjacency.upper_bound(node_label);
    }

    std::cout<<"Found EXTENDED connected components: "<<component_id+1<<std::endl;
}

std::pair<std::stack<std::pair<int, int>>, std::stack<int>> ShapeAnalyzer::get_extended_path(std::multimap<std::pair<int, int>, std::pair<int, int>> graph, int init, int end, Eigen::Vector3f grasp_line, std::pair<int, int> opposite_component_init, std::pair<int, int> opposite_component_end, int initial_angle, int desired_angle){
    std::cout<<"computing (e)path from: "<<init<<" to "<<end<<std::endl;
    std::pair<std::stack<std::pair<int, int>>, std::stack<int>> empty;

    if(node_angle_to_angle_component.count(std::pair<int, int>(init, initial_angle))<1){
        std::cout<<"invalid value in node: "<<init<<" of angle "<<initial_angle<<std::endl;
        return empty;
    }
    if(node_angle_to_angle_component.count(std::pair<int, int>(end, desired_angle))<1){
        std::cout<<"invalid value in node: "<<end<<" of angle "<<desired_angle<<std::endl;
        return empty;
    }

    int initial_angle_component=node_angle_to_angle_component.at(std::pair<int, int>(init, initial_angle));
    int desired_angle_component=node_angle_to_angle_component.at(std::pair<int, int>(end, desired_angle));
    std::cout<<"initial angle component: "<<initial_angle_component<<std::endl;
    std::cout<<"desired angle component: "<<desired_angle_component<<std::endl;
    std::pair<int, int> init_node(init, initial_angle_component);
    std::pair<int, int> goal_node(end, desired_angle_component);

    if(extended_nodes_to_connected_component.count(goal_node)<1){
        std::cout<<"invalid value of node: "<<goal_node.first<<" "<<goal_node.second<<std::endl;
        return empty;
    }
    //check if a path exists. i.e. the elements are in the connencted component:
    if(extended_nodes_to_connected_component.at(init_node)!=extended_nodes_to_connected_component.at(goal_node)||
        extended_nodes_to_connected_component.at(opposite_component_init)!=extended_nodes_to_connected_component.at(opposite_component_end)){
        std::cout<<"The nodes are not connected. Regrasping is needed."<<std::endl;
        
        return empty;
    }

    
    

    //if the nodes are connected, it is possible to find a path! (Always)

    //check the normal to the opposite_component
    Eigen::Vector3f plane_normal=component_to_average_normal.at(nodes_to_connected_component.at(opposite_component_init.first));
    Eigen::Vector3f l0, p0;
    l0<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(init))).x, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(init))).y, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(init))).z;

    p0<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(opposite_component_init.first))).x, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(opposite_component_init.first))).y, 
        all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(uint32_t(opposite_component_init.first))).z;

    //insert the first distance
    double distance_between_contacts=(p0-l0).norm();
    extended_node_to_slave_distance.insert(std::pair<std::pair<int, int>, double>(init_node, distance_between_contacts));
    node_to_slave_distance.insert(std::pair<uint32_t, double>(uint32_t(init), distance_between_contacts));

    //do graph search here
    //create the set of keys (vertices)
    std::set<int> Q;
    std::vector<double> dist, dist_faked;
    std::map<std::pair<int, int>, int> node_to_int_idx;
    std::map<int, std::pair<int, int>> int_idx_to_node;
    int count=0;

    std::vector<int> prev;
    for(uint32_t i=0; i<all_centroids_cloud->points.size(); i++){
        int node_id=pc_to_supervoxel_idx.at(i);
        std::vector<int> angle_components=node_to_angle_components.at(node_id);
        //get all the angle components too
        for(int ii=0; ii<angle_components.size(); ii++){
            Q.insert(count);
            //std::cout<<"inserting: "<<i<<" "<<ii<<std::endl;
            node_to_int_idx.insert(std::pair<std::pair<int, int>, int>(std::pair<int, int>(node_id, ii), count));
            int_idx_to_node.insert(std::pair<int, std::pair<int, int>>(count, std::pair<int, int>(node_id, ii)));
            dist.push_back(DBL_MAX);
            dist_faked.push_back(DBL_MAX);
            prev.push_back(-1);
            count++;
        }
    }

    dist[node_to_int_idx.at(init_node)]=0;
    dist_faked[node_to_int_idx.at(init_node)]=0;

    //std::cout<<std::endl<<"---------------------------------------------"<<std::endl;
    //std::cout<<"data structures initialized for graph search."<<std::endl;

    int u;
    std::pair<int, int> u_node;
    std::stack<int> S;
    std::stack<std::pair<int, int>> S_node;
    //iterate over the vertices
    while(Q.size()>0){
        //std::cout<<"=========================================================================== size: "<<Q.size()<<std::endl;
        //get element with minimum distance
        std::vector<double>::iterator it=std::min_element(std::begin(dist_faked), std::end(dist_faked)); 
        u=std::distance(std::begin(dist_faked), it);
        u_node=int_idx_to_node.at(u);
        //remove this element from Q
        Q.erase(u);
        dist_faked[u]=DBL_MAX; //this is to look for shortest distance of elements still in Q
        //std::cout<<"current vertex: "<<u<<std::endl;
        //check if the element found is the goal

        if(u_node!=goal_node){
            //loop over all the neighbours of u (if u is in the adjacency map)
            if(graph.count(u_node)>0){
                std::multimap<std::pair<int, int>, std::pair<int, int>>::iterator adjacent_itr=graph.equal_range(u_node).first;
                for ( ; adjacent_itr!=graph.equal_range(u_node).second; adjacent_itr++){
                    //if this element is still in Q
                    //std::cout<<"current adjacent element "<<adjacent_itr->second.first<<" "<<adjacent_itr->second.second<<std::endl;
                    int element=node_to_int_idx.at(adjacent_itr->second);
                    //std::cout<<"OK 0"<<std::endl;
                    if(Q.find(element)!=Q.end()){
                        //compute the intersection between the opposite plane and the grasp line(assumed constant!)
                        //l0 is the current considered point
                        //std::cout<<"BBBBBB"<<std::endl;
                        l0<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->second.first)).x, 
                            all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->second.first)).y, 
                            all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->second.first)).z;
                        double alpha=-(l0-p0).dot(plane_normal)/(grasp_line.dot(plane_normal));
                        Eigen::Vector3f intersection_point=alpha*grasp_line+l0;
                        //std::cout<<"CCCCCC"<<std::endl;    
                        //now find the nearest centroid to this point, and check if it belongs to the same connected component(be sure to check that this node IS associaded to a component first)
                        int K = 3;//three nearest neighbours
                        std::vector<int> pointIdxNKNSearch(K);
                        std::vector<float> pointNKNSquaredDistance(K);
                        //improve this distance according to the change in normal (TO DO)
                        double slave_dist=100;
                        pcl::PointXYZRGBA input;
                        input.x=intersection_point(0);
                        input.y=intersection_point(1);
                        input.z=intersection_point(2);

                        //now add the distance between the current contact point and the obtained component to the map between distances and nodes (along the normal)
                        distance_between_contacts=(l0-intersection_point).norm();
                        extended_node_to_slave_distance.insert(std::pair<std::pair<int, int>, double>(adjacent_itr->second, distance_between_contacts));
                        node_to_slave_distance.insert(std::pair<uint32_t, double>(uint32_t(adjacent_itr->second.first), distance_between_contacts));

                        //std::cout<<"Checking the opposite component."<<std::endl;

                        if (centroids_kdtree.nearestKSearch(input, K, pointIdxNKNSearch, pointNKNSquaredDistance)> 0){
                            bool not_equal=true;
                            for (size_t i = 0; i < pointIdxNKNSearch.size () && not_equal; ++i){
                                //search for one component in the default connected component
                                //std::cout<<"DDDDDD"<<std::endl;    
                                std::vector<int> all_possible_angle_components=node_to_angle_components.at(pc_to_supervoxel_idx.at(pointIdxNKNSearch[i]));
                                //std::cout<<"OK"<<std::endl;    
                                for(int jj=0; jj<all_possible_angle_components.size(); jj++){
                                    //std::cout<<"EEEEEE"<<std::endl;    
                                    std::pair<int, int> checking_node(int(pc_to_supervoxel_idx.at(pointIdxNKNSearch[i])), all_possible_angle_components[jj]);
                                    //std::cout<<"OK"<<std::endl;
                                    //std::cout<<"node: "<<int(pc_to_supervoxel_idx.at(pointIdxNKNSearch[i]))<<" "<<all_possible_angle_components[jj]<<std::endl;
                                    //std::cout<<"FFFFFFF1: "<<extended_nodes_to_connected_component.at(checking_node)<<std::endl;
                                    //std::cout<<"FFFFFFF2: "<<extended_nodes_to_connected_component.at(opposite_component_init)<<std::endl;
                                    //if a node is not in this map, it means it is very disconnected
                                    if(extended_nodes_to_connected_component.count(checking_node)>0){         
                                        if(extended_nodes_to_connected_component.at(checking_node)==extended_nodes_to_connected_component.at(opposite_component_init)){
                                            slave_dist=0;
                                            //std::cout<<"The slave contact is in the connected component."<<std::endl;
                                            not_equal=false;
                                            break;
                                        }
                                    }
                                    //std::cout<<"OK1"<<std::endl;
                                }

                            }
                            //std::cout<<"OK2"<<std::endl;
                        }
                        else{
                            //there is something very wrong
                            std::cout<<"ERROR: cold not associate slave contact to centroid"<<std::endl;
                        }

                        //compute the real distance between the nodes now
                        Eigen::Vector3f source_point;
                        source_point<<all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->first.first)).x, 
                            all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->first.first)).y, 
                            all_centroids_cloud->points.at(supervoxel_to_pc_idx.at(adjacent_itr->first.first)).z;

                        double node_dist=(l0-source_point).norm();

                        //the distance can be modified by the opposite finger if it is not in the same connected component
                        double alt=dist[u]+node_dist+slave_dist;
                        int dist_idx=node_to_int_idx.at(adjacent_itr->second);
                        if(alt<dist[dist_idx]){
                            //a new shortest path has been found
                            dist[dist_idx]=alt;
                            dist_faked[dist_idx]=alt;
                            prev[dist_idx]=u;
                        }
                    }
                }
            }

        }
        else{
            std::cout<<"Found Path!"<<std::endl;
            //do something
            while(prev[u]!=-1){
                std::cout<<"u: "<<u<<std::endl;
                S.push(int_idx_to_node.at(u).first);
                S_node.push(int_idx_to_node.at(u));
                u=prev[u];
            }
            S.push(int_idx_to_node.at(u).first);
            S_node.push(int_idx_to_node.at(u));

            break; 
        }
    }
    if(Q.size()==0){
        std::cout<<"No path has been found"<<std::endl;
        return std::pair<std::stack<std::pair<int, int>>, std::stack<int>>(S_node, S);
    }

    std::cout<<"Found (e)path of length: "<<S.size()-1<<std::endl;
    return std::pair<std::stack<std::pair<int, int>>, std::stack<int>>(S_node, S);
}

void ShapeAnalyzer::compute_extended_path(int finger_id){
    //first of all, clear the previously drawn path and delete the previous translation sequence
    int max_line_id=translation_sequence.size()-1;
    translation_sequence = std::vector<geometry_msgs::Point>();
    angle_sequence=std::vector<double>();
    distance_variations=std::vector<double>();
    for(int i=0; i<=max_line_id; i++){
        viewer->removeShape("line "+std::to_string(i));
    }

    //compute the line between the two contact points;
    Eigen::Vector3f grasp_point1, grasp_point2, grasp_line;
    grasp_point1<<initial_pose_1(0, 0), initial_pose_1(1, 0), initial_pose_1(2, 0);
    grasp_point2<<initial_pose_2(0, 0), initial_pose_2(1, 0), initial_pose_2(2, 0);
    //std::cout<<"grasp points: "<<std::endl<<grasp_point1.transpose()<<std::endl<<grasp_point2.transpose()<<std::endl;
    grasp_line=grasp_point2-grasp_point1;
    grasp_line.normalize();
    //std::cout<<"initial grasp line: "<<grasp_line.transpose()<<std::endl;
    //check if the desired contact is valid
    grasp_point1=Eigen::Vector3f(desired_pose_1(0, 0), desired_pose_1(1, 0), desired_pose_1(2, 0));
    grasp_point2=Eigen::Vector3f(desired_pose_2(0, 0), desired_pose_2(1, 0), desired_pose_2(2, 0));
    //std::cout<<"desired points: "<<std::endl<<grasp_point1.transpose()<<std::endl<<grasp_point2.transpose()<<std::endl;
    Eigen::Vector3f desired_grasp_line=grasp_point2-grasp_point1;
    desired_grasp_line.normalize();
    //std::cout<<"desired grasp line: "<<desired_grasp_line.transpose()<<std::endl;
    //std::cout<<"diff vector: "<<(grasp_line-desired_grasp_line).transpose()<<std::endl;
    //std::cout<<"diff norm: "<<(grasp_line-desired_grasp_line).norm()<<std::endl;
    if(fabs((grasp_line-desired_grasp_line).norm())>0.3){
        std::cout<<"Wrong final pose. Rotation around different axes could be required."<<std::endl;
        return;
    }

    //compute the initial and final angles
    std::pair<std::pair<int, int>, std::pair<int, int>> int_angles=get_int_angles(finger_id);
    if(int_angles.first.first==-1){
        std::cout<<"Unachievable desired path"<<std::endl;
        return;
    }

    std::stack<std::pair<int, int>> S_extended;
    std::stack<int> S;
    std::pair<std::stack<std::pair<int, int>>, std::stack<int>> S_solution;
    std::vector<std::pair<int, int>> path_extended;
    std::vector<int> path;
    if(finger_id==1){
        std::pair<int, int> opposite_component_init(initial_centroid_idx_2, node_angle_to_angle_component.at(std::pair<int, int>(initial_centroid_idx_2, int_angles.second.first)));
        std::pair<int, int> opposite_component_end(desired_centroid_idx_2, node_angle_to_angle_component.at(std::pair<int, int>(desired_centroid_idx_2, int_angles.second.second)));
        S_solution=get_extended_path(extended_refined_adjacency, initial_centroid_idx_1, desired_centroid_idx_1, grasp_line, opposite_component_init, opposite_component_end, int_angles.first.first, int_angles.first.second);
    }
    else{
        std::pair<int, int> opposite_component_init(initial_centroid_idx_1, node_angle_to_angle_component.at(std::pair<int, int>(initial_centroid_idx_1, int_angles.second.first)));
        std::pair<int, int> opposite_component_end(desired_centroid_idx_1, node_angle_to_angle_component.at(std::pair<int, int>(desired_centroid_idx_1, int_angles.second.second)));
        S_solution=get_extended_path(extended_refined_adjacency, initial_centroid_idx_2, desired_centroid_idx_2, -grasp_line, opposite_component_init, opposite_component_end, int_angles.first.first, int_angles.first.second);
    }

    S_extended=S_solution.first;
    S=S_solution.second;


    if(S.size()<1){
        return;
    }

    //then draw the lines to highligt the path and store the points to keep track of the necessary translations
    //get the first point
    int idx;
    std::pair<int, int> idx_extended;
    pcl::PointXYZRGBA point1;
    pcl::PointXYZRGBA point2;
    idx=S.top();
    idx_extended=S_extended.top();
    S.pop();
    S_extended.pop();
    path.push_back(idx);
    path_extended.push_back(idx_extended);
    //std::cout<<"node: "<<idx<<std::endl;
    //std::cout<<"node: "<<idx_extended.first<<" "<<idx_extended.second<<std::endl;
    point1=supervoxel_clusters.at(idx)->centroid_;

    //variables to store the translation sequence
    geometry_msgs::Point p1, p1Scaled;
    p1.x=point1.x;
    p1.y=point1.y;
    p1.z=point1.z;
    p1Scaled.x=point1.x/1000.0;
    p1Scaled.y=point1.y/1000.0;
    p1Scaled.z=point1.z/1000.0;
    translation_sequence.push_back(p1Scaled);

    //get the new elements and draw the line
    int count=0;
    while(S_extended.size()>0){
        idx=S.top();
        idx_extended=S_extended.top();
        S.pop();
        S_extended.pop();
        path.push_back(idx);
        path_extended.push_back(idx_extended);
        //std::cout<<"node: "<<idx<<std::endl;
        //std::cout<<"node: "<<idx_extended.first<<" "<<idx_extended.second<<std::endl;
        //std::cout<<"idx"<<idx<<std::endl;
        point2=supervoxel_clusters.at(idx)->centroid_;
        //store this in the translation sequence
        geometry_msgs::Point p2, p2Scaled;
        p2.x=point2.x;
        p2.y=point2.y;
        p2.z=point2.z;
        p2Scaled.x=point2.x/1000.0;
        p2Scaled.y=point2.y/1000.0;
        p2Scaled.z=point2.z/1000.0;
        translation_sequence.push_back(p2Scaled);
        viewer->addLine(point1, point2, 0, 1, 0, "line "+std::to_string(count));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line "+std::to_string(count));
        count++;
        //std::cout<<"line drawn: "<<count<<std::endl;
        point1=point2;
    }
    //std::cout<<"TRANSLATION SEQUENCE::::::"<<translation_sequence.size()<<std::endl;  

    //compute angle sequence
    //std::cout<<"BBBBBBB"<<std::endl;
    compute_extended_angle_sequence(path_extended, finger_id, int_angles.first.first, int_angles.first.second); 
    //compute distance sequence
    compute_contact_distances(path);
}

std::pair<std::pair<int, int>, std::pair<int, int>> ShapeAnalyzer::get_int_angles(int finger_id){
    Eigen::Matrix3f desired_gripper_to_base, initial_gripper_to_base, desired_component_to_base, initial_component_to_base;
    Eigen::Matrix3f slave_desired_gripper_to_base, slave_initial_gripper_to_base, slave_desired_component_to_base, slave_initial_component_to_base;
    Eigen::Quaternion<float> q1, q2, slave_q1, slave_q2;
    uint32_t initial_contact_index, slave_initial_contact_index;
    //if the returned value contains -1 it means that the path cannot be achieved
    std::pair<std::pair<int, int>, std::pair<int, int>> empty_pair(std::pair<int, int>(-1, -1), std::pair<int, int>(-1, -1));
    if(finger_id==1){
        initial_contact_index=uint32_t(initial_centroid_idx_1);
        slave_initial_contact_index=uint32_t(initial_centroid_idx_2);
    }
    else{
        initial_contact_index=uint32_t(initial_centroid_idx_2);
        slave_initial_contact_index=uint32_t(initial_centroid_idx_1);
    }

    int initial_contact_connected_component=nodes_to_connected_component.at(initial_contact_index);
    int slave_initial_contact_connected_component=nodes_to_connected_component.at(slave_initial_contact_index);
    Eigen::Vector3f component_normal=component_to_average_normal.at(initial_contact_connected_component);
    Eigen::Vector3f slave_component_normal=component_to_average_normal.at(slave_initial_contact_connected_component);
    Eigen::Vector3f nx, ny, nz;
    Eigen::Vector3f slave_nx, slave_ny, slave_nz;
    nx=component_normal;
    slave_nx=slave_component_normal;
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
    nz=nx.cross(ny);

    if(fabs(slave_nx(0))>0.00000001){
        slave_ny(1)=0;
        slave_ny(2)=sqrt(slave_nx(0)*slave_nx(0)/(slave_nx(0)*slave_nx(0)+slave_nx(2)*slave_nx(2)));
        slave_ny(0)=-slave_nx(2)*slave_ny(2)/slave_nx(0);
    }
    else if(fabs(nx(1))>0.00000001){
        slave_ny(2)=0;
        slave_ny(0)=sqrt(slave_nx(1)*slave_nx(1)/(slave_nx(1)*slave_nx(1)+slave_nx(0)*slave_nx(0)));
        slave_ny(1)=-slave_ny(0)*slave_nx(0)/slave_nx(1);
    }
    else{
        slave_ny(0)=0;
        slave_ny(1)=sqrt(slave_nx(2)*slave_nx(2)/(slave_nx(1)*slave_nx(1)+slave_nx(2)*slave_nx(2)));
        slave_ny(2)=-slave_nx(1)*slave_ny(1)/slave_nx(2);
    }
    slave_nz=slave_nx.cross(slave_ny);

    Eigen::Matrix3f component_to_base, slave_component_to_base;
    component_to_base(0,0)=nx(0);
    component_to_base(1,0)=nx(1);
    component_to_base(2,0)=nx(2);

    component_to_base(0,1)=ny(0);
    component_to_base(1,1)=ny(1);
    component_to_base(2,1)=ny(2);

    component_to_base(0,2)=nz(0);
    component_to_base(1,2)=nz(1);
    component_to_base(2,2)=nz(2);

    slave_component_to_base(0,0)=slave_nx(0);
    slave_component_to_base(1,0)=slave_nx(1);
    slave_component_to_base(2,0)=slave_nx(2);

    slave_component_to_base(0,1)=slave_ny(0);
    slave_component_to_base(1,1)=slave_ny(1);
    slave_component_to_base(2,1)=slave_ny(2);

    slave_component_to_base(0,2)=slave_nz(0);
    slave_component_to_base(1,2)=slave_nz(1);
    slave_component_to_base(2,2)=slave_nz(2);


    //2) matrix from base to gripper pose
    uint32_t desired_contact_index, slave_desired_contact_index;
    if(finger_id==1){
        desired_contact_index=uint32_t(desired_centroid_idx_1);
        slave_desired_contact_index=uint32_t(desired_centroid_idx_2);
        q1=Eigen::Quaternion<float>(initial_pose_1(6), initial_pose_1(3), initial_pose_1(4), initial_pose_1(5)); //Eigen has the w before in the quaternion
        q2=Eigen::Quaternion<float>(desired_pose_1(6), desired_pose_1(3), desired_pose_1(4), desired_pose_1(5));
        slave_q1=Eigen::Quaternion<float>(initial_pose_2(6), initial_pose_2(3), initial_pose_2(4), initial_pose_2(5)); //Eigen has the w before in the quaternion
        slave_q2=Eigen::Quaternion<float>(desired_pose_2(6), desired_pose_2(3), desired_pose_2(4), desired_pose_2(5));
    }
    else{
        desired_contact_index=uint32_t(desired_centroid_idx_2);
        slave_desired_contact_index=uint32_t(desired_centroid_idx_1);
        q1=Eigen::Quaternion<float>(initial_pose_2(6), initial_pose_2(3), initial_pose_2(4), initial_pose_2(5));
        q2=Eigen::Quaternion<float>(desired_pose_2(6), desired_pose_2(3), desired_pose_2(4), desired_pose_2(5));
        slave_q1=Eigen::Quaternion<float>(initial_pose_1(6), initial_pose_1(3), initial_pose_1(4), initial_pose_1(5)); //Eigen has the w before in the quaternion
        slave_q2=Eigen::Quaternion<float>(desired_pose_1(6), desired_pose_1(3), desired_pose_1(4), desired_pose_1(5));
    }


    initial_gripper_to_base=q1.toRotationMatrix();
    desired_gripper_to_base=q2.toRotationMatrix();

    slave_initial_gripper_to_base=slave_q1.toRotationMatrix();
    slave_desired_gripper_to_base=slave_q2.toRotationMatrix();

    // std::cout<<"slave initial gripper to base: "<<std::endl<<slave_initial_gripper_to_base<<std::endl<<std::endl;
    // std::cout<<"slave desired gripper to base: "<<std::endl<<slave_desired_gripper_to_base<<std::endl<<std::endl;
    // std::cout<<"slave component to base: "<<std::endl<<slave_component_to_base<<std::endl<<std::endl;
    // std::cout<<"slave component to base transpose: "<<std::endl<<slave_component_to_base.transpose()<<std::endl<<std::endl;

    //now get the vector Z' of the gripper expressed in the component's reference frame
    Eigen::Matrix3f initial_gripper_to_component=component_to_base.transpose()*initial_gripper_to_base;
    Eigen::Matrix3f desired_gripper_to_component=component_to_base.transpose()*desired_gripper_to_base;

    Eigen::Matrix3f slave_initial_gripper_to_component=slave_component_to_base.transpose()*slave_initial_gripper_to_base;
    Eigen::Matrix3f slave_desired_gripper_to_component=slave_component_to_base.transpose()*slave_desired_gripper_to_base;

    // std::cout<<"initial gripper to component: "<<std::endl<<initial_gripper_to_component<<std::endl<<std::endl;
    // std::cout<<"desired gripper to component: "<<std::endl<<desired_gripper_to_component<<std::endl<<std::endl;

    //the first column of the matrix is the x vector of the gripper. this one has to be inverted and then the angle w.r.t. the y axis of the gripper can be found
    Eigen::Vector3f x_prime=initial_gripper_to_component.block<3, 1>(0, 0);
    //std::cout<<"x_prime init: "<<x_prime.transpose()<<std::endl;
    double initial_angle=atan2(-x_prime(2), -x_prime(1));
    if (initial_angle<0){
        initial_angle+=2*M_PI;
    }
    x_prime=desired_gripper_to_component.block<3, 1>(0, 0);
    //std::cout<<"x_prime des: "<<x_prime.transpose()<<std::endl;
    double desired_angle=atan2(-x_prime(2), -x_prime(1));
    if (desired_angle<0){
        desired_angle+=2*M_PI;
    }

    //convert this angle into degrees and with intervals of 5 (angle_jump variable)
    initial_angle=initial_angle*180.0/M_PI;
    desired_angle=desired_angle*180.0/M_PI;
    double round_angle=initial_angle+angle_jump/2;
    //convert in int
    int int_initial_angle=floor(round_angle);
    //convert from 360 to 0:
    int_initial_angle=int_initial_angle-(int_initial_angle%angle_jump);
    if(int_initial_angle==360){
        int_initial_angle=0;
    }
    //same for desired angle
    round_angle=desired_angle+angle_jump/2;
    int int_desired_angle=floor(round_angle);
    int_desired_angle=int_desired_angle-(int_desired_angle%angle_jump);
    if(int_desired_angle==360){
        int_desired_angle=0;
    }

    std::cout<<"Initial angle: "<<int_initial_angle<<std::endl;
    std::cout<<"Desired angle: "<<int_desired_angle<<std::endl;

    //do the same thing with the slave contact:
    Eigen::Vector3f slave_x_prime=slave_initial_gripper_to_component.block<3, 1>(0, 0);
    //std::cout<<"x_prime init: "<<x_prime.transpose()<<std::endl;
    double slave_initial_angle=atan2(-slave_x_prime(2), -slave_x_prime(1));
    if (slave_initial_angle<0){
        slave_initial_angle+=2*M_PI;
    }
    slave_x_prime=slave_desired_gripper_to_component.block<3, 1>(0, 0);
    //std::cout<<"x_prime des: "<<x_prime.transpose()<<std::endl;
    double slave_desired_angle=atan2(-slave_x_prime(2), -slave_x_prime(1));
    if (slave_desired_angle<0){
        slave_desired_angle+=2*M_PI;
    }

    //convert this angle into degrees and with intervals of 5
    slave_initial_angle=slave_initial_angle*180.0/M_PI;
    slave_desired_angle=slave_desired_angle*180.0/M_PI;
    double slave_round_angle=slave_initial_angle+angle_jump/2;
    //convert in int
    int slave_int_initial_angle=floor(slave_round_angle);
    //convert from 360 to 0:
    slave_int_initial_angle=slave_int_initial_angle-(slave_int_initial_angle%angle_jump);
    if(slave_int_initial_angle==360){
        slave_int_initial_angle=0;
    }
    //same for desired angle
    slave_round_angle=slave_desired_angle+angle_jump/2;
    int slave_int_desired_angle=floor(slave_round_angle);
    slave_int_desired_angle=slave_int_desired_angle-(slave_int_desired_angle%angle_jump);
    if(slave_int_desired_angle==360){
        slave_int_desired_angle=0;
    }

    //std::cout<<"Slave Initial angle: "<<slave_int_initial_angle<<std::endl;
    //std::cout<<std::endl;

    //std::cout<<"Index: "<<index<<std::endl;
    //now cicle backwards the nodes to compute the sequence of angles
    std::set<int> current_node_angles=possible_angles.at(desired_contact_index);
    std::set<int> next_node_angles;
    int current_angle=int_desired_angle;
    //std::cout<<"master desired angle: "<<current_angle<<std::endl;
    //std::cout<<"master desired contact idx: "<<desired_contact_index<<std::endl;
    //first of all check if it is possible. If it is not, well... the list of angles will be empty and the task is impossible to solve
    if(current_node_angles.find(current_angle)==current_node_angles.end()){
        std::cout<<"The desired angle cannot be achieved."<<std::endl;
        return empty_pair;
    }
    //check also for the slave finger final pose:
    std::set<int> slave_current_node_angles=possible_angles.at(slave_desired_contact_index);
    int slave_current_angle=slave_int_desired_angle;
    //std::cout<<"slave desired angle: "<<slave_current_angle<<std::endl;
    //std::cout<<"slave desired contact idx: "<<slave_desired_contact_index<<std::endl;
    if(slave_current_node_angles.find(slave_current_angle)==slave_current_node_angles.end()){
        std::cout<<"The desired angle cannot be achieved."<<std::endl;
        return empty_pair;
    }

    //first the master int angle (initial, desired) and then the slave
    return std::pair<std::pair<int, int>, std::pair<int, int>>(std::pair<int, int>(int_initial_angle, int_desired_angle), std::pair<int, int>(slave_int_initial_angle, slave_current_angle));

} 

void ShapeAnalyzer::compute_extended_angle_sequence(std::vector<std::pair<int, int>> path, int finger_id, int init_angle, int desired_angle){
    int index=0;
    std::set<int> current_node_angles=node_component_to_angles_subset.at(path[index]);
    std::cout<<"Initial index: "<<index<<std::endl;
    std::cout<<"size: "<<current_node_angles.size()<<std::endl;
    std::set<int> prev_node_angles;
    angle_sequence=std::vector<double>();
    angle_sequence.resize(path.size());
    //angle_sequence[path.size()-1]=double(desired_angle)*M_PI/180.0;
    angle_sequence[index]=double(init_angle)*M_PI/180.0;
    index=index+1;
    int current_angle=init_angle;
    std::cout<<"Index: "<<index<<std::endl;
    while(index<path.size()){
        prev_node_angles=current_node_angles;
        if(node_component_to_angles_subset.count(path[index])<1){
            std::cout<<"Error in getting angle subset"<<std::endl;
            return;
        }
        std::cout<<"Index: "<<index<<std::endl;
        current_node_angles=node_component_to_angles_subset.at(path[index]);
        std::cout<<"size: "<<current_node_angles.size()<<std::endl;
        std::set<int> intersection;
        std::set_intersection(current_node_angles.begin(), current_node_angles.end(), prev_node_angles.begin(), prev_node_angles.end(), std::inserter(intersection, intersection.begin()));
        //check if the current angle is in the intersection (i.e. the translation can be with the gripper at this angle) 
        if(intersection.find(current_angle)!=intersection.end()){
            angle_sequence[index]=double(current_angle)*M_PI/180.0;
        }
        else{
            if(intersection.size()==0){
                std::cout<<"error"<<std::endl;
                std::cout<<"The intersection is empty! The adjacency is wrong. Index: "<<index<<std::endl;
            }
            else{//in this case the element is random. Choose one that is closer to the one we already have
                current_angle=*intersection.begin();
            }
            angle_sequence[index]=double(current_angle)*M_PI/180.0;
        }
        index=index+1;
        std::cout<<"Index: "<<index<<std::endl;
    }

    //now change the list so that it becomes a list of deltas
    double last_angle=desired_angle*M_PI/180.0; //this is the last angle
    for(int i=angle_sequence.size()-1 ; i>=0; i--){
        std::cout<<"angle ["<<i<<"] = "<<angle_sequence[i]<<std::endl;
        angle_sequence[i]=last_angle-angle_sequence[i];
        last_angle=last_angle-angle_sequence[i];//uptade to the next angle
    }
}