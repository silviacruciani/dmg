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
    //visualizer to be used for debug purposes
    viewer=new pcl::visualization::PCLVisualizer ("3D Viewer");
    int v(0);
    v1=v;
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addCoordinateSystem(1.0);
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
    pcl::PointXYZRGBA input;
    input.x=p.x*1000.0;
    input.y=p.y*1000.0;
    input.z=p.z*1000.0;
    //later quaternion

    if(finger_id==1){
        initial_centroid_idx_1=connect_centroid_to_contact(p, q, "initial centroid "+std::to_string(finger_id), true);
        desired_pose_1<<p.x, p.y, p.z, q.x, q.y, q.z, q.w;
    }
    else{
        initial_centroid_idx_2=connect_centroid_to_contact(p, q, "initial centroid "+std::to_string(finger_id), true);
        desired_pose_2<<p.x, p.y, p.z, q.x, q.y, q.z, q.w;
    }
    viewer->removeShape("initial contact "+std::to_string(finger_id));
    viewer->addCube(Eigen::Vector3f(input.x, input.y, input.z), Eigen::Quaternionf(q.x, q.y, q.z, q.w), 3, 3, 3, "initial contact "+std::to_string(finger_id));
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
        initial_pose_1<<p.x, p.y, p.z, q.x, q.y, q.z, q.w;
    }
    else{
        desired_centroid_idx_2=connect_centroid_to_contact(p, q, "desired centroid "+std::to_string(finger_id), true);
        initial_pose_2<<p.x, p.y, p.z, q.x, q.y, q.z, q.w;
    }
    viewer->removeShape("desired contact "+std::to_string(finger_id));
    viewer->addCube(Eigen::Vector3f(input.x, input.y, input.z), Eigen::Quaternionf(q.x, q.y, q.z, q.w), 3, 3, 3, "desired contact "+std::to_string(finger_id));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "desired contact "+std::to_string(finger_id));
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "desired contact "+std::to_string(finger_id));

}

std::stack<int> ShapeAnalyzer::get_path(std::multimap<uint32_t,uint32_t> graph, int init, int goal){
    std::cout<<"computing path from: "<<init<<" to "<<goal<<std::endl;
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
    std::cout<<"data structures initialized for graph search."<<std::endl;

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
            std::multimap<uint32_t,uint32_t>::iterator adjacent_itr=supervoxel_adjacency.equal_range(uint32_t(u)).first;
            for ( ; adjacent_itr!=supervoxel_adjacency.equal_range(uint32_t(u)).second; adjacent_itr++){
                //if this element is still in Q
                //std::cout<<"current adjacent element "<<adjacent_itr->second<<std::endl;
                if(Q.find(int(adjacent_itr->second))!=Q.end()){
                    double alt=dist[u]+1.0; //all the edges assumed at distance 1
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
                std::cout<<"u: "<<u<<std::endl;
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
    //first of all, clear the previously drawn path
    int max_line_id=translation_sequence.size()-1;
    for(int i=0; i<=max_line_id; i++){
        viewer->removeShape("line "+std::to_string(i));
    }

    std::stack<int> S;
    std::vector<int> path;
    if(finger_id==1){
        S=get_path(supervoxel_adjacency, initial_centroid_idx_1, desired_centroid_idx_1);
    }
    else{
        S=get_path(supervoxel_adjacency, initial_centroid_idx_2, desired_centroid_idx_2);
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
        viewer->addLine(point1, point2, 1, 1, 0, "line "+std::to_string(count));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line "+std::to_string(count));
        count++;
        //std::cout<<"line drawn: "<<count<<std::endl;
        point1=point2;
    }  

    compute_angle_sequence(path, finger_id);  
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
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel=supervoxel_clusters.at (supervoxel_label);
        //now get the normal of this supervoxel
        pcl::Normal supervoxel_normal=supervoxel->normal_;

        //Now we need to iterate through the adjacent supervoxels to check their normals
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers; //this will be used for debug
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr=supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range(supervoxel_label).second; adjacent_itr++){
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel=supervoxel_clusters.at(adjacent_itr->second);
            pcl::Normal adjacent_normal=neighbor_supervoxel->normal_;
            //now check if these two normals have a similar direction
            if(fabs((supervoxel_normal.normal_x-adjacent_normal.normal_x)*(supervoxel_normal.normal_x-adjacent_normal.normal_x)+
                (supervoxel_normal.normal_y-adjacent_normal.normal_y)*(supervoxel_normal.normal_y-adjacent_normal.normal_y)+
                (supervoxel_normal.normal_z-adjacent_normal.normal_z)*(supervoxel_normal.normal_z-adjacent_normal.normal_z))<0.07){ //check what value to put there

                //add this to the visualization debug
                adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
                //add this to the refined adjacency
                refined_adjacency.insert(std::pair<uint32_t, uint32_t>(supervoxel_label, adjacent_itr->second));
            }
            //check if the normal is pointing in the opposite direction (inwards instead of outwards)
            else if(fabs((supervoxel_normal.normal_x+adjacent_normal.normal_x)*(supervoxel_normal.normal_x+adjacent_normal.normal_x)+
                (supervoxel_normal.normal_y+adjacent_normal.normal_y)*(supervoxel_normal.normal_y+adjacent_normal.normal_y)+
                (supervoxel_normal.normal_z+adjacent_normal.normal_z)*(supervoxel_normal.normal_z+adjacent_normal.normal_z))<0.07){

                //add this to the visualization debug
                adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
                //add this to the refined adjacency
                refined_adjacency.insert(std::pair<uint32_t, uint32_t>(supervoxel_label, adjacent_itr->second));
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
        //get the supervoxel id
        uint32_t supervoxel_label = label_itr->first;
        if(!(examined_nodes.find(supervoxel_label)!=examined_nodes.end())){
            //this node had not been analyzed before!
            component_id++;
            int num_components=0;
            Eigen::Vector3f sum_normal;
            sum_normal<<0.0, 0.0, 0.0;

            Eigen::Vector3f reference_normal;
            reference_normal<<supervoxel_clusters.at(supervoxel_label)->normal_.normal_x,
                        supervoxel_clusters.at(supervoxel_label)->normal_.normal_y,
                        supervoxel_clusters.at(supervoxel_label)->normal_.normal_z;

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

                    nodes_to_connected_component.insert(std::pair<uint32_t, int>(v, component_id));
                    Eigen::Vector3f normal;
                    normal<<supervoxel_clusters.at(v)->normal_.normal_x,
                        supervoxel_clusters.at(v)->normal_.normal_y,
                        supervoxel_clusters.at(v)->normal_.normal_z;
                    //check if the normal has not a huge discordance:
                    if((reference_normal-normal).norm()<0.03){
                        //average the normal
                        sum_normal=sum_normal+normal;
                        num_components++;
                    }
                    else{
                        //average the normal but change sign
                        sum_normal=sum_normal-normal;
                        num_components++;
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
    for(int i=0; i<360; i+=5){
        all_angles.insert(i);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Vector3f x, y, z;
    x<<1, 0, 0;
    y<<0, 1, 0;
    z<<0, 0, 1;

    pcl::PassThrough<pcl::PointXYZ> pass;

    for(int component=0; component<=component_id; component++){
        std::cout<<"component: "<<component<<std::endl;
        //get the average normal of the component:
        Eigen::Vector3f nx, ny, nz;
        nx=component_to_average_normal.at(component);
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
        Eigen::Matrix3f R;
        R(0,0)=nx.dot(x);
        R(0,1)=nx.dot(y);
        R(0,2)=nx.dot(z);

        R(1,0)=ny.dot(x);
        R(1,1)=ny.dot(y);
        R(1,2)=ny.dot(z);

        R(2,0)=nz.dot(x);
        R(2,1)=nz.dot(y);
        R(2,2)=nz.dot(z);

        /*transform.block<3,3>(0,0)=R;*/
        //for the translation, cicle through all the nodes in the component
        std::vector<uint32_t> nodes(connected_component_to_set_of_nodes.at(component).begin(), connected_component_to_set_of_nodes.at(component).end());
        for(int idx=0; idx<nodes.size(); idx++){
            std::cout<<"    node: "<<nodes[idx]<<std::endl;
            int pc_idx=supervoxel_to_pc_idx.at(nodes[idx]);
            pcl::PointXYZRGBA c=all_centroids_cloud->at(pc_idx);
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
            std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
            std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

            std::set<int> angles_per_node(all_angles);

            std::cout<<"        impossible angles: ";
            if(kdtree.radiusSearch(searchPoint, l_finger+1.0, pointIdxRadiusSearch, pointRadiusSquaredDistance)>0){
                pcl::PointCloud<pcl::PointXYZ>::Ptr neighbour_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                for(int i=0; i<pointIdxRadiusSearch.size(); i++){
                    //check if the distance is larger
                    if(pointRadiusSquaredDistance[i]>=l_finger*l_finger){
                        //create a 3D vector:
                        Eigen::Vector3f point;
                        point<<object_shape->at(pointIdxRadiusSearch[i]).x, object_shape->at(pointIdxRadiusSearch[i]).y, object_shape->at(pointIdxRadiusSearch[i]).z;
                        Eigen::Vector3f transformed_point=R*point;
                        //use the coordinates of the point in the normal reference frame to compute the elevation
                        double elevation=asin(transformed_point(0)/sqrt(pointRadiusSquaredDistance[i]));
                        //now check if the elevation is in the defined interval:
                        if(fabs(elevation)<0.1){ //10 degrees total
                            //std::cout<<"point: "<<transformed_point.transpose()<<std::endl;
                            //add the point to the poincloud for debug
                            neighbour_cloud->push_back(object_shape->at(pointIdxRadiusSearch[i]));
                            //check the point and associate it with the corresponding angle
                            //the x axis corresponds to the normal. the ny axis corresponds to the 0 angle
                            double angle=atan2(transformed_point(2), transformed_point(1));
                            //convert the angle into degrees:
                            angle=angle*180.0/M_PI;
                            //std::cout<<"    angle: "<<angle<<std::endl;
                            //approximate the angle in multiples of 5
                            double round_angle=angle+5/2;
                            //round_angle-=floor(round_angle)%5;
                            //convert in int
                            int int_round_angle=floor(round_angle);
                            //convert from 360 to 0:
                            if(int_round_angle==360){
                                int_round_angle=0;
                            }
                            int int_angle=int_round_angle-(int_round_angle%5);
                            if(int_angle%5!=0){
                                std::cout<<"ERROR IN THE ROUNDING TO 5"<<std::endl;
                            }
                            if (angles_per_node.find(int_angle)!=angles_per_node.end()){
                                angles_per_node.erase((int_angle)); //does it work if the set already doesn't have the angle?
                                std::cout<<int_angle<<", ";
                            }
                        }
                    }
                }
                //add the neighbor pointcloud to the viewer
                std::stringstream ss;
                ss<<"neighbor_" <<nodes[idx];
                //random color
                /*pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> hc(neighbour_cloud);
                viewer->addPointCloud (neighbour_cloud, hc, ss.str(), v2);
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, ss.str());
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, ss.str());*/
            }
            std::cout<<std::endl;

            //now add this mapping between the supervoxel and the possible angles
            possible_angles.insert(std::pair<uint32_t, std::set<int>>(nodes[idx], angles_per_node));
        }

        //great. Now add this to the visualizer so we can debug? for now it is very difficult
    }
    //now further refine the adjacency by removing all the impossible connections and impossible vertices
    std::cout<<"Refining the adjacency list according to the possible angles"<<std::endl;
    //loop on all the connected components
    std::multimap<uint32_t, uint32_t> angles_refined_adjacency;
    for(int component=0; component<=component_id; component++){
        //get all the nodes in the component:
        std::set<uint32_t> nodes_set=connected_component_to_set_of_nodes.at(component);
        //now loop on all the nodes and check if their connections (or the node itself) have to be removed from the connected component
        for(std::set<uint32_t>::iterator it=nodes_set.begin(); it!=nodes_set.end(); it++){
            std::set<int> node_angles=possible_angles.at(*it);
            //if this node has no possible angles, it will be removed from the connected component and not added to the new adjacency map
            uint32_t node=*it;
            if(node_angles.size()<1){
                connected_component_to_set_of_nodes.at(component).erase(node);
                nodes_to_connected_component.erase(node);
            }
            else{
                pcl::PointXYZRGBA node_center=supervoxel_clusters.at(node)->centroid_;
                pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
                //now loop over all the components
                std::multimap<uint32_t,uint32_t>::iterator adjacent_itr=refined_adjacency.equal_range(node).first;
                for ( ; adjacent_itr!=refined_adjacency.equal_range(node).second; adjacent_itr++){
                    //get the intersection between the two 
                    std::set<int> angles_intersection;
                    std::set_intersection(node_angles.begin(), node_angles.end(), possible_angles.at(adjacent_itr->second).begin(), possible_angles.at(adjacent_itr->second).end(), std::inserter(angles_intersection, angles_intersection.begin()));
                    if(angles_intersection.size()>0){
                        angles_refined_adjacency.insert(std::pair<uint32_t, uint32_t>(node, adjacent_itr->second));
                        adjacent_supervoxel_centers.push_back(supervoxel_clusters.at(adjacent_itr->second)->centroid_);
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
}

void ShapeAnalyzer::compute_angle_sequence(std::vector<int> path, int finger_id){
    //get the desired angle from the desired pose (angle between the axiz z ("projected" on ny and nz) around nx, with ny being the 0 angle)
    //obtain the rotation matrix to transform the pose into the nx ny nz reference frame:
    Eigen::Matrix3f desired_gripper_to_base, initial_gripper_to_base, desired_component_to_base, initial_component_to_base;
    Eigen::Quaternion<float> q1, q2;

    //1) matrix from base frame to nx ny nz
    //get the component of the initial contact
    uint32_t initial_contact_index;
    if(finger_id==1){
        initial_contact_index=uint32_t(initial_centroid_idx_1);
    }
    else{
        initial_contact_index=uint32_t(initial_centroid_idx_2);
    }

    int initial_contact_connected_component=nodes_to_connected_component.at(initial_contact_index);
    Eigen::Vector3f component_normal=component_to_average_normal.at(initial_contact_connected_component);
    Eigen::Vector3f nx, ny, nz;
    nx=component_normal;
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

    Eigen::Matrix3f component_to_base;
    component_to_base(0,0)=nx(0);
    component_to_base(0,1)=nx(1);
    component_to_base(0,2)=nx(2);

    component_to_base(1,0)=ny(0);
    component_to_base(1,1)=ny(1);
    component_to_base(1,2)=ny(2);

    component_to_base(2,0)=nz(0);
    component_to_base(2,1)=nz(1);
    component_to_base(2,2)=nz(2);


    //2) matrix from base to gripper pose
    uint32_t desired_contact_index;
    if(finger_id==1){
        desired_contact_index=uint32_t(desired_centroid_idx_1);
        q1=Eigen::Quaternion<float>(initial_pose_1(3), initial_pose_1(4), initial_pose_1(5), initial_pose_1(6));
        q2=Eigen::Quaternion<float>(desired_pose_1(3), desired_pose_1(4), desired_pose_1(5), desired_pose_1(6));
    }
    else{
        desired_contact_index=uint32_t(desired_centroid_idx_2);
        q1=Eigen::Quaternion<float>(initial_pose_2(3), initial_pose_2(4), initial_pose_2(5), initial_pose_2(6));
        q2=Eigen::Quaternion<float>(desired_pose_2(3), desired_pose_2(4), desired_pose_2(5), desired_pose_2(6));
    }


    initial_gripper_to_base=q1.toRotationMatrix();
    desired_gripper_to_base=q2.toRotationMatrix();

    //now get the vector Z' of the gripper expressed in the component's reference frame
    Eigen::Matrix3f initial_gripper_to_component=component_to_base.transpose()*initial_gripper_to_base;
    Eigen::Matrix3f desired_gripper_to_component=component_to_base.transpose()*desired_gripper_to_base;
    //the last column of the matrix is the z vector. its projection on the nz ny plane are the second and third components
    Eigen::Vector3f z_prime=initial_gripper_to_component.block<3, 1>(0, 2);
    double initial_angle=atan2(z_prime(2), z_prime(1));
    if (initial_angle<0){
        initial_angle+=M_PI;
    }
    z_prime=desired_gripper_to_component.block<3, 1>(0, 2);
    double desired_angle=atan2(z_prime(2), z_prime(1));
    if (desired_angle<0){
        desired_angle+=M_PI;
    }

    //convert this angle into degrees and with intervals of 5
    initial_angle=initial_angle*180.0/M_PI;
    desired_angle=desired_angle*180.0/M_PI;
    double round_angle=initial_angle+5/2;
    //convert in int
    int int_initial_angle=floor(round_angle);
    //convert from 360 to 0:
    if(int_initial_angle==360){
        int_initial_angle=0;
    }
    int_initial_angle=int_initial_angle-(int_initial_angle%5);
    //same for desired angle
    round_angle=desired_angle+5/2;
    int int_desired_angle=floor(round_angle);
    if(int_desired_angle==360){
        int_desired_angle=0;
    }
    int_desired_angle=int_desired_angle-(int_desired_angle%5);

    std::cout<<"Initial angle: "<<int_initial_angle<<std::endl;
    std::cout<<"Desired angle: "<<int_desired_angle<<std::endl;

    int index=path.size()-1;
    std::cout<<"Index: "<<index<<std::endl;
    //now cicle backwards the nodes to compute the sequence of angles
    std::set<int> current_node_angles=possible_angles.at(desired_contact_index);
    std::set<int> next_node_angles;
    int current_angle=int_desired_angle;
    //first of all check if it is possible. If it is not, well... the list of angles will be empty and the task is impossible to solve
    if(current_node_angles.find(current_angle)==current_node_angles.end()){
        std::cerr<<"The desired angle cannot be achieved."<<std::endl;
        return;
    }
    //convert the angle back to double
    angle_sequence.resize(path.size());
    angle_sequence[index]=double(desired_angle)*M_PI/180.0;
    index=index-1;
    std::cout<<"Index: "<<index<<std::endl;
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
                std::cerr<<"The intersection is empty! The adjacency is wrong."<<std::endl;
            }
            //in this case the element is random. Choose one that is closer to the one we already have
            current_angle=*intersection.begin();
            angle_sequence[index]=double(current_angle)*M_PI/180.0;
        }
        index=index-1;
        std::cout<<"Index: "<<index<<std::endl;
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