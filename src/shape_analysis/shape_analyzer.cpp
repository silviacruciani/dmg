/*
 *  shape_analyzer.cpp
 *
 *  Created on: October 24 2017
 *      Author: Silvia Cruciani
*/

#include "shape_analysis/shape_analyzer.hpp"
#include <iostream>


using namespace shape_analysis;

ShapeAnalyzer::ShapeAnalyzer(){
    //object_shape=pcl::PointCloud<pcl::PointXYZ>::Ptr();
}

ShapeAnalyzer::~ShapeAnalyzer(){

}

void ShapeAnalyzer::set_object_from_pointcloud(std::string file_name){
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    if ( pcl::io::loadPCDFile(file_name, *in_cloud)<0){
        std::cout<<"Error loading model cloud."<<std::endl;
    }
    object_shape=in_cloud;
    //visualizer to be used for debug purposes
    viewer=new pcl::visualization::PCLVisualizer ("3D Viewer");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
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

void ShapeAnalyzer::get_supervoxels(){
    //get the parameters from somewhere. For now use default things
    bool disable_transform=true; //the transformation has to be disabled for organized pointclouds
    float voxel_resolution=1.0;
    float seed_resolution =20.0;
    float color_importance = 0.0;
    float spatial_importance = 0.9;
    float normal_importance = 1.0;

    //now use the supervoxels
    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
    if(disable_transform){
        super.setUseSingleCameraTransform (false);
    }
    PointCloudT::Ptr cloud=boost::shared_ptr <PointCloudT> (new PointCloudT ());
    pcl::copyPointCloud(*object_shape,*cloud);
        

    super.setInputCloud(cloud);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);

    pcl::console::print_highlight ("Extracting supervoxels!\n");
    super.extract (supervoxel_clusters);
    pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
    super.refineSupervoxels(10, supervoxel_clusters);
    voxel_centroid_cloud=super.getVoxelCentroidCloud();
    std::cout<<"VoxelCentroidCloud size: "<<voxel_centroid_cloud->points.size()<<std::endl;

    viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "voxel centroids");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

    labeled_voxel_cloud = super.getLabeledVoxelCloud ();
    viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "labeled voxels");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

    //get the normals
    sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
    std::cout<<"VoxelNormalCloud size: "<<sv_normal_cloud->points.size()<<std::endl;
    viewer->addPointCloudNormals<pcl::PointNormal> (sv_normal_cloud,1, 5.0, "supervoxel_normals");


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr centroids_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //iterate over the supervoxels to get the REAL centroids cloud
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
        //necessary mapping from index in the point cloud to supervoxel label:
        pc_to_supervoxel_idx.insert(std::pair<int, uint32_t>(pc_idx, x.first));
        pc_idx++;
        //std::cout<<"CentroidsCloud size: "<<centroids_cloud->points.size()<<std::endl;
    }

    all_centroids_cloud=centroids_cloud;
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
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        PointCloudT adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; adjacent_itr++){
          pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
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
        addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }

    //spin_viewer();
}

void ShapeAnalyzer::addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer* viewer){

    vtkSmartPointer<vtkPoints> points=vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells=vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyLine> polyLine=vtkSmartPointer<vtkPolyLine>::New();

    //Iterate through all adjacent points, and add a center point to adjacent point pair
    PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
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
    viewer->addModelFromPolyData (polyData,supervoxel_name);
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
    }
    else{
        initial_centroid_idx_2=connect_centroid_to_contact(p, q, "initial centroid "+std::to_string(finger_id), true);
    }
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
    }
    else{
        desired_centroid_idx_2=connect_centroid_to_contact(p, q, "desired centroid "+std::to_string(finger_id), true);
    }
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
    std::stack<int> S;
    std::vector<int> path;
    if(finger_id==1){
        S=get_path(supervoxel_adjacency, initial_centroid_idx_1, desired_centroid_idx_1);
    }
    else{
        S=get_path(supervoxel_adjacency, initial_centroid_idx_2, desired_centroid_idx_2);
    }

    //then draw the lines to highligt the path
    //get the first point
    int idx;
    pcl::PointXYZRGBA point1;
    pcl::PointXYZRGBA point2;
    idx=S.top();
    S.pop();
    path.push_back(idx);
    point1=supervoxel_clusters.at(idx)->centroid_;
    //get the new elements and draw the line
    int count=0;
    while(S.size()>0){
        idx=S.top();
        S.pop();
        point2=supervoxel_clusters.at(idx)->centroid_;
        viewer->addLine(point1, point2, 1, 1, 0, "line "+std::to_string(count));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line "+std::to_string(count));
        count++;
        //std::cout<<"line drawn: "<<count<<std::endl;
        point1=point2;
    }    
}
