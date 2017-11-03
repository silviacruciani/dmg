/*
 *  shape_analyzer.hpp
 *
 *  Created on: October 24 2017
 *      Author: Silvia Cruciani
*/

#ifndef SHAPE_ANALYSIS_SHAPE_ANALIZER
#define SHAPE_ANALYSIS_SHAPE_ANALIZER

#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/supervoxel_clustering.h>
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
#include <pcl/features/normal_3d.h>
//#include <thread>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

namespace shape_analysis{
    class ShapeAnalyzer{
        public:
            ShapeAnalyzer();
            ~ShapeAnalyzer();
            void set_object_from_pointcloud(std::string file_name);
            void set_object_from_mesh(std::string file_name);
            void subdivide_object(double subcube_dimension);
            void get_supervoxels();
            void spin_viewer();
            void set_initial_contact(geometry_msgs::Point p, geometry_msgs::Quaternion q, int finger_id);
            void set_desired_contact(geometry_msgs::Point p, geometry_msgs::Quaternion q, int finger_id);
            void compute_path(int finger_id);
            void set_finger_length(double l);
            void refine_adjacency(); //this checks for simple collisions with the gripper

        private:
            void addSupervoxelConnectionsToViewer(pcl::PointXYZRGBA &supervoxel_center,
                                  pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer* viewer,
                                  int viewport);
            bool return_manipulation_sequence(); //create service file for this
            int connect_centroid_to_contact(geometry_msgs::Point p, geometry_msgs::Quaternion q, std::string id, bool render_sphere);
            std::stack<int> get_path(std::multimap<uint32_t,uint32_t> graph, int init, int end);
            
            double l_finger;
            pcl::PointCloud<pcl::PointXYZ>::Ptr object_shape;
            pcl::visualization::PCLVisualizer *viewer;
            int v1, v2; //viewports
            std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud;
            pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud;
            pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud;
            pcl::KdTreeFLANN<pcl::PointXYZRGBA> centroids_kdtree;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr all_centroids_cloud; //contains olnly the center of the supervoxels
            pcl::PointCloud<pcl::Normal>::Ptr all_normals_clouds;
            std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
            std::multimap<uint32_t, uint32_t> refined_adjacency;
            int initial_centroid_idx_1; //first finger
            int initial_centroid_idx_2; //second finger
            int desired_centroid_idx_1; //first finger
            int desired_centroid_idx_2; //second finger
            //std::thread *viewer_thread;
            std::map <int, uint32_t> pc_to_supervoxel_idx;
            std::map <uint32_t, int> supervoxel_to_pc_idx;
            std::vector<uint32_t> centroids_ids;
    };
}

#endif