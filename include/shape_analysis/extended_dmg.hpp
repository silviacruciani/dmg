/**
 *  extended_dmg.hpp
 *
 *  Created on: March 14 2018
 *      Author: Silvia Cruciani
*/

#ifndef SHAPE_ANALYSIS_EXTENDED_DMG
#define SHAPE_ANALYSIS_EXTENDED_DMG


#include "shape_analysis/in_hand_path_server.hpp"
#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "shape_analysis/RayTracing.h"
#include "shape_analysis/CollisionCheck.h"
#include "geometry_msgs/Pose.h"


namespace shape_analysis{
    class ExtendedDMG : public ShapeAnalyzer{
    public:
        ExtendedDMG(ros::NodeHandle n);
        ~ExtendedDMG();

        /**
            Computes the complete sequence of regrasps and in-hand manipulations.
            @param finger_id the id of the principal finger
            @return -1 if no solution is found, 0 if no regrasp is nedeed, 1 otherwise.
        */
        int compute_extended_path(int finger_id);

        /**
            Returns the two grasp points, for the second gripper and the first gripper in this order.
            @return two std::pair with two Eigen::Vector3f in the object's reference frame.
        */
        std::pair<std::pair<Eigen::Vector3f, Eigen::Vector3f>, std::pair<Eigen::Vector3f, Eigen::Vector3f>> get_regrasp_points();

        /**
            Returns all the necessary translations for the contact points.
            @return 3 translation sequences of geometry_msgs::Point for the 3 regrasps steps.
        */
        std::vector<std::vector<geometry_msgs::Point>> get_translation_sequence();

        /**
            Returns all the necessary rotations.
            @return 3 rotations sequences for the 3 regrasps steps.
        */
        std::vector<std::vector<double>> get_angle_sequence();

        /**
            Returns the distance between the fingers during the translations.
            @return 3 distance sequences corresponding to the 3 translation sequences.
        */
        std::vector<std::vector<double>> get_distance_sequence();

        /**
            sets the name used for the ray tracing service.
            @param the name of the service.
        */
        void set_ray_tracing_service_name(std::string name);        

        /**
            sets the name used for the angle collision check service.
            @param the name of the service.
        */
        void set_angle_collision_service_name(std::string name);

        /**
            adds all the debug visualization to a viewer
        */
        void visualize_results();

        /**
            overrides the parent's function to spin both the basic viewer and the extended DMG viewer
        */
        void spin_viewer_once();

        /**
            overrides the parent's function and adds the stl file also to the regrasp viewer
            @param file_name the name of the pointcloud file
        */
        void set_object_from_pointcloud(std::string file_name);

        /**
            checks if a normal at the contact is inwards or outwards
            @param contact the contact point with the surface
            @param normal the normal direction to be tested
            @return true if inwards, false if outwards
        */
        bool is_normal_inwards(Eigen::Vector3f contact, Eigen::Vector3f normal);


        /**
            return the regrasp pose from the regrasp point and angle
            @param gripper the gripper id (0 for 1st gripper, >0 for 2nd gripper)
            @return the regrasp pose
        */
        geometry_msgs::Pose get_regrasp_pose(int gripper);



    private:
        /**
            Gets the intersection from a ray caster server.
            @param start the Vector3f start point of the ray
            @param end the Vector3f end point of the ray
            @return a vector with all the intersections, ordered from the closest to the starting point onwards.
        */
        std::vector<Eigen::Vector3f> get_ray_intersections(Eigen::Vector3f start, Eigen::Vector3f end);

        /**
            Finds a set of available regrasping points close to the given contact and keeps the translations and rotations to 
            connect them to the desired grasp
            @param principal_contact the contact point of the principal finger
            @param secondary_contact the contact point of the secondary finger
            @param principal_angle the angle of the finger in the principal connected component
            @param secondary_angle the angle of the finger in the secondary connected component
            @param gripper_id 0 for the first gripper, 1 for the second
        */
        void find_available_regrasping_points(Eigen::Vector3f principal_contact, Eigen::Vector3f secondary_contact, int principal_angle, int secondary_angle, int gripper_id);


        /**
            Checks if the finger is in collision in that point with the given angle
            @param contact the finger contact point
            @param angle the finger's angle
            @return true if there is collision, false otherwise
        */
        bool is_in_collision(Eigen::Vector3f contact, double angle);

        /**
            Obtains the supervoxel centroid closest to the contact
            @param contact the finger contact point
            @param scale true if the point is not in the mm scale of the object
            @return the supervoxel index
        */
        int get_supervoxel_index(Eigen::Vector3f contact, bool scale);

        /**
            Gets the normal vector of the surface in contact
            @param contact the contact point
            @return the normal vector
        */
        Eigen::Vector3f get_normal_at_contact(Eigen::Vector3f contact);

        /**
            Gets the direction of the y axis in the connected component, which is the one at which the angle is 0
            @param contact the finger contact point
            @return the axis vector
        */
        Eigen::Vector3f get_zero_angle_direction(Eigen::Vector3f contact);

        /**
            Get the corresponding angle for the given gripper pose
            @param q the quaternion with the finger orientation
            @param component the connected component of the contact point
            @return the int angle (degrees)
        */
        int pose_to_angle(Eigen::Quaternion<float> q, int component);

        /**
            Gets the quaternion corresponding to a grasp on the given contact point with the given angle
            @param angle the angle at the contact point
            @param contact the 3D contact point
            @return the corresponding quaternion of the gripper
        */
        Eigen::Quaternion<float> angle_to_pose(double angle, Eigen::Vector3f contact);

        /**
            Finds the orientation of the frame of the connected components as rotation matrix
            @param component the component index
            @return the rotation matrix
        */
        Eigen::Matrix3f component_pose_matrix(int component);

        /**
            Converts the angle from radians to degrees in the given resolution
            @param angle the angle in radians
            @return the int angle in degrees
        */
        int filter_angle(double angle);

        /**
            finds an axis orthogonal to the given one
            @param nx the input axis
            @return the orthogonal axis
        */
        Eigen::Vector3f get_orthogonal_axis(Eigen::Vector3f nx);

        /**
            Obtains the corresponding nodes of the secondary finger, given the principal finger, with the given direction (orientation) of the gripper
            @param direction the line connecting the principal and secondary contact points
            @param principal_node the node corresponding to the principal finger
            @return a list with the opposite nodes for the secondary finger
        */
        std::vector<std::pair<int, int>> get_opposite_finger_nodes(Eigen::Vector3f direction, std::pair<int, int> principal_node);

        /**
            To each node in the graph, assigns a value according to the distance from the two grasping point of the first gripper
            @param release_contact_1 the principal contact of the 1st gripper before releasing the object
            @param release_contact_2 the secondary contact of the 1st gripper before releasing the object
            @param regrasp_contact_1 the principal contact of the 1st gripper when regrasping
            @param regrasp_contact_2 the secondary contact of the 1st gripper when regrasping
            @return a mat from node to double containing how "good" a node is for the 2nd gripper regrasping. The bigger the value, the better.
        */
        std::map<int, double> weight_regrasping_area(Eigen::Vector3f release_contact_1, Eigen::Vector3f release_contact_2, Eigen::Vector3f regrasp_contact_1, Eigen::Vector3f regrasp_contact_2);


        /**
            Given a regrasp candidate and the current gripping pose and future regrasp pose, returns the best angle
            @param contact1_principal the candidate contact point principal
            @param contact1_secondary the candidate contact point secondary
            @param point1_principal the current gripping point principal
            @param point1_secondary the current gripping point secondary
            @param grasping_angle1 the current grasping angle principal
            @param point2_principal the candidate regrasping point principal
            @param point2_secondary the candidate regrasping point secondary
            @param grasping_angle2 the candidate regrasping angle principal (can be -1 and adds freedom of choiche. If that's the case, its value is modified)
            @return the best angle for regrasping, if there is any available. -1 otherwise
        */
        int get_collision_free_regrasp_angle(Eigen::Vector3f contact1_principal, Eigen::Vector3f contact1_secondary, Eigen::Vector3f point1_principal, Eigen::Vector3f point1_secondary, int grasping_angle1, Eigen::Vector3f point2_principal, Eigen::Vector3f point2_secondary, int &grasping_angle2);
        
        /**
            Draws the regrasping poses for the fingers
            @param name the identifier for the object in the viewer
            @param position the 3D position (in mm)
            @param orientation the 3D orientation
            @param color_profile 0, 1, 2 depending on the gripper and regrasp
            @param viewer 0 if first viewer, 1 if regrasp viewer
        */
        void draw_finger(std::string name, Eigen::Vector3f position, Eigen::Quaternion<float> orientation, int color_profile, bool is_egrasp_viewer);

        /**
            Draws the given path
            @param path the path (nodes) to draw
            @param sequence a string to identify what sequence it is (e.g. the 1st_gripper_regrasp)
        */
        void draw_path(std::vector<geometry_msgs::Point> path, std::string sequence);

        //regrasp 1st gripper, regrasp 2nd gripper
        Eigen::Vector3f regrasp1_principal, regrasp2_principal; //for the principal contact point
        Eigen::Vector3f regrasp1_secondary, regrasp2_secondary; //for the secondary contact point
        std::vector<std::vector<geometry_msgs::Point>> r_translations;
        std::vector<std::vector<double>> r_rotations;
        std::vector<std::vector<double>> r_finger_distances;
        double regrasp1_angle, regrasp2_angle;
        int regrasp2_node1, regrasp2_node2;

        std::vector<std::vector<std::pair<int, int>>> regrasping_candidate_nodes; //for the 1st and 2nd gripper, all the good nodes for regrasping (regrasp area)
        std::map<std::pair<int, int>, double> regrasp_poses_distance_values; //a regrasp pose is defined by two points on the object's surface 

        std::string ray_tracing_service_name;
        std::string angle_collision_service_name;
        ros::NodeHandle node_handle;
        ros::ServiceClient ray_tracing_client;
        ros::ServiceClient angle_collision_client;

        double max_fingers_opening_mm, regrasp_distance_threshold;

        //weights of the nodes according to the distance from regrasping areas
        std::map<int, double> nodes_distances_from_regrasps;

        //viewer for debug
        pcl::visualization::PCLVisualizer *regrasp_viewer;
        
    };
}

#endif