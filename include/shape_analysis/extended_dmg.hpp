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
            @param desired_angle if the angle to be checked is the desired one or not
        */
        void find_available_regrasping_points(Eigen::Vector3f principal_contact, Eigen::Vector3f secondary_contact, bool desired_angle);


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
            @return the supervoxel index
        */
        int get_supervoxel_index(Eigen::Vector3f contact);

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


        //regrasp 1st gripper, regrasp 2nd gripper
        Eigen::Vector3f regrasp1_principal, regrasp2_principal; //for the principal contact point
        Eigen::Vector3f regrasp1_secondary, regrasp2_secondary; //for the secondary contact point
        std::vector<std::vector<geometry_msgs::Point>> r_translations;
        std::vector<std::vector<double>> r_rotations;
        std::vector<std::vector<double>> r_finger_distances;

        std::string ray_tracing_service_name;
        std::string angle_collision_service_name;
        ros::NodeHandle node_handle;
        ros::ServiceClient ray_tracing_client;
        ros::ServiceClient angle_collision_client;
        
    };
}

#endif