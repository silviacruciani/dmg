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
            Returns the two grasp points, for the second gripper and the first gripper.
            @return a std::pair with two Eigen::Vector3f in the object's reference frame.
        */
        std::pair<Eigen::Vector3f, Eigen::Vector3f> get_regrasp_points();

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


    private:
        /**
            Get the intersection from a ray caster server.
            @return a vector with all the intersections, ordered from the closest to the starting point onwards.
        */
        std::vector<Eigen::Vector3f> get_ray_intersections(Eigen::Vector3f start, Eigen::Vector3f end);


        //regrasp 1st gripper, regrasp 2nd gripper
        Eigen::Vector3f regrasp1, regrasp2;
        std::vector<std::vector<geometry_msgs::Point>> r_translations;
        std::vector<std::vector<double>> r_rotations;
        std::vector<std::vector<double>> r_finger_distances;

        std::string ray_tracing_service_name;
        ros::NodeHandle node_handle;
        ros::ServiceClient ray_tracing_client;
        
    };
}

#endif