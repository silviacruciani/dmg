/**
 *  extended_dmg.cpp
 *
 *  Created on: March 14 2018
 *      Author: Silvia Cruciani
*/

#include "shape_analysis/extended_dmg.hpp"

using namespace shape_analysis;

ExtendedDMG::ExtendedDMG() : ShapeAnalyzer(){
    r_translations=std::vector<std::vector<geometry_msgs::Point>>(3);
    r_rotations=std::vector<std::vector<double>>(3);
    r_finger_distances=std::vector<std::vector<double>>(3);
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
    return 1;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> ExtendedDMG::get_regrasp_points(){
    return std::pair<Eigen::Vector3f, Eigen::Vector3f>(regrasp1, regrasp2);
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

