/*
 * extendedDMG_server.hpp
 *
 *  Created on: April 24, 2018
 *      Author: Silvia Cruciani
*/

#ifndef SHAPE_ANALYSIS_EXTENDEDDMG_SERVER
#define SHAPE_ANALYSIS_EXTENDEDDMG_SERVER

#include "ros/ros.h"
#include "shape_analysis/extended_dmg.hpp"
#include "shape_analysis/ExtendedInHandPath.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <ros/package.h>

namespace shape_analysis{
    class ExtendedDMGServer{
    public:
        ExtendedDMGServer();
        ~ExtendedDMGServer();
        void init(ros::NodeHandle &n);
        void spinOnce();
        bool compute_path(ExtendedInHandPath::Request &req, ExtendedInHandPath::Response &res);

    private:
        ExtendedDMG* shape_analizer;
        ros::ServiceServer service;        
    };

}

#endif