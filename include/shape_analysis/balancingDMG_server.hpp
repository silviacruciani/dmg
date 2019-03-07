/*
 * balancingDMG_server.hpp
 *
 *  Created on: MArch 7, 2019
 *      Author: Silvia Cruciani
*/

#ifndef SHAPE_ANALYSIS_BALANCING_DMG_SERVER
#define SHAPE_ANALYSIS_BALANCING_DMG_SERVER

#include "ros/ros.h"
#include "shape_analysis/balancing_dmg.hpp"
#include "shape_analysis/InHandPath.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <ros/package.h>

namespace shape_analysis{
    class BalancingDMGServer{
    public:
        BalancingDMGServer();
        ~BalancingDMGServer();
        void init(ros::NodeHandle &n);
        void spinOnce();
        bool compute_path(InHandPath::Request &req, InHandPath::Response &res);

    private:
        BalancingDMG shape_analizer;
        ros::ServiceServer service;        
    };

}

#endif