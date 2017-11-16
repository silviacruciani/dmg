/*
 * in_hand_path_server.hpp
 *
 *  Created on: November 15, 2017
 *      Author: Silvia Cruciani
*/

#ifndef SHAPE_ANALYSIS_IN_HAND_PATH_SERVER
#define SHAPE_ANALYSIS_IN_HAND_PATH_SERVER

#include "ros/ros.h"
#include "shape_analysis/shape_analyzer.hpp"
#include "shape_analysis/InHandPath.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <ros/package.h>

namespace shape_analysis{
    class InHandPathServer{
    public:
        InHandPathServer();
        ~InHandPathServer();
        void init(ros::NodeHandle &n);
        void spinOnce();
        bool compute_path(InHandPath::Request &req, InHandPath::Response &res);

    private:
        ShapeAnalyzer shape_analizer;
        ros::ServiceServer service;        
    };

}

#endif