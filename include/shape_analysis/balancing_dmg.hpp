/**
 *  balancing_dmg.hpp
 *
 *  Created on: March 7 2019
 *      Author: Silvia Cruciani
*/

#ifndef SHAPE_ANALYSIS_BALANCING_DMG
#define SHAPE_ANALYSIS_BALANCING_DMG

#include "shape_analysis/shape_analyzer.hpp"

namespace shape_analysis{
    class BalancingDMG : public ShapeAnalyzer{
    public:
        BalancingDMG();
        ~BalancingDMG();

        /**
            Draws all the fingers with their possible orientation at each DMG node to show the angular component
        */
        void drawAllFingers();

        /**
            Saves the DMG components in a file
            @param file_name the name of the file
        */
        void saveDMGComponents(std::string file_path, std::string file_name);

    private:

        /**
            Draws the fingers given the pose
            @param name the identifier for the object in the viewer
            @param position the 3D position (in mm)
            @param orientation the 3D orientation
            @param r the r color value
            @param g the g color value
            @param b the b color value
        */
        void draw_finger(std::string name, Eigen::Vector3f position, Eigen::Quaternion<float> orientation, double r, double g, double b);
        
    };
}

#endif