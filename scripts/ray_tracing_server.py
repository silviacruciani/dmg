#!/usr/bin/python

"""
    This file contains a server to compute intersections between a ray and an stl mesh

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
from geometry_msgs.msg import Point
from shape_analysis.srv import *
import numpy as np
from pycaster import pycaster
import rospkg
import tf.transformations

class RayTracingServer():
    """docstring for RayTracingServer"""
    def __init__(self):
        self._caster = None
        self._shape_name = None

        pkg = rospkg.RosPack()
        #self._object_path = pkg.get_path('shape_analysis') + '/shapes/'
        self._object_path = ""

    def obtain_intersections(self, req):
        name = req.mesh_name
        if self._shape_name is not name:
            #load this new mesh
            self._caster = pycaster.rayCaster.fromSTL(self._object_path + name, scale=1.0)
            self._shape_name = name

        #now get the first and last points of the radius (when calling, these must be scaled if all the intersections along one directions are desired)
        source = [req.start_point.x, req.start_point.y, req.start_point.z]
        destination = [req.end_point.x, req.end_point.y, req.end_point.z]
        intersections = self._caster.castRay(source, destination)
        intersections_points = list()

        #return the list of intersections as list of points
        for p in intersections: 
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = p[2]
            intersections_points.append(point)

        response = RayTracingResponse()
        response.intersections = intersections_points
        return response


if __name__ == '__main__':
    rospy.init_node('ray_tracing_server')
    ray_tracing = RayTracingServer()
    s = rospy.Service('ray_tracing', RayTracing, ray_tracing.obtain_intersections)
    rospy.spin()