#!/usr/bin/python2

"""
    This file contains a server to check the collision between a polygon (4 vertices) and the object

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
import numpy as np
import rospkg
import vtk
from shape_analysis.srv import *

class CollisionCheckServer():
    """docstring for CollisionCheck"""
    def __init__(self):
        self._shape_name = None
        pkg = rospkg.RosPack()
        self._object_path = pkg.get_path('shape_analysis') + '/shapes/'

        #reader for vtk
        self._reader = vtk.vtkSTLReader()
        self._shape = None #saved as polydata
        self._shapeTri = None

        self._booleanOperation = vtk.vtkBooleanOperationPolyDataFilter()
        self._booleanOperation.SetOperationToIntersection()


    def check(self, req):
        name = req.mesh_name
        if self._shape_name is not name:
            #load this new mesh in vtk
            self._reader.SetFileName(self._object_path + name)
            self._shape_name = name
            self._reader.Update()
            self._shape = self._reader.GetOutput()
            self._shapeTri = vtk.vtkTriangleFilter()
            self._shapeTri.SetInputData(self._shape)
            

        #setup the 4 vertices of the rectangle
        points = vtk.vtkPoints()
        points.InsertNextPoint(req.v1.x, req.v1.y, req.v1.z)
        points.InsertNextPoint(req.v2.x, req.v2.y, req.v2.z)
        points.InsertNextPoint(req.v3.x, req.v3.y, req.v3.z)
        points.InsertNextPoint(req.v4.x, req.v4.y, req.v4.z)

        # Create the polygon
        polygon = vtk.vtkPolygon()
        polygon.GetPointIds().SetNumberOfIds(4)  # make a quad
        polygon.GetPointIds().SetId(0, 0)
        polygon.GetPointIds().SetId(1, 1)
        polygon.GetPointIds().SetId(2, 2)
        polygon.GetPointIds().SetId(3, 3)

        # Add the polygon to a list of polygons
        polygons = vtk.vtkCellArray()
        polygons.InsertNextCell(polygon)

        # Create a PolyData
        polygonPolyData = vtk.vtkPolyData()
        polygonPolyData.SetPoints(points)
        polygonPolyData.SetPolys(polygons)

        #now check if they are in collision
        polygonTri = vtk.vtkTriangleFilter()
        polygonTri.SetInputData(polygonPolyData)

        self._booleanOperation.SetInputConnection(0, self._shapeTri.GetOutputPort())
        self._booleanOperation.SetInputConnection(1, polygonTri.GetOutputPort())
        self._booleanOperation.Update()

        intersection = self._booleanOperation.GetOutput()
        points = intersection.GetPoints()

        response = CollisionCheckResponse()
        if intersection is None:
            response.collision = False
        elif points.GetNumberOfPoints()>0:
            response.collision = True
        else:
            response.collision = False

        return response



if __name__ == '__main__':
    rospy.init_node('collision_check_server')
    cc = CollisionCheckServer()
    s = rospy.Service('collision_check', CollisionCheck, cc.check)
    rospy.spin()

        