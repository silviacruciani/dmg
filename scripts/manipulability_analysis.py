#!/usr/bin/python

"""
    This file contains an analyzer for a the manipulability of a specific object. It tests different poses to see if there exists a path between them

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
import math
from shape_analysis.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from pycaster import pycaster
import rospkg
import tf.transformations

def call_server(server_name, pose1, pose2, pose1d, pose2d):
    rospy.wait_for_service(server_name, timeout = 5.0)
    try:
        manipulation_path = rospy.ServiceProxy(server_name, InHandPath)
        req = InHandPathRequest()
        req.initial_grasp.append(pose1)
        req.initial_grasp.append(pose2)
        req.desired_grasp.append(pose1d)
        req.desired_grasp.append(pose2d)
        # print ("Sending sequest: ", req)
        resp = manipulation_path(req)
        # print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return 0
    #check the response to see if the solution is valid or not
    if resp.translations is None:
        return 0
    elif len(resp.translations) < 1 or len(resp.rotations) < 1:
        return 0
    elif (len(resp.translations)< 2) and abs(resp.rotations[0]) < 0.1 :   
        return 0
    else:
        return 1

def computeFingersPoses(x1, x2, y1, y2, z1, z2, roll, pitch, yaw):
    """This function generates ros compatible poses given the inputs"""
    p1 = Point()
    p2 = Point()
    q1 = Quaternion()
    q2 = Quaternion()

    p1.x = x1/1000.0
    p1.y = y1/1000.0
    p1.z = z1/1000.0
    p2.x = x2/1000.0
    p2.y = y2/1000.0
    p2.z = z2/1000.0

    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    q1.x = quat[0]
    q1.y = quat[1]
    q1.z = quat[2]
    q1.w = quat[3]
    q2.x = quat[0]
    q2.y = quat[1]
    q2.z = quat[2]
    q2.w = quat[3]

    pose1 = Pose()
    pose2 = Pose()
    pose1.position = p1
    pose1.orientation = q1
    pose2.position = p2
    pose2.orientation = q2

    return [pose1, pose2]

def main():
    rospy.init_node('manipulaibility_analysis')
    object_name = rospy.get_param('~object_marker', "shape2")
    server_name = rospy.get_param('~server_name', "/in_hand_path_server/in_hand_path")
    
    #set the ray caster to generate valid contact points
    pkg = rospkg.RosPack()
    object_path = pkg.get_path('shape_analysis') + '/shapes/'
    caster = pycaster.rayCaster.fromSTL(object_path + object_name + ".stl", scale=1.0)

    #get the bounding box of the object
    xsource = [-1000, 0, 0]
    xdest = [1000, 0, 0]
    ysource = [0, -1000, 0]
    ydest = [0, 1000, 0]
    zsource = [0, 0 ,-1000]
    zdest = [0, 0, 1000]

    xintersections = caster.castRay(xsource, xdest)
    yintersections = caster.castRay(ysource, ydest)
    zintersections = caster.castRay(zsource, zdest)

    delta = 10.0 #use this to get away from the border of 1 cm

    min_x = xintersections[0][0]
    max_x = xintersections[-1][0]
    min_y = yintersections[0][1]
    max_y = yintersections[-1][1]
    min_z = zintersections[0][2]
    max_z = zintersections[-1][2]

    min_x2 = min_x + delta
    max_x2 = max_x - delta
    min_y2 = min_y + delta
    max_y2 = max_y - delta
    min_z2 = min_z + delta
    max_z2 = max_z - delta

    #now generate many different poses by changing x y z inside the bounding box and roll pitch yaw between 0 and 360
    x_changes = 5
    y_changes = 5
    z_changes = 5
    angle_changes = 18 #degrees
    max_angle = 360.0

    #initialize the manipulability matrix
    matrix_size = y_changes*z_changes*angle_changes + x_changes*y_changes*angle_changes + z_changes*x_changes*angle_changes
    manipulability_matrix = np.eye(matrix_size)

    poses = dict()
    idx = 0

    x_step = (max_x2 - min_x2)/x_changes
    y_step = (max_y2 - min_y2)/y_changes
    z_step = (max_z2 - min_z2)/z_changes
    angle_step = max_angle/angle_changes

    #all this changes do not include the final limit, but maybe it is enough for statistics
    #variations on x (working, verified):
    for y in np.arange(min_y2, max_y2, y_step):
        for z in np.arange (min_z2, max_z2, z_step):
            for angle in np.arange(0, max_angle, angle_step):
                #insert the pose in the dictionary
                fing_poses = computeFingersPoses(min_x, max_x, y, y, z, z, 0, angle*math.pi/180.0, math.pi/2.0)
                poses[idx] = fing_poses
                idx = idx + 1

    #variations on y (working, verified):
    for x in np.arange(min_x2, max_x2, x_step):
        for z in np.arange (min_z2, max_z2, z_step):
            for angle in np.arange(0, max_angle, angle_step):
                #insert the pose in the dictionary
                fing_poses = computeFingersPoses(x, x, min_y, max_y, z, z, math.pi/2.0, angle*math.pi/180.0, 0)
                poses[idx] = fing_poses
                idx = idx + 1

    #variations on z (working, verified):
    for x in np.arange(min_x2, max_x2, x_step):
        for y in np.arange (min_y2, max_y2, y_step):
            for angle in np.arange(0, max_angle, angle_step):
                #insert the pose in the dictionary
                poses[idx] = computeFingersPoses(x, x, y, y, min_z, max_z, math.pi/2.0, 0, angle*math.pi/180.0)
                idx = idx + 1

    print "idx final: " + str(idx)
    print "matrix_size: " + str(matrix_size)

    j_max = 0
    #now loop through all the poses and combinations to get the values in the matrix
    for i in range(0, idx + 1):
        #the second range changes because the matrix is symmetric and not every pose combination has to be checked twice!
        for j in range(0, j_max):
            # print "i: " + str(i)
            # print "j: " + str(j)
            if not i == j:
                [pose1, pose2] = poses[i]
                [pose1d, pose2d] = poses[j]
                # print "poses[i]: " + str(poses[i])
                # print "poses[j]: " + str(poses[j])

                result = call_server(server_name, pose1, pose2, pose1d, pose2d)
                manipulability_matrix[i, j] = result
                manipulability_matrix[j, i] = result

        #increase the column limit per each row
        j_max = j_max + 1
        print "--- " + str((float(i)/float(idx))*100.0) + "%"
        # rospy.sleep(10.0)

    #now save the obtained matrix into a file
    file_path = pkg.get_path('shape_analysis') + '/matrices/'
    np.save(object_name + '_' + str(matrix_size) + '_matrix.npy', manipulability_matrix)
    np.savetxt(object_name + '_' + str(matrix_size) + '_matrix.txt', manipulability_matrix)

    #show the resulting rank
    print object_name + " manipulability matrix rank: " + str(np.linalg.matrix_rank(manipulability_matrix))



if __name__ == "__main__":
    main()
    