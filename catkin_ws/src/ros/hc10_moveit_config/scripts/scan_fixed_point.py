#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Point
import moveit_commander
from tf.transformations import quaternion_about_axis

def quat_from_two_vectors(a, b):
    """Retourne un quaternion qui aligne le vecteur a sur b"""
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    dot = np.clip(np.dot(a, b), -1.0, 1.0)
    if dot > 0.999999:
        return np.array([0, 0, 0, 1])
    if dot < -0.999999:
        ortho = np.array([1, 0, 0])
        if abs(a[0]) > 0.9:
            ortho = np.array([0, 1, 0])
        axis = np.cross(a, ortho)
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        q = quaternion_about_axis(math.pi, axis)
        return np.array([q[0], q[1], q[2], q[3]])
    axis = np.cross(a, b)
    axis /= (np.linalg.norm(axis) + 1e-12)
    angle = math.acos(dot)
    q = quaternion_about_axis(angle, axis)
    return np.array([q[0], q[1], q[2], q[3]])

def look_at_quat(from_point, to_point, tool_axis='+Z'):
    """Calcule l’orientation qui fait pointer l’axe du tool vers le centre"""
    dir_vec = np.array([to_point.x - from_point.x,
                        to_point.y - from_point.y,
                        to_point.z - from_point.z])
    dir_vec /= (np.linalg.norm(dir_vec) + 1e-12)

    axes = {
        '+X': np.array([1,0,0]), '-X': np.array([-1,0,0]),
        '+Y': np.array([0,1,0]), '-Y': np.array([0,-1,0]),
        '+Z': np.array([0,0,1]), '-Z': np.array([0,0,-1]),
    }
    tool = axes[tool_axis]
    q = quat_from_two_vectors(tool, dir_vec)
    return q

def main():
    rospy.init_node("scan_fixed_point")

    # --- PARAMS ---
    group_name = "hc10_arm"   # nom du planning group MoveIt
    base_frame = "base_link"
    tool_axis = "+Z"          # axe optique de la caméra
    center = Point(0.7, 0.0, 0.8)  # position fixe de l’objet (x,y,z en m)
    radius = 0.5
    z_offset = 0.3
    n_points = 36
    step = 0.01
    # -------------

    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_pose_reference_frame(base_frame)

    waypoints = []

    # Pose de départ
    start_pose = group.get_current_pose().pose
    waypoints.append(start_pose)

    # Génération cercle
    for i in range(n_points):
        theta = 2*math.pi * i / n_points
        px = center.x + radius*math.cos(theta)
        py = center.y + radius*math.sin(theta)
        pz = center.z + z_offset

        q = look_at_quat(Point(px,py,pz), center, tool_axis)

        pose = Pose()
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        waypoints.append(pose)

    (plan, fraction) = group.compute_cartesian_path(waypoints, step, 0.0)
    rospy.loginfo("Fraction planifiée: %.2f", fraction)

    if fraction > 0.9:
        group.execute(plan, wait=True)
        rospy.loginfo("Trajectoire exécutée")
    else:
        rospy.logwarn("Trajectoire incomplète, fraction=%.2f", fraction)

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
