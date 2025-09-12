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
    """Quaternion qui aligne le vecteur a sur b"""
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    dot = np.clip(np.dot(a, b), -1.0, 1.0)
    if dot > 0.999999:
        return np.array([0,0,0,1])
    if dot < -0.999999:
        ortho = np.array([1,0,0])
        if abs(a[0]) > 0.9:
            ortho = np.array([0,1,0])
        axis = np.cross(a, ortho)
        axis /= (np.linalg.norm(axis)+1e-12)
        q = quaternion_about_axis(math.pi, axis)
        return np.array([q[0], q[1], q[2], q[3]])
    axis = np.cross(a, b)
    axis /= (np.linalg.norm(axis)+1e-12)
    angle = math.acos(dot)
    q = quaternion_about_axis(angle, axis)
    return np.array([q[0], q[1], q[2], q[3]])

def look_at_quat(from_point, to_point, tool_axis='+Z'):
    """Orientation qui pointe l’axe du tool vers le centre"""
    dir_vec = np.array([to_point.x - from_point.x,
                        to_point.y - from_point.y,
                        to_point.z - from_point.z])
    dir_vec /= (np.linalg.norm(dir_vec)+1e-12)
    axes = {
        '+X': np.array([1,0,0]), '-X': np.array([-1,0,0]),
        '+Y': np.array([0,1,0]), '-Y': np.array([0,-1,0]),
        '+Z': np.array([0,0,1]), '-Z': np.array([0,0,-1]),
    }
    tool = axes[tool_axis]
    return quat_from_two_vectors(tool, dir_vec)

def main():
    rospy.init_node("circle_near_base")

    # --- Paramètres ---
    group_name = "hc10_arm"       # Vérifie le nom de ton Planning Group
    base_frame = "base_link"
    tool_axis = "+Z"              # Axe optique fictif
    center = Point(0.5, 0.0, 0.5) # Point près de la base (x=0.5m, z=0.5m)
    radius = 0.3                  # Rayon du cercle (30 cm)
    z_offset = 0.0                # Décalage en hauteur
    n_points = 24                 # Nombre de waypoints
    step = 0.01                   # Résolution cartésienne
    # ------------------

    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_pose_reference_frame(base_frame)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)

    waypoints = []

    # Ajoute la pose courante comme départ
    start_pose = group.get_current_pose().pose
    waypoints.append(start_pose)

    # Génération cercle
    for i in range(n_points+1):
        theta = 2*math.pi*i/n_points
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
    rospy.loginfo("Fraction calculée: %.2f", fraction)

    if fraction > 0.9:
        group.execute(plan, wait=True)
        rospy.loginfo("Trajectoire exécutée")
    else:
        rospy.logwarn("Trajectoire incomplète (%.2f)", fraction)

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
