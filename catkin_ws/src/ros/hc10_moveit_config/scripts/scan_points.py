#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def main():
    # Initialisation
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_path_node', anonymous=True)

    # Charger l'interface MoveGroup
    group_name = "hc10_arm"   # vérifie le nom exact dans ton SRDF
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Créer une liste de waypoints
    waypoints = []

    # Pose de départ
    start_pose = move_group.get_current_pose().pose
    waypoints.append(start_pose)

    # Centre et offset
    center = geometry_msgs.msg.Point(0.2, 0.0, 0.2)  
    offset = 0.1                                    

    # Orientation = orientation de départ (fixée pour les 4 points)
    orientation = start_pose.orientation

    # 4 points autour du centre
    points = [
        geometry_msgs.msg.Point(center.x + offset, center.y + offset, center.z),
        geometry_msgs.msg.Point(center.x - offset, center.y + offset, center.z),
        geometry_msgs.msg.Point(center.x, center.y + offset, center.z + offset),
        geometry_msgs.msg.Point(center.x, center.y + offset, center.z - offset),
    ]

    for p in points:
        wpose = geometry_msgs.msg.Pose()
        wpose.position = p
        wpose.orientation = orientation
        waypoints.append(wpose)

    # Générer la trajectoire cartésienne
    (plan, fraction) = move_group.compute_cartesian_path(
                        waypoints,   # liste de poses
                        0.01,        # résolution en mètre
                        0.0)         # saut max

    rospy.loginfo("Fraction de la trajectoire planifiée: %f", fraction)

    # Exécuter le plan si assez bon
    if fraction > 0.8:
        move_group.execute(plan, wait=True)
        rospy.loginfo("Trajectoire exécutée avec succès !")
    else:
        rospy.logwarn("Trajectoire incomplète (fraction=%f)", fraction)

if __name__ == '__main__':
    main()

