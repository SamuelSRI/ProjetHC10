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
    group_name = "hc10_arm"  
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Créer une liste de waypoints
    waypoints = []

    # Point de départ = position courante
    start_pose = move_group.get_current_pose().pose
    waypoints.append(start_pose)

    # Déplacement en x
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation = start_pose.orientation
    wpose.position.x = start_pose.position.x + 0.2
    wpose.position.y = start_pose.position.y
    wpose.position.z = start_pose.position.z
    waypoints.append(wpose)

    # Déplacement en z
    wpose.position.z += 0.2
    waypoints.append(wpose)

    # Générer la trajectoire cartésienne
    (plan, fraction) = move_group.compute_cartesian_path(
                        waypoints,   # liste de poses
                        0.01,        # résolution en mètre
                        0.0)         # saut max

    rospy.loginfo("Fraction de la trajectoire planifiée: %f", fraction)

    # Exécuter le plan
    move_group.execute(plan, wait=True)

    rospy.loginfo("Trajectoire cartésienne exécutée avec succès!")

if __name__ == '__main__':
    main()
