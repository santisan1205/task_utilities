#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def move_left_arm():
    # Inicializar moveit_commander y el nodo ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_left_arm', anonymous=True)

    # Crear un objeto MoveGroupCommander para el brazo izquierdo.
    # Asegúrate de que "left_arm" coincide con el nombre de tu grupo de planificación.
    group_name = "left_arm"  
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Obtener la pose actual del brazo
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo("Pose actual: %s", current_pose)

    # Crear una nueva pose desplazada (por ejemplo, 5 cm en la dirección X)
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = current_pose.position.x + 0.1
    target_pose.position.y = current_pose.position.y
    target_pose.position.z = current_pose.position.z
    target_pose.orientation = current_pose.orientation  # Mantenemos la orientación actual

    # Establecer la nueva pose objetivo
    move_group.set_pose_target(target_pose)

    # Planificar y ejecutar el movimiento
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.loginfo("Movimiento completado.")

    # Cerrar la comunicación con MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_left_arm()
    except rospy.ROSInterruptException:
        pass
