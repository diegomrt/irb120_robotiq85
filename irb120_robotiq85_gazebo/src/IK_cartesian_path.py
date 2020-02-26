#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Inicializacion
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moving_panda_robot', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("panda_arm")
hand_group = moveit_commander.MoveGroupCommander("hand")

# Para publicar trayectorias en RViz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# Funcion para mover eje a eje un robot de 7 ejes
def move_joint_arm(joint_0,joint_1,joint_2,joint_3,joint_4,joint_5,joint_6):
    joint_goal = arm_group.get_current_joint_values()
    joint_goal[0] = joint_0
    joint_goal[1] = joint_1
    joint_goal[2] = joint_2
    joint_goal[3] = joint_3
    joint_goal[4] = joint_4
    joint_goal[5] = joint_5
    joint_goal[6] = joint_6

    arm_group.go(joint_goal, wait=True)
    arm_group.stop() # Garantiza que no hay movimiento residual

# Moviento a una pose "home" antes de empezar la trayectoria
rospy.loginfo("Moving arm to pose Home")	
move_joint_arm(0,-0.3,0,-1.4,0,1,pi/4)
rospy.sleep(3)


# Creacion de los puntos de la trayectoria
waypoints = []

wpose = arm_group.get_current_pose().pose # Tomamos la pose actual
wpose.position.z -= 0.3  # First move up (z), relative units
wpose.position.y += 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose)) # Se usa deepcopy para 

# Posicion 2, lateral (absoluta)
wpose.position.x = 0
wpose.position.y = 0.7
wpose.position.z = 0.5  
waypoints.append(copy.deepcopy(wpose))

# Posicion 3, casi sobre el eje del robot (absoluta)
wpose.position.x = 0.2
wpose.position.y = 0
wpose.position.z = 0.8
waypoints.append(copy.deepcopy(wpose))

# Posicion 4, lateral opuesto (absoluta)
wpose.position.x = 0
wpose.position.y = -0.7
wpose.position.z = 0.5  
waypoints.append(copy.deepcopy(wpose))

# Posicion 5, frontal (absoluta)
wpose.position.x = 0.7
wpose.position.y = 0
wpose.position.z = 0.5  
waypoints.append(copy.deepcopy(wpose))


(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
rospy.loginfo("Cartesian path planned")	

# Visualizacion en RVIZ de lo calculado
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory);

# Ejecucion de la trayectoria completa
arm_group.execute(plan, wait=True)
rospy.loginfo("Cartesian path finished. Shutting down")	
moveit_commander.roscpp_shutdown()
