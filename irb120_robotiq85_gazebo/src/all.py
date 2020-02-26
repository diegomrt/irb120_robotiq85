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

# Funcion para mover la herramienta
def move_joint_hand(joint):
    joint_goal = hand_group.get_current_joint_values()
    joint_goal[0] = joint
    joint_goal[1] = joint

    hand_group.go(joint_goal, wait=True)
    hand_group.stop() # Garantiza que no hay movimiento residual

# Funcion para mover el TCP a una posicion y orientacion determinada
def move_pose_arm(roll,pitch,yaw,x,y,z):
    pose_goal = geometry_msgs.msg.Pose()
    quat = quaternion_from_euler(roll,pitch,yaw)
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm_group.set_pose_target(pose_goal)

    plan = arm_group.go(wait=True)

    arm_group.stop()
    arm_group.clear_pose_targets()

if __name__ == '__main__':
    
    # Estado del robot
    print "============ Printing robot state ============"
    print robot.get_current_state()
    print ""  

    # Movimientos eje a eje, repetidos
    for i in range(0):	
	rospy.loginfo("Moving arm to pose_1")	
	move_joint_arm(-pi/2,1,0.2,-1,0,2,0.1)
        rospy.sleep(3)
	rospy.loginfo("Moving arm to pose_2")
	move_joint_arm(pi/2,0,0.2,-pi/2,0,3,1)
        rospy.sleep(3)
    
    # Movimientos del TCP punto a punto, repetidos
    for i in range(6):	
	rospy.loginfo("Moving arm to point_1")	
	move_pose_arm(pi,0,0,0.5,-0.25,0.3)
        rospy.sleep(2)
        rospy.loginfo("Closing hand")
        move_joint_hand(0)
        rospy.sleep(2)
	rospy.loginfo("Moving arm to point_2")
	move_pose_arm(pi,0,0,0.5,0.25,0.3)
        rospy.sleep(2)
        rospy.loginfo("Openning hand")
        move_joint_hand(0.03)
        rospy.sleep(2)
	rospy.loginfo("Moving arm to point_3")	
	move_pose_arm(0,0,0,0.5,0.25,0.9)
        rospy.sleep(2)
	rospy.loginfo("Moving arm to point_4")
	move_pose_arm(0,0,pi,0.5,-0.25,0.9)
        rospy.sleep(2)

    rospy.loginfo("All movements finished. Shutting down")	
    moveit_commander.roscpp_shutdown()
