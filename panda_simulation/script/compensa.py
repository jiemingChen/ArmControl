import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

import rospy
import numpy as np
from math import sin, cos, sqrt
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from pdb import set_trace
model_file = "/home/jieming/catkin_ws/src/panda_simulation/franka_description/robots/model.urdf"
jt_positions = kdl.JntArray(7)
jt_velocities = kdl.JntArray(7)
flag=0

def callback(data):
	global flag
	if(flag==0):
		print('first callback')
		flag =1
	for i in range(7):
		jt_positions[i] = data.position[i]
		jt_velocities[i] = data.velocity[i]
def main():
	#ros configure
	rospy.init_node('inv_dyn', anonymous=True)
	rate = rospy.Rate(2000) # 1000hz
	rospy.Subscriber("/robot1/joint_states", JointState, callback)
	pub = rospy.Publisher('/robot1/panda_joint1_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/robot1/panda_joint2_controller/command', Float64, queue_size=10)
	pub3 = rospy.Publisher('/robot1/panda_joint3_controller/command', Float64, queue_size=10)
	pub4 = rospy.Publisher('/robot1/panda_joint4_controller/command', Float64, queue_size=10)
	pub5 = rospy.Publisher('/robot1/panda_joint5_controller/command', Float64, queue_size=10)
	pub6 = rospy.Publisher('/robot1/panda_joint6_controller/command', Float64, queue_size=10)
	pub7 = rospy.Publisher('/robot1/panda_joint7_controller/command', Float64, queue_size=10)

	#kdl configure
	robot = URDF.from_xml_file(model_file)
	tree = kdl_tree_from_urdf_model(robot)
	chain = tree.getChain("panda_link0", "panda_link8")
	grav_vector = kdl.Vector(0, 0, -9.8)

	dyn_kdl = kdl.ChainDynParam(chain, grav_vector)
	kdl_kin = KDLKinematics(robot, "panda_link0", "panda_link8")

	grav_matrix = kdl.JntArray(7)
	coriolis = kdl.JntArray(7)
	goal_pose = kdl.Vector(0.1, 0, 0.9)
	print('start')
	#main
	while not rospy.is_shutdown():
		rate.sleep()
		#if flag==0:
		#	continue

		jt_positions[0]=0; jt_positions[1]=-0.785; jt_positions[2] = 0;
		jt_positions[3]=-2.356; jt_positions[4]=0; jt_positions[5] = 1.57;
		jt_positions[6]=0.785

		dyn_kdl.JntToGravity(jt_positions, grav_matrix)
		dyn_kdl.JntToCoriolis(jt_positions, jt_velocities, coriolis)
		J = kdl_kin.jacobian(list(jt_positions))
		cur_pose = kdl_kin.forward(list(jt_positions))

		print(cur_pose)
		print(coriolis)
		print(grav_matrix)
		print(J)
		set_trace()
		pos_error = np.array([ [goal_pose[0]-cur_pose[0,3]],
				   [goal_pose[1]-cur_pose[1,3]],
				   [goal_pose[2]-cur_pose[2,3]],
                                   [0], [0], [0]])*100 
		vel_error = -2*(J.dot(list(jt_velocities))).reshape((6,1))
		error = pos_error + vel_error
		#error = kdl.Vector(goal_pose[0]-cur_pose[0,3], goal_pose[1]-cur_pose[1,3], goal_pose[2]-cur_pose[2,3], 0, 0, 0)*100
		tau_task = J.T.dot(error)

		#u = [grav_matrix[i]+coriolis[i]+tau_task[i] for i in range(grav_matrix.rows())]
		u = [grav_matrix[i]+coriolis[i] for i in range(grav_matrix.rows())]
		pub.publish( u[0])
		pub2.publish(u[1])
		pub3.publish(u[2])
		pub4.publish(u[3])
		pub5.publish(u[4])
		pub6.publish(u[5])
		pub7.publish(u[6])


main()


#print grav_matrix

#gravity_compensating_jt_torques = [grav_matrix[i] for i in range(grav_matrix.rows())]
#print gravity_compensating_jt_torques








#print tree.getNrOfSegments()  ###8

#print tree.getNrOfSegments()  ###8


#kdl_kin = KDLKinematics(robot, "panda_link0", "panda_link8")
#q = [1, 1, 3.1415926/2, -0.5, 0, 0, 0]
#q = [0, 0, 0, 0, 0, 0, 0]
#pose = kdl_kin.forward(q)
#print pose

#J = kdl_kin.jacobian(q)
#print  J
#kdl_kin = KDLKinematics(robot, "panda_link0", "panda_link8")
#q = [1, 1, 3.1415926/2, -0.5, 0, 0, 0]
#q = [0, 0, 0, 0, 0, 0, 0]
#pose = kdl_kin.forward(q)
#print pose

#J = kdl_kin.jacobian(q)
#print  J
