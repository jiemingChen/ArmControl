import PyKDL as kdl

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

robot = URDF.from_xml_file("/home/jieming/catkin_ws/src/panda_simulation/franka_description/robots/model.urdf")
tree = kdl_tree_from_urdf_model(robot)
#print tree.getNrOfSegments()  ###8


#kdl_kin = KDLKinematics(robot, "panda_link0", "panda_link8")
#q = [1, 1, 3.1415926/2, -0.5, 0, 0, 0]
#q = [0, 0, 0, 0, 0, 0, 0]
#pose = kdl_kin.forward(q)
#print pose

#J = kdl_kin.jacobian(q)
#print  J

chain = tree.getChain("panda_link0", "panda_link8")
#print chain.getNrOfJoints()  ###7

grav_vector = kdl.Vector(0, 0, -9.81)  # relative to kdl chain base link
dyn_kdl = kdl.ChainDynParam(chain, grav_vector)

jt_positions = kdl.JntArray(7)
jt_positions[0] = 0.0
jt_positions[1] = 1.0
jt_positions[2] = 0.3
jt_positions[3] = -0.3
grav_matrix = kdl.JntArray(7)
dyn_kdl.JntToGravity(jt_positions, grav_matrix)
print grav_matrix

gravity_compensating_jt_torques = [grav_matrix[i] for i in range(grav_matrix.rows())]

