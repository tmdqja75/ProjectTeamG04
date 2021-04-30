import rospy
from panda_robot import PandaArm

rospy.init_node('panda_demo')

r = PandaArm()

r.move_to_neutral()
r.move_to_joint_position([-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01]) # move robot to the specified pose
