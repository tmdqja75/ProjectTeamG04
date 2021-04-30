import rospy
import threading
import quaternion
import numpy as np

from interactive_markers.interactive_marker_server import *
from panda_robot import PandaArm

'''
Control Interface: The equilibrium is the initial configuration. After starting the controller try to push the robot around and try different stiffness levels.
1. Set goal pose as global variable (position and orientation)
2. Get current position of the robot
3. 
'''
'''
Parameters that can be controlled:
K_pos, K_ori
D_pos, D_ori
error

1st demo
K_pos, K_ori = 150, 50
D_pos, D_ori = 1, 1
2nd demo
K_pos, K_ori = 150, 50
D_pos, D_ori = 2, 2
3rd demo
K_pos, K_ori = 200, 100
D_pos, D_ori = 1, 1

'''
# define compliance parameters: translational_stiffness and rotational_stiffness
K_pos, K_ori = 200, 100
D_pos, D_ori = 1, 1

# Get the initial position of the robot when the code is first runned
def set_equilibrium(robot):
    end_effector_config = robot.endpoint_pose()
    initial_pos, initial_ori = end_effector_config['position'],end_effector_config['orientation']
    return initial_pos, initial_ori

def cal_difference(curr_ori, init_ori):
    curr_mat = quaternion.as_rotation_matrix(curr_ori)
    init_mat = quaternion.as_rotation_matrix(init_ori)
    rel_mat = init_mat.T.dot(curr_mat)
    rel_quat = quaternion.from_rotation_matrix(rel_mat)
    vec = quaternion.as_float_array(rel_quat)[1:]
    if rel_quat.w < 0.0:
        vec = -vec       
    return -init_mat.dot(vec)

def impedance_control(rate):
    while not rospy.is_shutdown():
        error = 100
        while error > 0.005:
            curr_pos, curr_ori = panda.ee_pose()
            curr_vel, curr_omg = panda.ee_velocity()
            delta_pos = (init_pos - curr_pos).reshape([3,1])
            delta_ori = cal_difference(curr_ori, init_ori).reshape([3,1])
            curr_vel = curr_vel.reshape([3,1])
            curr_omg = curr_omg.reshape([3,1])
            # compute control
            F = np.vstack([K_pos*(delta_pos), K_ori*(delta_ori)]) - np.vstack([D_pos*(curr_vel), D_ori*(curr_omg)])
            error = np.linalg.norm(delta_pos) + np.linalg.norm(delta_ori)
            J = panda.zero_jacobian()
            trq = np.dot(J.T,F) 
            panda.exec_torque_cmd(trq)
            rate.sleep()

def _on_shutdown():
    global thread
    if thread.is_alive():
        thread.join()

def set_ee_target(action, value):
        pos, ori = panda.ee_pose()
        if action == 'position':
            pos += value
        status, j_des = panda.inverse_kinematics(pos, ori)
        if status:
            panda.move_to_joint_position(j_des)


if __name__ == "__main__":
    rospy.init_node('panda_imped_control')

    panda = PandaArm()
    panda.move_to_neutral()
    q1
    init_pos, init_ori = set_equilibrium(panda)
    rospy.on_shutdown(_on_shutdown)
    publish_rate = 100
    rate = rospy.Rate(publish_rate)
    thread = threading.Thread(target=impedance_control, args = [rate])
    thread.start()


