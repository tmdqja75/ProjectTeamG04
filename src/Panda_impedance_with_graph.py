import rospy
# import threading
import quaternion
import numpy as np
import matplotlib.pyplot as plt

from interactive_markers.interactive_marker_server import *
from panda_robot import PandaArm

'''
Control Interface: The equilibrium is the initial configuration. After starting the controller 
try to push the robot around and try different stiffness levels.
'''
'''
Parameters that can be controlled:
K_pos, K_ori
D_pos, D_ori

1st demo
K_pos, K_ori = 100, 100
D_pos, D_ori = 5, 1
2nd demo
K_pos, K_ori = 500, 100
D_pos, D_ori = 5, 1
3rd demo
K_pos, K_ori = 100, 100
D_pos, D_ori = 20, 1
4th demo
K_pos, K_ori = 500, 100
D_pos, D_ori = 20, 1
'''
# define compliance parameters: translational_stiffness and rotational_stiffness
K_pos, K_ori = 500, 100
D_pos, D_ori = 20, 1

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
    global F_TOT
    while not rospy.is_shutdown():
        # error = 100
        # while error > 0.005:
        curr_pos, curr_ori = panda.ee_pose()
        curr_vel, curr_omg = panda.ee_velocity()
        curr_vel = curr_vel.reshape([3,1])
        curr_omg = curr_omg.reshape([3,1])
        delta_pos = (init_pos - curr_pos).reshape([3,1])
        delta_ori = cal_difference(curr_ori, init_ori).reshape([3,1])
        t = rospy.get_time()
        
        # compute control
        F = np.vstack([K_pos*(delta_pos), K_ori*(delta_ori)]) - np.vstack([D_pos*(curr_vel), D_ori*(curr_omg)])
        F_cart = F[:3].reshape([1,3])
        F_cart = np.insert(F_cart, 0, t)
        F_TOT = np.vstack((F_TOT, F_cart))
        
        error = np.linalg.norm(delta_pos) + np.linalg.norm(delta_ori)
        J = panda.zero_jacobian()
        trq = np.dot(J.T,F)
        
        # Execute calculated torque to the robot
        panda.exec_torque_cmd(trq)
        rate.sleep()

F_TOT = np.array([0,0,0,0])
def _on_shutdown():
    # Plot the reaction Force of the end effector during impedance experiment
    print('Shutdown')
    global F_TOT
    F_TOT = np.delete(F_TOT, (0), axis=0)
    t = F_TOT[:,0]
    t -= t[0]
    plt.plot(t,F_TOT[:,1], label = '$F_x$')
    plt.plot(t,F_TOT[:,2], label = '$F_y$')
    plt.plot(t,F_TOT[:,3], label = '$F_z$')
    plt.legend()
    plt.title('Reaction Force Plot with $K={}$ and $D = {}$'.format(K_pos,D_pos))
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.savefig('K{}D{}'.format(K_pos,D_pos))
    plt.show()
    print('Exporting csv')
    np.savetxt("Cartesion_Reactive_Force_K{}D{}.csv".format(K_pos, D_pos), F_TOT, delimiter=",")
    print('Export Done')
    

if __name__ == "__main__":
    rospy.init_node('panda_imped_control')

    panda = PandaArm()
    panda.move_to_neutral()
    
    init_pos, init_ori = set_equilibrium(panda)
    rospy.on_shutdown(_on_shutdown)
    publish_rate = 100
    rate = rospy.Rate(publish_rate)
    impedance_control(rate)