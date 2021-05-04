import rospy
# import threading
import quaternion
import numpy as np
import matplotlib.pyplot as plt

from interactive_markers.interactive_marker_server import *
from panda_robot import PandaArm

'''
Impedance Control:
Initial State is the neutral position of the panda robot
'''
'''
Parameters that can be controlled:
K_p, K_o (Stiffness Coefficient [Position/Orientation])
D_p, D_o (Damping Coefficient [Position/Orientation])

1st demo
K_p, K_o = 100, 100
D_p, D_o = 5, 1
2nd demo
K_p, K_o = 500, 100
D_p, D_o = 5, 1
3rd demo
K_p, K_o = 100, 100
D_p, D_o = 20, 1
4th demo
K_p, K_o = 500, 100
D_p, D_o = 20, 1
'''
# define compliance parameters: translational_stiffness and rotational_stiffness
K_p, K_o = 500, 100
D_p, D_o = 20, 1

# Get the initial position of the robot when the code is first runned
def set_equilibrium(robot):
    end_effector_config = robot.endpoint_pose()
    initial_pos, initial_ori = end_effector_config['position'],end_effector_config['orientation']
    return initial_pos, initial_ori
    

def impedance_control(rate):
    # Variable for reaction force data
    global F_TOT
    while not rospy.is_shutdown():
        # Get deviated state off the robot (position and orientation)
        p, q = panda.ee_pose()         # Current position and orientation
        v, omega = panda.ee_velocity() # Current velocity and angular velocity
        
        # Reshape vector for calculation
        v = v.reshape([3,1])
        omega = omega.reshape([3,1])
        
        # Calculate difference between initial pose and current pose (position)
        dp = (p_i - p).reshape([3,1])
        
        # Calcualte difference between initial pose and current pose (orientation)
        # Convert quaternion into rotation matrix
        mat_c = quaternion.as_rotation_matrix(q) # current orientation
        mat_i = quaternion.as_rotation_matrix(q_i) # initial orientation
        mat_diff = mat_i.T.dot(mat_c)
        quat_diff = quaternion.from_rotation_matrix(mat_diff)
        vector_q = quaternion.as_float_array(quat_diff)[1:]
        if quat_diff.w < 0:
            vector_q = -vector_q

        dq = -mat_i.dot(vector_q).reshape([3,1])
        
        # compute reaction force
        stiff_diff = np.vstack([K_p*(dp), K_o*(dq)])
        damp_diff = np.vstack([D_p*(v), D_o*(omega)])
        F = stiff_diff - damp_diff
        
        # Store reaction force data to F_TOT
        F_cartesian = F[:3].reshape([1,3])
        t = rospy.get_time()
        F_cartesian = np.insert(F_cartesian, 0, t)
        F_TOT = np.vstack((F_TOT, F_cartesian))
        
        # From reaction force to Joint Torque
        J = panda.zero_jacobian()
        trq = np.dot(J.T,F)
        
        # Execute calculated torque to the robot
        panda.exec_torque_cmd(trq)
        rate.sleep()

# Store reaction force for plotting when the code is terminated
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
    plt.title('Reaction Force Plot with $K={}$ and $D = {}$'.format(K_p,D_p))
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.savefig('K{}D{}'.format(K_p,D_p))
    plt.show()
    print('Exporting csv')
    np.savetxt("Cartesion_Reactive_Force_K{}D{}.csv".format(K_p, D_p), F_TOT, delimiter=",")
    print('Export Done')
    

if __name__ == "__main__":
    rospy.init_node('panda_imped_control')

    panda = PandaArm()
    
    #Move Panda to neutral position
    panda.move_to_neutral()
    
    # Get initial state of the robot
    p_i, q_i = set_equilibrium(panda)
    rospy.on_shutdown(_on_shutdown)
    publish_rate = 100
    rate = rospy.Rate(publish_rate)
    impedance_control(rate)