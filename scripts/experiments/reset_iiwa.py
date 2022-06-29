#!/usr/bin/env python
# encoding: utf-8
#|
#|    Copyright Inria July 2017
#|    This project has received funding from the European Research Council (ERC) under
#|    the European Union's Horizon 2020 research and innovation programme (grant
#|    agreement No 637972) - see http://www.resibots.eu
#|
#|    Contributor(s):
#|      - Matthias Mayr (matthias.mayr@cs.lth.se)
#|      - Konstantinos Chatzilygeroudis (konstantinos.chatzilygeroudis@inria.fr)
#|      - Rituraj Kaushik (rituraj.kaushik@inria.fr)
#|      - Roberto Rama (bertoski@gmail.com)
#|
#|
#|    This software is governed by the CeCILL-C license under French law and
#|    abiding by the rules of distribution of free software.  You can  use,
#|    modify and/ or redistribute the software under the terms of the CeCILL-C
#|    license as circulated by CEA, CNRS and INRIA at the following URL
#|    "http://www.cecill.info".
#|
#|    As a counterpart to the access to the source code and  rights to copy,
#|    modify and redistribute granted by the license, users are provided only
#|    with a limited warranty  and the software's author,  the holder of the
#|    economic rights,  and the successive licensors  have only  limited
#|    liability.
#|
#|    In this respect, the user's attention is drawn to the risks associated
#|    with loading,  using,  modifying and/or developing or reproducing the
#|    software by the user in light of its specific status of free software,
#|    that may mean  that it is complicated to manipulate,  and  that  also
#|    therefore means  that it is reserved for developers  and  experienced
#|    professionals having in-depth computer knowledge. Users are therefore
#|    encouraged to load and test the software's suitability as regards their
#|    requirements in conditions enabling the security of their systems and/or
#|    data to be ensured and,  more generally, to use and operate it in the
#|    same conditions as regards security.
#|
#|    The fact that you are presently reading this means that you have had
#|    knowledge of the CeCILL-C license and that you accept its terms.
#|
import rospy
from time import sleep
import numpy as np
import sys

import actionlib
from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped ,WrenchStamped
from cartesian_impedance_controller.msg import ControllerConfig
from iiwa_tools.srv import GetFK, GetFKRequest, GetFKResponse
from std_msgs.msg import MultiArrayDimension
from cartesian_trajectory_generator.msg import TrajectoryAction, TrajectoryGoal

configs = dict()
configs["obstacle1_1"] = [-0.03, 0.64, -0.12, -1.27, 0.96, 1.80, -1.08]
configs["obstacle2_1"] = [-0.03, 0.64, -0.12, -1.27, 0.96, 1.80, -1.08]
configs["obstacle3_1"] = [-0.03, 0.64, -0.12, -1.27, 0.96, 1.80, -1.08]
configs["obstacle4_1"] = [-0.03, 0.64, -0.12, -1.27, 0.96, 1.80, -1.08]
configs["obstacle5_1"] = [-0.03, 0.64, -0.12, -1.27, 0.96, 1.80, -1.08]

configs["obstacle1_2"] = [0.08, 1.11, -0.12, -0.78, 0.97, 1.45, -1.08]
configs["obstacle2_2"] = [0.08, 1.11, -0.12, -0.78, 0.97, 1.45, -1.08]
configs["obstacle3_2"] = [0.08, 1.11, -0.12, -0.78, 0.97, 1.45, -1.08]
configs["obstacle4_2"] = [0.08, 1.11, -0.12, -0.78, 0.97, 1.45, -1.08]
configs["obstacle5_2"] = [0.08, 1.11, -0.12, -0.78, 0.97, 1.45, -1.08]

configs["obstacle1_3"] = [0.47, 1.54, -0.30, -0.84, 1.48, 1.34, -1.28]
configs["obstacle2_3"] = [0.43, 1.65, -0.32, -0.56, 1.46, 1.37, -1.11]
configs["obstacle3_3"] = [0.44, 1.58, -0.30, -0.90, 1.49, 1.34, -1.38]
configs["obstacle4_3"] = [0.49, 1.45, -0.28, -1.10, 1.50, 1.32, -1.45]
configs["obstacle5_3"] = [0.48, 1.52, -0.30, -0.74, 1.46, 1.35, -1.15]

configs["peg1_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg2_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg3_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg4_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg5_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg6_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg7_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg8_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg9_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68] 
configs["peg10_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg11_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg12_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg13_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg14_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]
configs["peg15_1"] = [1.10, 0.41, -0.42, -1.52, 1.30, 1.14, -0.68]

configs["peg1_2"] = [0.67, 0.27, -0.27, -1.40, 1.11, 1.49, -0.58]
configs["peg2_2"] = [0.70, 0.21, -0.27, -1.46, 1.09, 1.47, -0.56]
configs["peg3_2"] = [0.69, 0.25, -0.27, -1.39, 1.10, 1.49, -0.54]
configs["peg4_2"] = [0.52, 0.30, -0.15, -1.48, 1.10, 1.46, -0.69]
configs["peg5_2"] = [0.52, 0.26, -0.15, -1.55, 1.09, 1.45, -0.72]
configs["peg6_2"] = [0.65, 0.22, -0.27, -1.54, 1.01, 1.47, -0.67]
configs["peg7_2"] = [0.65, 0.30, -0.27, -1.40, 1.11, 1.50, -0.61]
configs["peg8_2"] = [0.70, 0.165, -0.27, -1.53, 1.08, 1.45, -0.59]
configs["peg9_2"] = [0.57, 0.09, -0.15, -1.72, 1.07, 1.41, -0.69]
configs["peg10_2"] = [0.50, 0.17, -0.15, -1.72, 1.09, 1.42, -0.80]
configs["peg11_2"] = [0.51, 0.21, -0.15, -1.65, 1.09, 1.43, -0.77]
configs["peg12_2"] = [0.57, 0.13, -0.15, -1.65, 1.07, 1.42, -0.66]
configs["peg13_2"] = [0.67, 0.23, -0.27, -1.47, 1.10, 1.48, -0.60]
configs["peg14_2"] = [0.67, 0.19, -0.27, -1.54, 1.09, 1.46, -0.63]
configs["peg15_2"] = [0.65, 0.26, -0.27, -1.47, 1.11, 1.48, -0.64]

configs["push1_1"] = [0.33, 0.89, -0.16, -1.39, 1.19, 1.39, -1.23]
configs["push2_1"] = [0.29, 0.85, -0.14, -1.87, 1.25, 1.25, -1.66]
configs["push3_1"] = [0.31, 1.15, -0.08, -1.63, 1.34, 1.17, -1.71]
configs["push4_1"] = [0.27, 1.34, -0.13, -1.47, 1.34, 1.23, -1.31]
configs["push5_1"] = [0.24, 0.96, -0.04, -1.49, 1.23, 1.18, -0.98]
configs["push6_1"] = [0.17, 1.50, -0.02, -0.61, 0.99, 1.47, -0.66]
positions = None

def callback_set_stiffness(data):
    global positions
    positions = data.position[0:7]

def set_stiffness(peg_joint_values, flag, change_factor):
    msg = ControllerConfig()

    msg.cartesian_stiffness.force.x = 400.0 / change_factor
    msg.cartesian_stiffness.force.y = 400.0 / change_factor
    msg.cartesian_stiffness.force.z = 400.0 / change_factor
    msg.cartesian_stiffness.torque.x = 50.0 / change_factor
    msg.cartesian_stiffness.torque.y = 50.0 / change_factor
    msg.cartesian_stiffness.torque.z = 50.0 / change_factor

    # Negative values mean that the default damping values apply --> 2* sqrt(stiffness)
    msg.cartesian_damping.force.x =  -1.0
    msg.cartesian_damping.force.y =  -1.0
    msg.cartesian_damping.force.z =  -1.0
    msg.cartesian_damping.torque.x = -1.0
    msg.cartesian_damping.torque.y = -1.0
    msg.cartesian_damping.torque.z = -1.0

    # msg.q_d_nullspace.q1 = peg_joint_values[0]
    # msg.q_d_nullspace.q2 = peg_joint_values[1]
    # msg.q_d_nullspace.q3 = peg_joint_values[2]
    # msg.q_d_nullspace.q4 = peg_joint_values[3]
    # msg.q_d_nullspace.q5 = peg_joint_values[4]
    # msg.q_d_nullspace.q6 = peg_joint_values[5]
    # msg.q_d_nullspace.q7 = peg_joint_values[6]

    msg.q_d_nullspace.append(peg_joint_values[0])
    msg.q_d_nullspace.append(peg_joint_values[1])
    msg.q_d_nullspace.append(peg_joint_values[2])
    msg.q_d_nullspace.append(peg_joint_values[3])
    msg.q_d_nullspace.append(peg_joint_values[4])
    msg.q_d_nullspace.append(peg_joint_values[5])
    msg.q_d_nullspace.append(peg_joint_values[6])

    if flag:
        msg.nullspace_stiffness = 5.0 / change_factor
    else:
        msg.nullspace_stiffness = 0.0

    return msg

def set_wrench():
    msg = WrenchStamped()

    msg.wrench.force.x = 0.0
    msg.wrench.force.y = 0.0
    msg.wrench.force.z = 0.0
    msg.wrench.torque.x = 0.0
    msg.wrench.torque.y = 0.0
    msg.wrench.torque.z = 0.0

    return msg

def set_target_pose(res):

    msg = PoseStamped()
    msg.header.frame_id = 'world'
    msg.pose.position.x = res.poses[0].position.x
    msg.pose.position.y = res.poses[0].position.y
    msg.pose.position.z = res.poses[0].position.z
    msg.pose.orientation.x = res.poses[0].orientation.x
    msg.pose.orientation.y = res.poses[0].orientation.y
    msg.pose.orientation.z = res.poses[0].orientation.z
    msg.pose.orientation.w = res.poses[0].orientation.w

    return msg

def set_fk():    
    req = GetFKRequest()
    mad = MultiArrayDimension()
    mad.size = 1
    req.joints.layout.dim.append(mad)
    mad2 = MultiArrayDimension()
    mad2.size = 7
    req.joints.layout.dim.append(mad2)

    return req

def callback(data):
    global positions
    positions = data.position[0:7]

def publisher(experiment):
    rospy.init_node('Reseter')
    pub_stiffness = rospy.Publisher('/bh/CartesianImpedance_trajectory_controller/set_config', ControllerConfig, queue_size=10)
    pub_wrench = rospy.Publisher('/bh/CartesianImpedance_trajectory_controller/set_cartesian_wrench', WrenchStamped, queue_size=10)
    sub = rospy.Subscriber("/bh/joint_states", JointState, callback)
    # pub_target_pose = rospy.Publisher('/bh/CartesianImpedance_trajectory_controller/goal', PoseStamped, queue_size=10)
    target_pose_ac = actionlib.SimpleActionClient('/bh/cartesian_trajectory_generator/goal_action', TrajectoryAction)
    target_pose_ac.wait_for_server()

    rospy.wait_for_service('/bh/service/iiwa_fk_server')
    srv_fk = rospy.ServiceProxy('/bh/service/iiwa_fk_server', GetFK)
    
    rospy.sleep(1)
    r = rospy.Rate(200)
    # stiffness_msg = set_stiffness(configs[experiment+"_"+ str(i)])
    # wrench_msg = set_wrench()

    # pub_stiffness.publish(stiffness_msg)
    # pub_wrench.publish(wrench_msg)
    req = set_fk()
    change_factor = [5,4,3,2,1]     # Divides the stiffness values by relevant factor
    start_k = 1
    if experiment[:-1] == "peg":
        req_cur = set_fk()
        req_cur.joints.data = np.array(positions)
        res_cur = srv_fk(req_cur)
        if res_cur.poses[0].position.z > 0.69:
            start_k = 2

    for k in range(start_k,4):
        print "--------------------"
        pos_name = experiment+"_"+ str(k)
        rospy.loginfo("Moving to %s: %s", pos_name, str(configs[pos_name]))
        req.joints.data = configs[pos_name]
        res = srv_fk(req)
        wrench_msg = set_wrench()
        pub_wrench.publish(wrench_msg)
        target_pose_msg = set_target_pose(res)
        # pub_target_pose.publish(target_pose_msg)
        trajectory_goal = TrajectoryGoal()
        trajectory_goal.pose = target_pose_msg
        target_pose_ac.send_goal(trajectory_goal)
        if k==start_k:
            for i in change_factor:
                stiffness_msg = set_stiffness(configs[experiment+"_"+ str(k)], True, i)
        pub_stiffness.publish(stiffness_msg)
        sleep(0.3)  # sleep before changing the value again
        req_cur = set_fk()
        req_des = set_fk()
        target_pose_ac.wait_for_result()
        ac_res = target_pose_ac.get_result()
        if ac_res.error_code is not 0:
            rospy.logerr("Trajectory generator aborted!")
        else:
            rospy.loginfo("Trajectory to %s finished.", pos_name)

        while not rospy.is_shutdown():
            # get cartesian pose 
            req_cur.joints.data = np.array(positions)
            res_cur = srv_fk(req_cur)
            
            req_des.joints.data = configs[pos_name]
            res_des = srv_fk(req_des)

            cur_pose = np.array([res_cur.poses[0].position.x, res_cur.poses[0].position.y, res_cur.poses[0].position.z,
                        res_cur.poses[0].orientation.x, res_cur.poses[0].orientation.y, res_cur.poses[0].orientation.z,
                        res_cur.poses[0].orientation.w])
            des_pose = np.array([res_des.poses[0].position.x, res_des.poses[0].position.y, res_des.poses[0].position.z,
                        res_des.poses[0].orientation.x, res_des.poses[0].orientation.y, res_des.poses[0].orientation.z,
                        res_des.poses[0].orientation.w])

            diff_pose = np.abs(cur_pose - des_pose)
            rospy.loginfo_throttle(1,"cur_pose: %s", str(cur_pose))
            rospy.loginfo_throttle(1,"des_pose: %s", str(des_pose))
            if all(x < 0.1 for x in diff_pose):
                rospy.loginfo("Reached %s", str(configs[experiment+"_"+ str(k)]))
                for i in change_factor:
                    stiffness_msg = set_stiffness(configs[experiment+"_"+ str(k)], False, i)
                pub_stiffness.publish(stiffness_msg)
                sleep(0.3)  # sleep before changing the value again
                rospy.sleep(0.1)
                break
        # Hack: Early abort for peg and push - should be re-implemented
        if experiment[:-1] == "push":
            break
        if experiment[:-1] == "peg" and k == 2:
            break
    # Wait for the trajectory generator to finish
    sleep(1)
    rospy.loginfo("End of reset script")
    exit()


if __name__ == '__main__':  
    if len(sys.argv) is not 2:
        print "Experiment 'peg' or 'peg?' or needs to be specified."
        exit()
    experiment = sys.argv[1]
    if experiment == "peg":
        print "Peg experiment - default configuration"
        experiment = "peg1"
    if experiment == "push":
        print "Push experiment - default configuration"
        experiment = "push1"
    if experiment == "obstacle":
        print "Obstacle experiment - default configuration"
        experiment = "obstacle1"

    print "Experiment: ", experiment
    first_key = experiment + "_1"
    if first_key in configs:
        desired_pos = configs[first_key]
    else:
        print "No configuration found for key", first_key
        print "Did you mispell the experiment? Exiting."
        exit()
    publisher(experiment)
