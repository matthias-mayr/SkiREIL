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
import sys
import signal
import rospy
import time

from geometry_msgs.msg import Pose
import tf

from skireil.srv import SkirosNextAction, SkirosNextActionRequest, SkirosNextActionResponse
from skireil.msg import ParamFloat
from skiros2_msgs.msg import SkillProgress

from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, NoFail, Selector, Sequential
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes, ParamHandler

class SkirosManualExecution():
    SkillStateDict = {SkillProgress.SUCCESS: 'success', SkillProgress.FAILURE: 'failure',
                      SkillProgress.RUNNING: 'running', SkillProgress.IDLE: 'idle'}

    def __init__(self, nr=1):
        # Change to /test_robot for integration test
        service_name = "skiros_worker_" + str(nr)
        self.episode_length = rospy.Duration.from_sec(15.0)
        rospy.wait_for_service(service_name)
        print "Connected to", service_name
        self.ac = rospy.ServiceProxy(service_name, SkirosNextAction, persistent=True)
        self.rate = rospy.Rate(10)
        self.listener = tf.TransformListener()
        self.get_ee_pose()

    def get_ee_pose(self):
        pose = Pose()
        trans = None
        while not trans:
            try:
                (trans, rot) = self.listener.lookupTransform('/world', '/bh_link_ee', rospy.Time(0))
                pose.position.x = trans[0]
                pose.position.y = trans[1]
                pose.position.z = trans[2]
                pose.orientation.x = rot[0]
                pose.orientation.y = rot[1]
                pose.orientation.z = rot[2]
                pose.orientation.w = rot[3]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                time.sleep(0.01)
                rospy.logwarn("Failed to get transform for ee_pose. Will retry forever.")
                continue
        return pose

    def param_obstacle_avoidance_exp(self, params):
        f_params = list()
        p = ParamFloat()
        p.value = params[0]
        p.key = "mp2_threshold"
        f_params.append(p)
        p = ParamFloat()
        p.value = params[1]
        p.key = "mp3_threshold"
        f_params.append(p)
        p = ParamFloat()
        p.value = params[2]
        p.key = "mp2_position_y"
        f_params.append(p)
        p = ParamFloat()
        p.value = params[3]
        p.key = "mp2_position_z"
        f_params.append(p)
        p = ParamFloat()
        p.value = params[4]
        p.key = "mp1_position_y"
        f_params.append(p)
        p = ParamFloat()
        p.value = params[5]
        p.key = "mp1_position_z"
        f_params.append(p)
        return f_params

    def param_json_skill_exp(self, params):
        f_params = list()
        p = ParamFloat()
        p.value = params[0]
        p.key = "PathVelocity"
        f_params.append(p)
        p = ParamFloat()
        p.value = params[1]
        p.key = "Radius"
        f_params.append(p)
        p = ParamFloat()
        p.value = params[2]
        p.key = "Force"
        f_params.append(p)
        return f_params

    def set_ee_pose(self, request_object, params, diff):
        request_object.ee_pose.position.x = params[0]
        request_object.ee_pose.position.y = params[1]
        request_object.ee_pose.position.z = params[2] - diff
        request_object.ee_pose.orientation.x = params[3]
        request_object.ee_pose.orientation.y = params[4]
        request_object.ee_pose.orientation.z = params[5]
        request_object.ee_pose.orientation.w = params[6]    
        return request_object.ee_pose

    def run(self):
        # initiate task

        # rospy returns 0 sometimes. For reference:
        # https://answers.ros.org/question/300956/rospytimenow-returns-0/
        while rospy.Time.now().to_sec() == 0.0:
            time.sleep(0.01)
        self.start = rospy.Time.now()
        # num of tasks
        num_tasks = 1
        j = 0
        total_ticks = 0
        while j < num_tasks:

            req = SkirosNextActionRequest()
            req.type = req.START_TASK
            req.sim_time = 0.0
            # print "Starting task"
            ## ee_poses
            params_ee_start = [-0.6, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0]
            params_ee_approach = [-0.6, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0]
            params_ee_end = [-0.6, 0.0, 0.7, 0.0, 0.0, 0.0, 1.0]
            # ## stiffness
            # params_stiff_start = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
            # params_stiff_approach = [200.0, 200.0, 200.0, -1.0, -1.0, -1.0]
            # params_stiff_end = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
            # ## force
            # params_force_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # params_force_approach = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
            # params_force_end = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # ## overlay_motion
            # params_overlay_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # params_overlay_approach = [1.0, 0.07, 0.05, 0.15, 0.0, 0.0, 0.0, 1.0]
            # params_overlay_end = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            print ("params_ee_start",params_ee_start)
            req.ee_pose = self.get_ee_pose()
            # req.ee_pose.position.x = params_ee_start[0]
            # req.ee_pose.position.y = params_ee_start[1]
            # req.ee_pose.position.z = params_ee_start[2]
            # req.ee_pose.orientation.w = params_ee_start[3]
            # req.ee_pose.orientation.x = params_ee_start[4]
            # req.ee_pose.orientation.y = params_ee_start[5]
            # req.ee_pose.orientation.z = params_ee_start[6]
            # req.ee_pose = self.set_ee_pose(req, params_ee_start)
            print ("req.ee_pose",req.ee_pose)

            req.skill_name = "json_skill"
            req.robot_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # params = [1.0, 0.35, -0.5, 1.15, 0.5, 1.1]
            if req.skill_name == "json_skill":
                params = [0.1234, 0.5555, 2.222]
                req.params_float = self.param_json_skill_exp(params)
            else:
                params = [1.1, 0.35, -0.2, 1.3, 0.5, 1.3]
                req.params_float = self.param_obstacle_avoidance_exp(params)
            # print ("req.ee_pose",req.params_float)
            # print ("param_Val",self.params["Gripper"].value)
            req.ee_pose = self.set_ee_pose(req, params_ee_start, 0.0)
            res = self.ac(req)
            req.type = req.TICK_TASK
            i = 0
            num_ticks = 20
            time = 0.0
            diff = 0.0
            while i < num_ticks:
                # print "Passed:", (rospy.Time.now() - self.start).to_sec()
                # req.sim_time = rospy.Time.now().to_sec()
                # if i == 3:
                #     print (i)
                #     req.ee_pose = self.set_ee_pose(req, params_ee_approach)
                #     # req.ee_pose = self.get_ee_pose()
                #     # print ("params_ee_approach",params_ee_approach)
                #     # req.ee_pose.position.x = params_ee_approach[0]
                #     # req.ee_pose.position.y = params_ee_approach[1]
                #     # req.ee_pose.position.z = params_ee_approach[2]
                #     # req.ee_pose.orientation.w = params_ee_approach[3]
                #     # req.ee_pose.orientation.x = params_ee_approach[4]
                #     # req.ee_pose.orientation.y = params_ee_approach[5]
                #     # req.ee_pose.orientation.z = params_ee_approach[6]
                #     print ("req.ee_pose",req.ee_pose)
                # if i == 7:
                #     print (i)
                #     # req.ee_pose = self.get_ee_pose()
                #     # print ("params_ee_end",params_ee_end)
                #     # req.ee_pose.position.x = params_ee_end[0]
                #     # req.ee_pose.position.y = params_ee_end[1]
                #     # req.ee_pose.position.z = params_ee_end[2]
                #     # req.ee_pose.orientation.w = params_ee_end[3]
                #     # req.ee_pose.orientation.x = params_ee_end[4]
                #     # req.ee_pose.orientation.y = params_ee_end[5]
                #     # req.ee_pose.orientation.z = params_ee_end[6] 
                #     req.ee_pose = self.set_ee_pose(req, params_ee_end)
                #     print ("req.ee_pose",req.ee_pose)
                if i > 13:
                    diff = 0.1 * 13
                print ("tick number: ",i)
                req.ee_pose = self.set_ee_pose(req, params_ee_start, diff)
                res = self.ac(req)
                i += 1
                diff +=0.1
                total_ticks += 1
                time += 0.1
                req.sim_time = time
                print ("current_pose",req.ee_pose)
                rospy.sleep(1)
                rospy.loginfo_throttle(10, "Did %i ticks in %f s. %f ms per tick", total_ticks, (rospy.Time.now() - self.start).to_sec(), (rospy.Time.now() - self.start).to_nsec() / (10**6 * float(total_ticks)))
            # time_diff = rospy.Time.now() - self.start
            # print num_ticks, "ticks took", (time_diff).to_sec(), "s"
            # print time_diff.to_nsec() / (10**6 * float(num_ticks)), "millisec per tick"
            req.type = req.STOP_TASK
            res = self.ac(req)
            # print "Task finished with", self.SkillStateDict[res.bt_response]
            j += 1


if __name__ == '__main__':
    # Allows to exit upon ctrl-c
    def signal_handler(signal, frame):
        print("\nProgram exiting gracefully")
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("skiros_manual_execution", anonymous=True)
    nr = 1
    if len(sys.argv) > 1:
        nr = sys.argv[1]
    ex = SkirosManualExecution(nr)
    ex.run()
