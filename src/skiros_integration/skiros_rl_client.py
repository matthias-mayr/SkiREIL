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

import time
import pickle
import sys
import signal
import rospy
import Queue
import textwrap

from skiros2_common.core.params import ParamHandler
from skiros2_common.core.world_element import Element
from skiros2_world_model.ros.world_model_interface import WorldModelInterface
from skiros2_msgs.msg import TreeProgress, SkillProgress
import skiros2_common.core.params as param
import skiros2_common.ros.utils as utils
from skiros2_skill.ros.utils import deserialize_skill
from skiros2_skill.core.skill import SkillDescription, SkillBase, SerialStar, ParallelFf, State, SkillWrapper

from control_msgs.msg import FollowJointTrajectoryGoal
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from skiros2_common.core.params import ParamTypes

from skireil.srv import SkirosNextAction, SkirosNextActionRequest, SkirosNextActionResponse

# Translates SkiROS skill states to strings
SkillStateDict = {0: 'unkown', SkillProgress.SUCCESS: 'success', SkillProgress.FAILURE: 'failure',
                  SkillProgress.RUNNING: 'running', SkillProgress.IDLE: 'idle'}

# Used when skill running trajectories
class ArmTrajectoryMotion():
    def __init__(self, id, plan, start_time):
        self.id = id
        self.plan = plan
        self.start_time = start_time

    def get_action(self, time):
        # Find timestep in trajectory
        i = 0
        while time > (self.start_time + self.plan.trajectory.points[i].time_from_start.to_sec()) \
                and i < (len(self.plan.trajectory.points) - 1):
                i += 1
        point = self.plan.trajectory.points[i]
        js = JointState()
        js.position = point.positions
        js.velocity = point.velocities
        return (js)

class BB_update():
    def __init__(self):
        self.mg_goal = list()
        self.stiffness = list()
        self.force = list()
        self.overlay = list()
        self.default_stiffness_flag = True
        self.default_force_flag = True
        self.default_overlay_flag = True

    # Registers a motion generator configuration
    def process_mg_msg(self, msg):
        params_des = utils.deserializeParamMap(msg.params)
        mg_values = list(params_des["mg_goal"].values)
        if len(mg_values) == 7:
            self.mg_goal.append(mg_values)
        else:
            rospy.logerr("Got faulty MG configuration. This should not happen.")

    def process_stiffness_msg(self, msg):
        params_des = utils.deserializeParamMap(msg.params)
        stiffness_values = list(params_des["EndEffectorStiffness"].values)
        if len(stiffness_values) == 6:
            self.stiffness.append(stiffness_values)
            self.default_stiffness_flag = False
        else:
            rospy.logerr("Got faulty Stiffness configuration. This should not happen.")

    def process_force_msg(self, msg):
        params_des = utils.deserializeParamMap(msg.params)
        force_values = list(params_des["EndEffectorForce"].values)
        if len(force_values) == 6:
            self.force.append(force_values)
            self.default_force_flag = False
        else:
            rospy.logerr("Got faulty Force configuration. This should not happen.")

    def process_overlay_msg(self, msg):
        params_des = utils.deserializeParamMap(msg.params)
        overlay_values = list(params_des["EndEffectorMotion"].values)
        # print ("overlay_values",overlay_values)
        if len(overlay_values) == 8:
            self.overlay.append(overlay_values)
            self.default_overlay_flag = False
        else:
            rospy.logerr("Got faulty Overlay Motion configuration. This should not happen.")

    # Postprocessing MG goals after parsing TreeProgress
    # After having parsed the tree, we need to extract the current goals
    def postprocessing_mg_goals(self, response):
        if len(self.mg_goal) > 1:
            rospy.logerr("More than one motion generator configuration. This should not happen. Got: %s", self.mg_goal)
        if len(self.mg_goal) > 0:
            if len(self.mg_goal[0]) == 7:
                response.ee_target_pose.position.x = self.mg_goal[0][0]
                response.ee_target_pose.position.y = self.mg_goal[0][1]
                response.ee_target_pose.position.z = self.mg_goal[0][2]
                response.ee_target_pose.orientation.x = self.mg_goal[0][3]
                response.ee_target_pose.orientation.y = self.mg_goal[0][4]
                response.ee_target_pose.orientation.z = self.mg_goal[0][5]
                response.ee_target_pose.orientation.w = self.mg_goal[0][6]
                response.calculate_ee_target_pose = False
                # Workaround until it is actually set by the black board
                # self.response.cart_stiffness.force.x = 200
                # self.response.cart_stiffness.force.y = 200
                # self.response.cart_stiffness.force.z = 200
                # self.response.cart_stiffness.torque.x = 20
                # self.response.cart_stiffness.torque.y = 20
                # self.response.cart_stiffness.torque.z = 20
                # Just for testing
                # self.response.overlay.dir.z = 1.0
                # self.response.overlay.path_velocity = 0.1
                # self.response.overlay.path_distance = 0.005
                # self.response.overlay.radius = 0.1
                # self.response.overlay.motion = "archimedes"
                # self.response.ee_target_wrench.force.z = -100
                # self.response.tool = 0
            else:
                rospy.logerr("Was about to set a faulty MG configuration. This should not happen.")

    def postprocessing_stiffness(self, response):
        # print ("postprocessing_stiffness called",response.overlay)
        if len(self.stiffness) > 1:
            rospy.logerr("More than one motion generator configuration. This should not happen. Got: %s", self.stiffness)
        if len(self.stiffness) > 0:
            if len(self.stiffness[0]) == 6:
                response.cart_stiffness.force.x = self.stiffness[0][0]
                response.cart_stiffness.force.y = self.stiffness[0][1]
                response.cart_stiffness.force.z = self.stiffness[0][2]
                response.cart_stiffness.torque.x = self.stiffness[0][3]
                response.cart_stiffness.torque.y = self.stiffness[0][4]
                response.cart_stiffness.torque.z = self.stiffness[0][5]
            else:
                rospy.logerr("Was about to set a faulty Stiffness configuration. This should not happen.")

    def postprocessing_force(self, response):
        # print ("postprocessing_force called",response.overlay)
        if len(self.force) > 1:
            rospy.logerr("More than one motion generator configuration. This should not happen. Got: %s", self.force)
        if len(self.force) > 0:
            if len(self.force[0]) == 6:
                response.ee_target_wrench.force.x = self.force[0][0]
                response.ee_target_wrench.force.y = self.force[0][1]
                response.ee_target_wrench.force.z = self.force[0][2]
                response.ee_target_wrench.torque.x = self.force[0][3]
                response.ee_target_wrench.torque.y = self.force[0][4]
                response.ee_target_wrench.torque.z = self.force[0][5]
            else:
                rospy.logerr("Was about to set a faulty Force configuration. This should not happen.")

    def postprocessing_overlay(self, response):
        # print ("postprocessing_overlay called",response.overlay)
        if len(self.overlay) > 1:
            rospy.logerr("More than one motion generator configuration. This should not happen. Got: %s", self.force)
        if len(self.overlay) > 0:
            if len(self.overlay[0]) == 8:
                response.overlay.motion = "archimedes" if self.overlay[0][0]==1.0 else ""
                response.overlay.radius = self.overlay[0][1]
                response.overlay.path_distance = self.overlay[0][2]
                response.overlay.path_velocity = self.overlay[0][3]
                response.overlay.allow_decrease = True if self.overlay[0][4]==1.0 else False
                response.overlay.dir.x = self.overlay[0][5]
                response.overlay.dir.y = self.overlay[0][6]
                response.overlay.dir.z = self.overlay[0][7]
                # self.response.ee_target_wrench.force.z = -100
                # self.response.tool = 0
            else:
                rospy.logerr("Was about to set a faulty Overlay Motion configuration. This should not happen.")
    
    def postprocessing(self, response):
        self.postprocessing_mg_goals(response)
        if not self.default_stiffness_flag:
            self.postprocessing_stiffness(response)
        else:
            self.set_default_stiffness(response)
        if not self.default_force_flag:
            self.postprocessing_force(response)
        else:
            self.set_default_force(response)
        if not self.default_overlay_flag:
            self.postprocessing_overlay(response)
        else:
            self.set_default_overlay(response)

    def set_default_stiffness(self, response):
        response.cart_stiffness.force.x = 400.0
        response.cart_stiffness.force.y = 400.0
        response.cart_stiffness.force.z = 400.0
        response.cart_stiffness.torque.x = 50.0
        response.cart_stiffness.torque.y = 50.0
        response.cart_stiffness.torque.z = 50.0

    def set_default_force(self, response):
        response.ee_target_wrench.force.x = 0.0
        response.ee_target_wrench.force.y = 0.0
        response.ee_target_wrench.force.z = 0.0
        response.ee_target_wrench.torque.x = 0.0
        response.ee_target_wrench.torque.y = 0.0
        response.ee_target_wrench.torque.z = 0.0
        
    def set_default_overlay(self, response):
        response.overlay.motion = ""
        response.overlay.radius = 0.0
        response.overlay.path_distance = 0.0
        response.overlay.path_velocity = 0.0
        response.overlay.allow_decrease = False
        response.overlay.dir.x = 0.0
        response.overlay.dir.y = 0.0
        response.overlay.dir.z = 0.0
        
# The RL Client definition.
#
# Listens to the "next" calls from the algorithm and starts, stops and ticks skills in SkiROS.
class SkirosRlClient(object):
    def __init__(self, worker_number, feedback_queue, agent):
        self.agent = agent
        self.task_id = 0
        self.nr = worker_number
        self.skill_name = rospy.get_param("~skill_name", "json_skill")
        self.verbose = rospy.get_param("~verbose", False)

        # Activate trajectory version if needed
        self.handle_arm_trajectories = False
        if self.skill_name is "ostacle_avoidance_exp":
            self.handle_arm_trajectories = True
            self.arm_motions = {}

        self.monitor_queue = feedback_queue
        self.response = SkirosNextActionResponse()
        self.bb_params = ParamHandler()
        self.wmi = WorldModelInterface()
        self.last_time = -1
        # Get world model element for the arm
        # Change to :test_robot for integration test
        skiros_arm_name = "skiros:bh_arm_"
        skiros_arm_name += str(self.nr)
        self.arm = self.wmi.resolve_element(Element("rparts:ArmDevice", skiros_arm_name))
        if self.arm is None:
            rospy.logfatal("Could not resolve world model element for '%s'. Aborting.", skiros_arm_name)

        # Register functions to populate skills parameters
        self.skill_param_dict = {
            "obstacle_avoidance_exp": self.param_obstacle_avoidance_exp,
            "obstacle_avoidance_exp_mg": self.param_obstacle_avoidance_exp,
            "json_skill": self.param_json_skill_exp
        }
        # Change to /test_robot for integration test
        rospy.Service('/skiros_worker_' + str(self.nr), SkirosNextAction, self.next_callback)

    def shutdown(self):
        if self.agent:
            self.agent.shutdown()

    ##########################################################################################
    # Parameterization functions
    ##########################################################################################
    def param_obstacle_avoidance_exp(self, skill, params, skill_name):
        down_quat = [1.0, 0.0, 0.0, 0.0]
        print ("param_obstacle_avoidance_exp",params)

        skill.ph["Arm"].value = self.arm

        x_offset = -0.5
        skill.ph["mp1_position"].values = [x_offset, params["mp1_position_y"], params["mp1_position_z"]]
        skill.ph["mp1_orientation"].values = down_quat

        skill.ph["mp2_position"].values = [x_offset, params["mp2_position_y"], params["mp2_position_z"]]
        skill.ph["mp2_orientation"].values = down_quat
        skill.ph["mp2_threshold"].values = params["mp2_threshold"]

        skill.ph["mp3_position"].values = [-0.6, 0.0, 0.7]
        skill.ph["mp3_orientation"].values = down_quat
        skill.ph["mp3_threshold"].values = params["mp3_threshold"]

    def param_json_skill_exp(self, skill, params, skill_name):
        # print ("skill",skill)   # skill is a skillHolder object but in JsonSkill it is SkillWrapper Object
        # print ("skill.ph.getParamMap()",skill.ph.getParamMap())
        # print ("params",params,type(params))
        skill.ph["JsonString"].values = skill_name
        for p in params:
            self.bb_params.addParam(p, params[p], ParamTypes.Required)
        # skill.ph[p.key].values = p.value
        if self.verbose:
            print ("skill.ph.getParamMap()",self.bb_params.getParamMap())
    ##########################################################################################
    # Functions for starting, stopping, ticking skills in SkiROS
    ##########################################################################################

    # Update blackboard parameters with current simulation state
    def update_bb_params(self, robot_state, ee_pose, sim_time):
        ee_pose = [ee_pose.position.x, ee_pose.position.y, ee_pose.position.z,
                   ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z,
                   ee_pose.orientation.w]
        self.bb_params.addParam("j_pos", list(robot_state.position), param.ParamTypes.Optional)
        self.bb_params.addParam("ee_pose", ee_pose, param.ParamTypes.Optional)
        self.bb_params.addParam("time", sim_time, param.ParamTypes.Optional)

    # Central function for starting, stopping and ticking
    def next_callback(self, req):
        if self.verbose:
            rospy.loginfo("%i: Start of next callback. Sim Time: %s, Current BT state: %s", self.nr, str(req.sim_time), SkillStateDict[self.response.bt_response])
        if self.last_time > req.sim_time:
            rospy.logfatal("%i: Went back in time. Got %f, but last time was %f.", self.nr, req.sim_time, self.last_time)
        self.last_time = req.sim_time

        # Start a new task
        if req.type == SkirosNextActionRequest.START_TASK:
            rospy.loginfo("%i: Starting new task", self.nr)
            self.response.robot_state = req.robot_state
            self.response.calculate_ee_target_pose = False
            self.response.ee_target_pose = req.ee_pose
            skiros_skill_name = self.skill_name
            # print req.skill_name
            # Workaround for old skill
            if (req.skill_name == "obstacle_avoidance_exp_mg"):
                rospy.logwarn("Warning: Workaround for old skill. Not using 'json_skill'.")
                skiros_skill_name = "obstacle_avoidance_exp_mg"
            skill = self.agent.get_skill(skiros_skill_name)
            # Parameterize the skill with the ones that come from SkiREIL
            # Workaround to test the old setup:
            params = dict()
            for i in req.params_float:
                params[i.key] = i.value
            self.skill_param_dict[skiros_skill_name](skill, params, req.skill_name)
            self.update_bb_params(req.robot_state, req.ee_pose, req.sim_time)
            params_des = utils.serializeParamMap(self.bb_params)
            # print "Deserialized params: ", str(params_des), "\n"
            res = self.agent.tick_once_sync(skill_list=[skill], param_list=params_des)
            self.task_id = res.execution_id
            # We receive two feedback blocks.
            self.process_tree_progress(req.sim_time, res)

        # Tick an existing one
        elif req.type == SkirosNextActionRequest.TICK_TASK:
            # The BT has finished
            if self.response.bt_response != SkillProgress.RUNNING:
                if self.verbose:
                    rospy.loginfo("%i: Asked to tick, but skipping because nothing is running.", self.nr)
                return self.response
            if self.verbose:
                rospy.loginfo("%i: Ticking task", self.nr)
            # Update blackboard parameters
            self.update_bb_params(req.robot_state, req.ee_pose, req.sim_time)
            params_des = utils.serializeParamMap(self.bb_params)
            res = self.agent.tick_once_sync(self.task_id, param_list=params_des)
            # Wait for monitor messages and parse them for the response
            self.process_tree_progress(req.sim_time, res)

        # Stop a task
        elif req.type == SkirosNextActionRequest.STOP_TASK:
            rospy.loginfo("%i: Ending task %i", self.nr, self.task_id)
            self.last_time = -1
            # Always explicitly preempt task
            self.agent.preempt_one(self.task_id)
            # if self.response.bt_response == SkillProgress.RUNNING:
                # We are only expecting final feedback if the task was running
                # self.process_feedback(req.sim_time)
            if self.handle_arm_trajectories:
                self.arm_motions.clear()
        else:
            rospy.logwarn("Unkown request type: %i. Doing nothing.", req.type)
        if self.verbose:
            rospy.loginfo("%i: End of next callback. Current BT state: %s", self.nr, SkillStateDict[self.response.bt_response])
        return self.response


    ##########################################################################################
    # Functions for SkiROS feedback and processing
    ##########################################################################################

    # Waits for feedback from SkiROS
    def process_feedback(self, time):
        if self.verbose:
            rospy.loginfo("%i: Start of process_feedback", self.nr)
        try:
            tree_progress = self.monitor_queue.get(block=True, timeout=2)
        except Queue.Empty:
            rospy.logerr("%i: TreeProgress queue was empty for 2 seconds. Doing nothing.", self.nr)
            return
        self.process_tree_progress(time, tree_progress)

    # Processes the feedback from SkiROS.
    # - Gets the skill state
    # - Fetches motion goals and trajectories
    def process_tree_progress(self, time, progress):
        tree_progress = progress.progress
        if not progress.ok:
            rospy.logerr("%i: Tick was not ok.", self.nr)
        if self.verbose:
            self.print_treeprogress(tree_progress)
        # Handles trajectories - otherwise we use motion generator goals
        if self.handle_arm_trajectories:
            motions_to_remove = set(self.arm_motions.keys())
            if self.verbose:
                rospy.loginfo("%i: Running motions: %s", self.nr, str(self.arm_motions.keys()))
        # else:
            # mg_goal = list()
            # stiffness = list()
            # force = list()
            # overlay = list()
            # print ("New Object")
        # Feedback comes in a TreeProgress object with child messages for the individual skills
        bb_update = BB_update()
        if progress.execution_id == self.task_id:
            for msg in tree_progress.progress:
                # The root tells us about the state of the whole tree
                if msg.type == ":Root":
                    self.response.bt_response = msg.state

                if self.handle_arm_trajectories:
                    # find execute_trajectory blocks
                    if (msg.label == "execute_trajectory_fake" or msg.label == "execute_trajectory") and \
                            msg.state == SkillProgress.RUNNING:
                        self.process_arm_trajectory_msg(msg, motions_to_remove)
                else:
                    # Find latest goal positions. We first used RUNNING, but then decided to make mg_set_goal_pose succeed when it is triggered
                    if (msg.label == "mg_set_goal_position") and (msg.state == SkillProgress.RUNNING or msg.state == SkillProgress.SUCCESS):
                        bb_update.process_mg_msg(msg)
                        # print ("bb_update.mg_goal",bb_update.mg_goal)
                        # break
                    if (msg.label == "change_stiffness"):
                        if (msg.state == SkillProgress.RUNNING):
                            bb_update.process_stiffness_msg(msg)
                            # print ("bb_update.stiffness",bb_update.stiffness)
                            # break
                    if (msg.label == "apply_force"): 
                        if (msg.state == SkillProgress.RUNNING):
                            bb_update.process_force_msg(msg)
                            # print ("bb_update.force",bb_update.force)
                            # break
                    if (msg.label == "overlay_motion"):
                        if (msg.state == SkillProgress.RUNNING):
                            bb_update.process_overlay_msg(msg)
                            # print ("bb_update.overlay",bb_update.overlay)
                            # break
        else:
            rospy.logerr("%i: TreeProgress ID is not the same as task ID: %i vs %i. Skipping.", self.nr, progress.execution_id, self.task_id)
        # print ('flags stiffness, force ,overlay',bb_update.default_stiffness_flag,bb_update.default_force_flag,bb_update.default_overlay_flag)
        if self.handle_arm_trajectories:
            self.postprocessing_trajectories(motions_to_remove)
        else:
            bb_update.postprocessing(self.response)
        if self.verbose:
            rospy.loginfo("%i: End of process_feedback", self.nr)
    # Registers an arm trajectory
    def process_arm_trajectory_msg(self, msg, motions_to_remove):
        if msg.id not in self.arm_motions:
            rospy.loginfo("%i: New arm motion registered: %i", self.nr, msg.id)
            params_des = utils.deserializeParamMap(msg.params)
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = pickle.loads(params_des["Plan"].value).joint_trajectory
            if self.verbose:
                rospy.loginfo("%i: New trajectory:\n%s", self.nr, str(goal.trajectory))
            self.arm_motions[msg.id] = ArmTrajectoryMotion(
                msg.id, goal, self.bb_params.getParam("time").value)
        else:
            # Several "running" are received when the skill is started
            motions_to_remove.discard(msg.id)

    # Postprocessing trajectories after parsing TreeProgress
    # After having parsed the tree, old trajectories need to be ended.
    def postprocessing_trajectories(self, motions_to_remove):
        # Remove the arm motions that were not running
        for motion in motions_to_remove:
            self.arm_motions.pop(motion)
        # Get the action based on the time stamp
        if len(self.arm_motions) > 1:
            rospy.logerr("More than one arm motion. This should not happen. Skipping action!", self.nr)
        if len(self.arm_motions) == 1:
            self.response.robot_state = self.arm_motions.itervalues().next().get_action(time)
            self.response.calculate_ee_target_pose = True

    def print_treeprogress(self, tree_progress):
        # Debugging information
        for skillprogress in tree_progress.progress:
            int_l = "    "
            int_1 = 1 * int_l
            int_2 = 2 * int_l
            int_3 = 3 * int_l
            print_params = True

            rospy.loginfo("==> Feedback robot: %s task_id: %i id: %i",
                          skillprogress.robot, skillprogress.task_id, skillprogress.id)
            rospy.loginfo("%sType: %s Label: %s", int_1, skillprogress.type, skillprogress.label)
            # rospy.loginfo("%sParent ID: %i Parent Label: %s",
            #              int_1, skillprogress.parent_id, skillprogress.parent_label)

            rospy.loginfo("%sState: %s Progress Code: %i  Progress message: %s", int_1,
                          SkillStateDict[skillprogress.state], skillprogress.progress_code,
                          skillprogress.progress_message)

            params_des = utils.deserializeParamMap(skillprogress.params)
            wrapper = textwrap.TextWrapper()
            wrapper.initial_indent = int_2
            wrapper.subsequent_indent = int_2
            wrapper.width = 100
            if print_params:
                # print wrapper.fill(text=str(skillprogress.params))
                for key, value in params_des.items():
                    rospy.loginfo("%s%s : %s", int_2, key, value.printState())
                    if key == "Plan" and value.isSpecified():
                        goal = FollowJointTrajectoryGoal()
                        goal.trajectory = pickle.loads(params_des["Plan"].value).joint_trajectory
                        rospy.loginfo("%sContains a plan with %i points", int_3,
                                      len(goal.trajectory.points))
                # print str(skillprogress)
                # print str(type(skillprogress))


def main():
    # Allows to exit upon ctrl-c
    def signal_handler(signal, frame):
        print("\nProgram exiting gracefully")
        rl_client.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    print ("rl_client")
    rl_client = SkirosRlClient()
    rospy.spin()


if __name__ == "__main__":
    main()
