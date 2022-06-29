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
from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, ParallelFf, ParallelFs, NoFail, Selector, Sequential
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes, ParamHandler
import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
import geometry_msgs.msg
from copy import deepcopy

#################################################################################
# GoToLinear Description and go_to_linear primitive
#################################################################################

class GoToLinear(SkillDescription):
    """
    @brief Move the end-effector from current pose to goal linearly
    Keeps the orientation constant only changes the position 
    """

    def createDescription(self):
        #=======Params=========
        self.addParam("StartPose", Element("skiros:TransformationPose"), ParamTypes.Inferred)
        self.addParam("GoalPose", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Inferred)
        self.addParam("ee_pose", float, ParamTypes.Optional)
        # #=======PreConditions=========
        # self.addPreCondition(self.getRelationCond("GripperAtStartPose", "skiros:at", "Gripper", "StartPose", True))
        # #=======PostConditions=========
        # self.addPostCondition(self.getRelationCond("NotGripperAtStartPose", "skiros:at", "Gripper", "StartPose", False))
        # self.addPostCondition(self.getRelationCond("GripperAtGoalPose", "skiros:at", "Gripper", "GoalPose", True))


class go_to_linear(SkillBase):
    def createDescription(self):
        self.setDescription(GoToLinear(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Serial())
        skill(
            self.skill("ElementToBBPose", "", remap={"Element":"GoalPose","Position":"goal_position", "Orientation":"goal_orientation"}),
            self.skill(ParallelFs())(
                self.skill("MGSetGoalPose", "", specify={"succeed":False}, remap={"target_position": "goal_position", "target_orientation": "goal_orientation"}),
                self.skill("EEPoseDistance", "", specify={"threshold":0.05},remap={"target_pose":"mg_goal"}),    # remove ee_pose for manual or rl-client
            ),
        )


class GoToLinearOffset(SkillDescription):
    """
    @brief Move the end-effector from current pose to goal linearly
    Keeps the orientation constant only changes the position 
    """

    def createDescription(self):
        #=======Params=========
        self.addParam("StartPose", Element("skiros:TransformationPose"), ParamTypes.Inferred)
        self.addParam("GoalPose", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Inferred)
        self.addParam("ee_pose", float, ParamTypes.Optional)
        self.addParam("pos_offset_x", float, ParamTypes.Optional)
        self.addParam("pos_offset_y", float, ParamTypes.Optional)
        self.addParam("pos_offset_z", float, ParamTypes.Optional)
        # #=======PreConditions=========
        # self.addPreCondition(self.getRelationCond("GripperAtStartPose", "skiros:at", "Gripper", "StartPose", True))
        # #=======PostConditions=========
        # self.addPostCondition(self.getRelationCond("NotGripperAtStartPose", "skiros:at", "Gripper", "StartPose", False))
        # self.addPostCondition(self.getRelationCond("GripperAtGoalPose", "skiros:at", "Gripper", "GoalPose", True))


class go_to_linear_offset(SkillBase):
    def createDescription(self):
        self.setDescription(GoToLinearOffset(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Serial())
        skill(
            self.skill("ElementToBBPose", "", remap={"Element":"GoalPose","Position":"goal_position", "Orientation":"goal_orientation"}),
            self.skill(ParallelFs())(
                self.skill("MGSetGoalPose", "", specify={"succeed":False}, remap={"target_position": "goal_position", "target_orientation": "goal_orientation"}),
                self.skill("EEPoseDistance", "", specify={"threshold":0.05},remap={"target_pose":"mg_goal"}),    # remove ee_pose for manual or rl-client
            ),
        )
