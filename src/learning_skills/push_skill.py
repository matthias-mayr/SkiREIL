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
from skiros2_skill.core.skill import SkillDescription, SkillBase, Serial, SerialStar, ParallelFf, ParallelFs, NoFail, Selector, Sequential
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class Push(SkillDescription):
    def createDescription(self):
        #=======Params=========
        # 1) Peg needs to be attached with the gripper all the times
        # 2) Add Start, Goal and Peg to the world model
        # self.addParam("GoalPose", Element("skiros:TransformationPose"), ParamTypes.Inferred)    # questionable. Might not be necessary if somehow a pose could be associated with Box
        self.addParam("ApproachPosePush", Element("skiros:TransformationPose"), ParamTypes.Inferred)
        self.addParam("FixedGoal", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("ObjectToBePushed", Element("skiros:Product"), ParamTypes.Required)
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Inferred)
        self.addParam("offset_x_start", float, ParamTypes.Required)
        self.addParam("offset_y_start", float, ParamTypes.Required)
        self.addParam("offset_x_goal", float, ParamTypes.Required)
        self.addParam("offset_y_goal", float, ParamTypes.Required)
        self.addParam("mg_goal", float, ParamTypes.Optional)
        self.addParam("ee_pose", float, ParamTypes.Optional)

        #=======PreConditions=========
        # self.addPreCondition(self.getPropCond("EmptyHanded", "skiros:ContainerState", "Gripper", "=", "Empty", True))
        # self.addPreCondition(self.getRelationCond("ObjectInContainer", "skiros:contain", "Container", "Object", True))
        # 1) The peg should be on top of the box (z-axis distance)
        # 2) Add PegAtLocation to the world model
        # self.addPreCondition(self.getRelationCond("Holding", "skiros:contain", "Gripper", "Object", True))
        self.addPreCondition(self.getRelationCond("NotObjectAtGoal", "skiros:at", "ObjectToBePushed", "FixedGoal", False))
        self.addPreCondition(self.getRelationCond("GripperInApproachPosePush", "skiros:at", "Gripper", "ApproachPosePush", True))
        self.addPreCondition(self.getRelationCond("ApproachPosePushToObject", "skiros:approachPose", "ApproachPosePush", "ObjectToBePushed", True))
        #=======HoldConditions=========
        # self.addHoldCondition(self.getRelationCond("Holding", "skiros:contain", "Gripper", "Object", True))
        #=======PostConditions=========
        # 1) The peg should be inside the box
        # self.addPostCondition(self.getRelationCond("PegAtLocation", "skiros:at", "Peg", "GoalLocation", True))
        # self.addPreCondition(self.getPropCond("EmptyHanded", "skiros:ContainerState", "Gripper", "=", "True", False))
        # self.addPostCondition(self.getRelationCond("Holding", "skiros:contain", "Gripper", "Object", True))
        self.addPostCondition(self.getRelationCond("ObjectAtGoal", "skiros:at", "ObjectToBePushed", "FixedGoal", True))
        self.addPostCondition(self.getRelationCond("NotGripperInApproachPosePush", "skiros:at", "Gripper", "ApproachPosePush", False))
        self.addPostCondition(self.getRelationCond("GripperInGoalPose", "skiros:at", "Gripper", "FixedGoal", True))
        # self.addPostCondition(self.getRelationCond("GoalPoseToBox", "skiros:approachPose", "GoalPose", "Box", True))

        #(skiros:at skiros:ObjectToBePushed-1 skiros:FixedGoalPose-1)
class push(SkillBase):
    def createDescription(self):
        self.setDescription(Push(), self.__class__.__name__)

    # def expand(self, skill):
    #     skill(
    #         self.skill("Wait", "wait", specify={"Duration": 1.0}),
    #         self.skill("WmMoveObject", "wm_move_object",
    #             remap={"StartLocation": "InitialLocation", "TargetLocation": "GoalLocation"}),
    #     )

    def expand(self, skill):

        skill.setProcessor(SerialStar())
        skill(
                # self.skill("ElementToBBPose", "", remap={"Element": "FixedGoal"}),
                # self.skill(ParallelFs())(
                #     self.skill("MGSetGoalPose", "", remap={"target_position": "Position", "target_orientation": "Orientation"}, specify={"succeed": False}),
                #     self.skill("ChangeStiffness", "", specify={"TransX": 200.0,"TransY": 200.0,"TransZ": 0.0, "RotX": 50.0,"RotY": 50.0,"RotZ": 50.0}),
                #     self.skill("ApplyForce", "", specify={"TransX": 0.0,"TransY": 0.0, "RotX": 0.0,"RotY": 0.0,"RotZ": 0.0}, remap={"TransZ":"Force"}),
                #     self.skill("EEPoseDistance", "", specify={"threshold":0.04},remap={"target_pose":"mg_goal"}),    # remove ee_pose for manual or rl-client
                # ),
                # self.skill("ElementToBBPose", "", remap={"Element": "ApproachPosePush"}),
                # self.skill("WmSetRelation", "wm_set_relation", remap={'Src': 'Gripper', 'Dst': 'ApproachPosePush'},specify={'Relation': 'skiros:at', 'RelationState': True}),
                self.skill(ParallelFs())(
                    self.skill("GoToLinearOffset", "", remap={"GoalPose": "ObjectToBePushed", "pos_offset_x": "offset_x_start", "pos_offset_y": "offset_y_start"}),
                    self.skill("ChangeStiffness", "", specify={"TransX": 700.0,"TransY": 700.0,"TransZ": 700.0, "RotX": 50.0,"RotY": 50.0,"RotZ": 50.0}),
                ),
                self.skill("ElementToBBPose", "", specify={"pos_offset_x": 0.0, "pos_offset_y": 0.0}, remap={"Element": "FixedGoal", "OutputPose" : "FixedGoalPose"}),
                self.skill(ParallelFs())(
                    self.skill("GoToLinearOffset", "", remap={"GoalPose": "FixedGoal", "pos_offset_x": "offset_x_goal", "pos_offset_y": "offset_y_goal"}),
                    self.skill("ChangeStiffness", "", specify={"TransX": 700.0,"TransY": 700.0,"TransZ": 700.0, "RotX": 50.0,"RotY": 50.0,"RotZ": 50.0}),
                    self.skill("ApplyForce", "", specify={"TransX": 0.0,"TransY": 0.0, "TransZ": -7.0,"RotX": 0.0,"RotY": 0.0,"RotZ": 0.0}),
                    # The following primitve can be used to prelimnary stop the execution when the end effector gets close to the goal
                    # The impact on learning is not known yet. Therefore it is turned off
                    # self.skill("EEPoseDistance", "", specify={"threshold":0.14}, remap={"target_pose": "FixedGoalPose"}),    # remove ee_pose for manual or rl-client
                ),
                # self.skill("WmSetRelation", "wm_set_relation",remap={'Src': 'Gripper', 'Dst': 'FixedGoal'},specify={'Relation': 'skiros:at', 'RelationState': True}),
                # self.skill("WmSetRelation", "wm_set_relation",remap={'Src': 'ObjectToBePushed', 'Dst': 'FixedGoal'},specify={'Relation': 'skiros:at', 'RelationState': True}),
            )
