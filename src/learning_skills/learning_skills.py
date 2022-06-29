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
from skiros2_skill.core.skill import SkillDescription, SkillBase, State
from skiros2_skill.core.skill import Selector, Serial, SerialStar, ParallelFs
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element


class ObstacleAvoidanceExp(SkillDescription):
    def createDescription(self):
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required)

        self.addParam("mp1_position", float, ParamTypes.Optional)
        self.addParam("mp1_orientation", float, ParamTypes.Optional)

        self.addParam("mp2_position", float, ParamTypes.Optional)
        self.addParam("mp2_orientation", float, ParamTypes.Optional)
        self.addParam("mp2_threshold", float, ParamTypes.Optional)

        self.addParam("mp3_position", float, ParamTypes.Required)
        self.addParam("mp3_orientation", float, ParamTypes.Required)
        self.addParam("mp3_threshold", float, ParamTypes.Required)


class obstacle_avoidance_exp(SkillBase):
    def createDescription(self):
        self.setDescription(ObstacleAvoidanceExp(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Serial())
        skill(
            self.skill(Selector())(
                self.skill(Serial())(
                    self.skill("EEPoseThreshold", "ee_pose_threshold_y", specify={"below_is_true": True}, remap={"threshold": "mp3_threshold"}),
                    self.skill("CartesianMovementParams", "goto_params", remap={"Position": "mp3_position", "Orientation": "mp3_orientation"})
                    # self.skill(SerialStar())(
                    #     self.skill("PlanCartesianParams", "plan_cartesian_fast_params", remap={"Position": "mp3_position", "Orientation": "mp3_orientation", "Plan": "plan_3"}),
                    #     self.skill("ExecuteTrajectory", "", remap={"Plan": "plan_3"})
                    # )
                ),
                self.skill(Serial())(
                    self.skill("EEPoseThreshold", "ee_pose_threshold_z", specify={"below_is_true": True}, remap={"threshold": "mp2_threshold"}),
                    self.skill("CartesianMovementParams", "goto_params", remap={"Position": "mp2_position", "Orientation": "mp2_orientation"})
                    # self.skill(SerialStar())(
                    #     self.skill("PlanCartesianParams", "plan_cartesian_fast_params", remap={"Position": "mp2_position", "Orientation": "mp2_orientation", "Plan": "plan_2"}),
                    #     self.skill("ExecuteTrajectory", "", remap={"Plan": "plan_2"})
                    # )
                ),
                self.skill("CartesianMovementParams", "goto_params", remap={"Position": "mp1_position", "Orientation": "mp1_orientation"})
                # self.skill(SerialStar())(
                #     self.skill("PlanCartesianParams", "plan_cartesian_fast_params", remap={"Position": "mp1_position", "Orientation": "mp1_orientation", "Plan": "plan_1"}),
                #     self.skill("ExecuteTrajectory", "", remap={"Plan": "plan_1"})
                # )
            ),
            self.skill("EEPositionDistance", "ee_position_distance", specify={"threshold": 0.05, "target": [-0.6, 0.0, 0.7]})
        )

class obstacle_avoidance_exp_mg(SkillBase):
    def createDescription(self):
        self.setDescription(ObstacleAvoidanceExp(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFs())
        skill(
            self.skill(Selector())(
                self.skill(Serial())(
                    self.skill("EEPoseThreshold", "ee_pose_threshold_y", specify={"below_is_true": False }, remap={"threshold": "mp3_threshold"}),
                    self.skill("MGSetGoalPose", "", remap={"target_position": "mp3_position", "target_orientation": "mp3_orientation"}, specify={"succeed": False}),
                ),
                self.skill(Serial())(
                    self.skill("EEPoseThreshold", "ee_pose_threshold_z", specify={"below_is_true": True}, remap={"threshold": "mp2_threshold"}),
                    self.skill("MGSetGoalPose", "", remap={"target_position": "mp2_position", "target_orientation": "mp2_orientation"}, specify={"succeed": False}),
                ),
                self.skill("MGSetGoalPose", "", remap={"target_position": "mp1_position", "target_orientation": "mp1_orientation"}, specify={"succeed": False}),
            ),
            self.skill("EEPositionDistance", "ee_position_distance", specify={"threshold": 0.05, "target": [-0.6, 0.0, 0.7]})
        )
