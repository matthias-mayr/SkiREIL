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
import numpy as np
from pyquaternion import Quaternion

#################################################################################
# GoToLinear Description and go_to_linear primitive
#################################################################################

class GoToLinearThreshold(SkillDescription):
    def createDescription(self):
        self.addParam("ThresholdPose", Element("skiros:TransformationPose"), ParamTypes.Required)  # GoalPose from GoToLinaear skill
        self.addParam("ee_pose", float, ParamTypes.Optional)

class go_to_linear_threshold(PrimitiveBase):
    def createDescription(self):
        self.setDescription(GoToLinearThreshold(), self.__class__.__name__)

    def onPreempt(self):
        return self.success("ThresholdPose achieved")

    def onInit(self):
        return True

    def onStart(self):
        self.threshold = self.params["ThresholdPose"].value.getData(":Position") + self.params["ThresholdPose"].value.getData(":Orientation")
        self.threshold_pos = self.params["ThresholdPose"].value.getData(":Position")
        self.threshold_ori = self.params["ThresholdPose"].value.getData(":Orientation")
        return True

    def execute(self):
        ee_pose = self.params["ee_pose"].values
        euc_dist_pos = np.sqrt((ee_pose[0] - self.threshold[0])**2 + (ee_pose[1] - self.threshold[1])**2 + (ee_pose[2] - self.threshold[2])**2)
        # euc_dist_ori = np.sqrt((ee_pose[3] - self.threshold[3])**2 + (ee_pose[4] - self.threshold[4])**2 + (ee_pose[5] - self.threshold[5])**2 + (ee_pose[6] - self.threshold[6])**2)
        euc_dist_ori = Quaternion.distance(Quaternion(self.threshold_ori),Quaternion(ee_pose[3:]))
        reason = "Position Distance from threshold: "
        reason += str(euc_dist_pos) 
        reason += " Angular Distance from threshold: "
        reason += str(euc_dist_ori) 
        if euc_dist_pos <= 0.2 or euc_dist_ori <= 0.2:
            return self.success(reason)
        else:
            return self.step(reason)