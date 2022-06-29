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
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from numpy.linalg import norm
from numpy import array
from pyquaternion import Quaternion

class EEPoseThreshold(SkillDescription):
    def createDescription(self):
        self.addParam("threshold", float, ParamTypes.Required)
        self.addParam("below_is_true", False, ParamTypes.Optional)
        self.addParam("ee_pose", float, ParamTypes.Optional)


class EEPositionDistance(SkillDescription):
    def createDescription(self):
        self.addParam("threshold", float, ParamTypes.Required)
        self.addParam("target", float, ParamTypes.Optional)
        self.addParam("above_is_true", False, ParamTypes.Optional)
        self.addParam("ee_pose", float, ParamTypes.Optional)

class EEPoseDistance(SkillDescription):
    def createDescription(self):
        self.addParam("threshold", float, ParamTypes.Required)
        self.addParam("target_pose", float, ParamTypes.Optional)
        self.addParam("above_is_true", False, ParamTypes.Optional)
        self.addParam("ee_pose", float, ParamTypes.Optional)
        
class MGSetGoalPose(SkillDescription):
    def createDescription(self):
        self.addParam("target_position", float, ParamTypes.Required)
        self.addParam("target_orientation", float, ParamTypes.Optional)
        self.addParam("mg_goal", float, ParamTypes.Optional)
        self.addParam("succeed", True, ParamTypes.Optional)


class ee_pose_threshold_y(PrimitiveBase):
    def createDescription(self):
        self.setDescription(EEPoseThreshold(), self.__class__.__name__)

    def onPreempt(self):
        return self.fail("Stopped", -2)

    def onInit(self):
        return True

    def onStart(self):
        self.threshold = self.params["threshold"].value
        self.below_is_true = self.params["below_is_true"].value
        return True

    def execute(self):
        ee_pose = self.params["ee_pose"].values
        reason = "Value y: "
        reason += str(ee_pose[1])
        reason += " Threshold: "
        reason += str(self.threshold)
        if self.below_is_true:
            if ee_pose[1] < self.threshold:
                return self.fail(reason, -1)
            else:
                return self.success(reason)
        else:
            if not ee_pose[1] < self.threshold:
                return self.fail(reason, -1)
            else:
                return self.success(reason)


class ee_pose_threshold_z(PrimitiveBase):
    def createDescription(self):
        self.setDescription(EEPoseThreshold(), self.__class__.__name__)

    def onPreempt(self):
        return self.fail("Stopped", -2)

    def onInit(self):
        return True

    def onStart(self):
        self.threshold = self.params["threshold"].value
        self.below_is_true = self.params["below_is_true"].value
        return True

    def execute(self):
        ee_pose = self.params["ee_pose"].values
        reason = "Value z: "
        reason += str(ee_pose[2])
        reason += " Threshold: "
        reason += str(self.threshold)
        if self.below_is_true:
            if ee_pose[2] < self.threshold:
                return self.fail(reason, -1)
            else:
                return self.success(reason)
        else:
            if not ee_pose[2] < self.threshold:
                return self.fail(reason, -1)
            else:
                return self.success(reason)
        return self.step(reason)


class ee_position_distance(PrimitiveBase):
    def createDescription(self):
        self.setDescription(EEPositionDistance(), self.__class__.__name__)

    def onStart(self):
        self.threshold = self.params["threshold"].value
        self.target = self.params["target"].values
        self.above_is_true = self.params["above_is_true"].value
        return True

    def execute(self):
        ee_lin = self.params["ee_pose"].values[0:3]
        diff = array(self.target) - array(ee_lin)
        diff = norm(diff)
        reason = "Difference: {} Threshold: {}".format(diff, self.threshold)
        if self.above_is_true:
            if diff > self.threshold:
                return self.success(reason)
        else:
            if diff < self.threshold:
                return self.success(reason)
        return self.step(reason)

class ee_pose_distance(PrimitiveBase):
    def createDescription(self):
        self.setDescription(EEPoseDistance(), self.__class__.__name__)

    def onStart(self):
        self.threshold = self.params["threshold"].value
        self.target_pos = self.params["target_pose"].values[0:3]
        self.target_ori = self.params["target_pose"].values[3:]
        # self.target = [0.0, 0.0, 0.1]
        self.above_is_true = self.params["above_is_true"].value
        return True

    def execute(self):
        #Todo: It happened that ee_pose was empty. Check for that.
        ee_lin = self.params["ee_pose"].values[0:3]
        ee_rot = self.params["ee_pose"].values[3:]
        diff_lin = array(self.target_pos) - array(ee_lin)
        diff_lin = norm(diff_lin)
        diff_ori = Quaternion.distance(Quaternion(self.target_ori),Quaternion(ee_rot))
        reason = "Linear_Difference: {} Angular_Difference: {} Threshold: {}".format(diff_lin, diff_ori, self.threshold)
        if self.above_is_true:
            # if diff_lin > self.threshold and diff_ori > self.threshold:
            if diff_lin > self.threshold:
                return self.success(reason)
        else:
            # if diff_lin < self.threshold and diff_ori < self.threshold:
            if diff_lin < self.threshold:
                return self.success(reason)
        return self.step(reason)

class mg_set_goal_position(PrimitiveBase):
    def createDescription(self):
        self.setDescription(MGSetGoalPose(), self.__class__.__name__)

    def onStart(self):
        self.succeed = self.params["succeed"].value
        self.target_pos = self.params["target_position"].values
        self.target_or = self.params["target_orientation"].values
        self.target = self.target_pos + self.target_or
        return True

    def execute(self):
        self.params["mg_goal"].values = self.target
        reason = "Set MG target to: {}".format(self.target)
        if (self.succeed):
            return self.success(reason)
        else:
            return self.step(reason)
