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
from numpy.linalg import norm
from numpy import array

#################################################################################
# ChangeStiffness Description and change_stiffness primitive
#################################################################################

class ChangeStiffness(SkillDescription):
    """
    @brief Change end effector stiffness.
    """

    def createDescription(self):
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Required)
        self.addParam("TransX", -1.0, ParamTypes.Optional)
        self.addParam("TransY", -1.0, ParamTypes.Optional)
        self.addParam("TransZ", -1.0, ParamTypes.Optional)
        self.addParam("RotX", -1.0, ParamTypes.Optional)
        self.addParam("RotY", -1.0, ParamTypes.Optional)
        self.addParam("RotZ", -1.0, ParamTypes.Optional)
        self.addParam("EndEffectorStiffness", float, ParamTypes.Inferred)

class change_stiffness(PrimitiveBase):
    def createDescription(self):
        self.setDescription(ChangeStiffness(), self.__class__.__name__)

    def onStart(self):
        msg = list()
        msg.append(self.params["TransX"].value)
        msg.append(self.params["TransY"].value)
        msg.append(self.params["TransZ"].value)
        msg.append(self.params["RotX"].value)
        msg.append(self.params["RotY"].value)
        msg.append(self.params["RotZ"].value)
        self.addParam("EndEffectorStiffness", msg, ParamTypes.Inferred)
        # print ("EndEffectorStiffness",self.params["EndEffectorStiffness"].values)
        self.start_time = rospy.Time.now()
        self.max_time = rospy.Duration(2.0)
        return True

    def onPreempt(self):
        return self.success("Stiffness changed.")

    def onEnd(self):
        self.params["TransX"].setValue(-1.0)
        self.params["TransY"].setValue(-1.0)
        self.params["TransZ"].setValue(-1.0)
        self.params["RotX"].setValue(-1.0)
        self.params["RotY"].setValue(-1.0)
        self.params["RotZ"].setValue(-1.0)
        # return self.success("success")
        return True
        
    def execute(self):
        return self.step("Changing stiffness.")

#################################################################################
# ApplyForce Description and apply_force primitive
#################################################################################

class ApplyForce(SkillDescription):
    """
    @brief Apply force with the end effector.
    """

    def createDescription(self):
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Required)
        self.addParam("TransX", float, ParamTypes.Optional)
        self.addParam("TransY", float, ParamTypes.Optional)
        self.addParam("TransZ", float, ParamTypes.Optional)
        self.addParam("RotX", 0.0, ParamTypes.Optional)
        self.addParam("RotY", 0.0, ParamTypes.Optional)
        self.addParam("RotZ", 0.0, ParamTypes.Optional)
        self.addParam("EndEffectorForce", float, ParamTypes.Inferred)

class apply_force(PrimitiveBase):
    def createDescription(self):
        self.setDescription(ApplyForce(), self.__class__.__name__)

    def onStart(self):
        msg = list()
        msg.append(self.params["TransX"].value)
        msg.append(self.params["TransY"].value)
        msg.append(self.params["TransZ"].value)
        msg.append(self.params["RotX"].value)
        msg.append(self.params["RotY"].value)
        msg.append(self.params["RotZ"].value)
        self.addParam("EndEffectorForce", msg, ParamTypes.Inferred)
        # print ("EndEffectorForce",self.params["EndEffectorForce"].values)

        self.start_time = rospy.Time.now()
        self.max_time = rospy.Duration(2.0)
        return True

    def onPreempt(self):
        return self.success("Force changed.")

    def onEnd(self):
        self.params["TransX"].setValue(0.0)
        self.params["TransY"].setValue(0.0)
        self.params["TransZ"].setValue(0.0)
        self.params["RotX"].setValue(0.0)
        self.params["RotY"].setValue(0.0)
        self.params["RotZ"].setValue(0.0)
        # return self.success("success")  
        return True

    def execute(self):
        return self.step("Changing force.")

#################################################################################
# OverlayMotion Description and overlay_motion primitive
#################################################################################

class OverlayMotion(SkillDescription):
    """
    @brief Apply Overlay motion with the end effector.
    """

    def createDescription(self):
        self.addParam("Motion", float, ParamTypes.Required) # 1.0 --> "archimedes" and 0.0 --> ""
        self.addParam("Radius", float, ParamTypes.Optional)
        self.addParam("PathDistance", float, ParamTypes.Optional)
        self.addParam("PathVelocity", float, ParamTypes.Optional)
        self.addParam("AllowDecrease", float,  ParamTypes.Optional) # 1.0 --> True and 0.0 --> False
        self.addParam("Dir",float, ParamTypes.Optional)
        self.addParam("EndEffectorMotion", float, ParamTypes.Inferred)

class overlay_motion(PrimitiveBase):
    def createDescription(self):
        self.setDescription(OverlayMotion(), self.__class__.__name__)

    def onStart(self):
        req = list()
        req.append(self.params['Motion'].value)
        req.append(self.params['Radius'].value)
        req.append(self.params['PathDistance'].value)
        req.append(self.params['PathVelocity'].value)
        # print(type(req[-1]))
        req.append(self.params['AllowDecrease'].value)
        dir = self.params["Dir"].values
        req.append(dir[0])
        req.append(dir[1])
        req.append(dir[2])
        self.addParam("EndEffectorMotion", req, ParamTypes.Inferred)
        # self.params['EndEffectorMotion'].values = req
        # print ("EndEffectorMotion",self.params['EndEffectorMotion'].values)
        self.start_time = rospy.Time.now()
        self.max_time = rospy.Duration(2.0)
        return True

    def onPreempt(self):
        return self.success("Overlay motion applied.")

    def onEnd(self):
        self.params["Motion"].setValue(0.0)
        # return self.success("success")  
        return True

    def execute(self):
        # if (rospy.Time.now() - self.start_time) > self.max_time:
        # return self.success("Assuming Overlay motion has changed")
        return self.step("Applying Overlay motion.")

#################################################################################
# ElementToBBPose Description and element_to_bb_pose primitive
#################################################################################

class ElementToBBPose(SkillDescription):
    """
    @brief Apply Overlay motion with the end effector.
    """

    def createDescription(self):
        self.addParam("Element", Element("skiros:Object"), ParamTypes.Required)
        self.addParam("Position", float, ParamTypes.Optional)
        self.addParam("Orientation", float, ParamTypes.Optional)
        self.addParam("OutputPose", float, ParamTypes.Optional)
        self.addParam("pos_offset_x", float, ParamTypes.Optional)
        self.addParam("pos_offset_y", float, ParamTypes.Optional)
        self.addParam("pos_offset_z", float, ParamTypes.Optional)


class element_to_bb_pose(PrimitiveBase):
    def createDescription(self):
        self.setDescription(ElementToBBPose(), self.__class__.__name__)

    def onStart(self):
        self.element_pos = self.params["Element"].value.getData(":Position")
        if (self.params["pos_offset_x"].value is not None):
            self.element_pos[0] += self.params["pos_offset_x"].value
        if (self.params["pos_offset_y"].value is not None):
            self.element_pos[1] += self.params["pos_offset_y"].value
        if (self.params["pos_offset_z"].value is not None):
            self.element_pos[2] += self.params["pos_offset_z"].value
        self.element_ori = self.params["Element"].value.getData(":Orientation")  # Gets X Y Z W order
        return True

    def execute(self):
        self.params["Position"].setValues(self.element_pos)
        self.params["Orientation"].setValues(self.element_ori)
        self.params["OutputPose"].setValues(self.element_pos + self.element_ori)
        return self.success("Pose set relative to the Element")
