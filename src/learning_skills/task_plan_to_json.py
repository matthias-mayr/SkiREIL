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
from skiros2_skill.core.skill import SkillDescription, SkillBase, SerialStar, ParallelFf, State, SkillWrapper
from skiros2_skill.core.skill import SkillBase, Selector, Serial, State, ParallelFs
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_task.ros.task_manager_interface import TaskManagerInterface
from skiros2_skill.ros.utils import deserialize_skill
import skiros2_common.ros.utils as utils
from skiros2_skill.ros.utils import SkillHolder
from skiros2_common.core.params import ParamHandler
import skiros2_common.tools.logger as log
from skiros2_skill.ros.skill_manager import SkillManager
import skiros2_task.core.pddl_interface as pddl
import rospy
import skiros2_msgs.srv as srvs
import json
import os
import os.path

#################################################################################
# Skill Descriptions
#################################################################################

class TaskPlanJson(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Goal", str, ParamTypes.Required)
        self.addParam("Plan", str, ParamTypes.Optional)
        # self.specifyDefault("Goal","(skiros:contain skiros:LargeBox-80 skiros:Starter-145)")
        
class SkillToFile(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Plan", str, ParamTypes.Required)

class SceneToFile(SkillDescription):
    def createDescription(self):
        pass

# primitive skill here that give learnable parameters: separating the parameters and creating json file
class SeparateParameters(SkillDescription):
    def createDescription(self):
        self.addParam("Plan", str, ParamTypes.Required)
        self.addParam("LearnableParameters", dict, ParamTypes.Inferred)

class CreateLearningScenario(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Plan", str, ParamTypes.Required)
        self.addParam("LearnableParameters", dict, ParamTypes.Required)

class JsonSkill(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("JsonString", str, ParamTypes.Required)

class CompletePlanLearn(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Goal", str, ParamTypes.Required)
   
   #(skiros:at skiros:Starter-145 skiros:BoxWithHole-1)
#################################################################################
# TaskPlanJson
#################################################################################

class task_planning(SkillBase):
    """
    This skill gives a task plan for the specified goal condition and sets the Plan 
    parameter accordingly
    """
    def createDescription(self):
        self.setDescription(TaskPlanJson(), self.__class__.__name__)
        self._expand_on_start = True
        self._skill_to_expand = None

    def expand(self, skill):
        self._skill_to_expand = skill

    def onPreempt(self):
        self._tm.preempt()
        return self.fail("Preempted.", -100)

    def onStart(self):
        self._action_status = None
        self._action_msg = None
        self._tm = TaskManagerInterface()
        return self._tm.plan(self.params["Goal"].value, self._done_planning)

    def _done_planning(self, status, msg):
        self._action_status = msg.progress_code
        self._action_msg = msg.progress_message

    def _add_children(self, skill, children):
        string = ""
        for i in children:
            s = self.skill(i.type, i.name)
            s.specifyParamsDefault(i.ph)
            skill.addChild(s)
            string += "{}[{}](".format(s.label, s.params.printState())
            string += self._add_children(skill.last(), i.children)
            string += ")\n"
        return string

    def execute(self):
        if self._action_status is None:
            return self.step("Planning...")
        elif self._action_status==0 or self._action_status==1:
            return self.fail(self._action_msg, -101)
        elif self._action_status==2:
            return self.success(self._action_msg)
        elif self._skill_to_expand:
            self.params["Plan"].setValue(self._action_msg)
            return self.success("{}".format(self._action_msg))

#################################################################################
# SkillToFile
#################################################################################

class skill_to_file(PrimitiveBase):
    """This skill saves the plan to a file in json format"""
    def createDescription(self):
        self.setDescription(SkillToFile(), self.__class__.__name__)

    def execute(self):
        d = rospy.get_param('/wm/workspace_dir', "")
        folder_path = d.replace("owl", "owl/planned_skill_scenes/")
        if not os.path.isdir(folder_path):
            os.mkdir(folder_path)
        task = deserialize_skill(self.params["Plan"].value)
        print ("folder_path",folder_path)
        if os.path.isdir(folder_path):
            count = 1
            while True:
                if not os.path.isfile(folder_path+"peg_insertion_task_"+str(count)+".json"):
                    file = folder_path+"peg_insertion_task_"+str(count)+".json"
                    count = count + 1
                    break
                else:
                    count = count + 1
                    continue
        print ("file",file)
        with open(file, 'w') as f:
            f.write(task.toJson())
        return self.success("Done")

#################################################################################
# SceneToFile
#################################################################################

class scene_to_file(PrimitiveBase):
    """This skill saves the plan to a file in json format"""
    def createDescription(self):
        self.setDescription(SceneToFile(), self.__class__.__name__)

    def execute(self):
        d = rospy.get_param('/wm/workspace_dir', "")
        folder_path = d.replace("owl", "owl/planned_skill_scenes/")
        if os.path.isdir(folder_path):
            count = 1
            while True:
                if not os.path.isfile(folder_path+"peg_insertion_task_scene_"+str(count)+".turtle"):
                    self.wmi.save(filename="planned_skill_scenes/"+"peg_insertion_task_scene_"+str(count)+".turtle")
                    count = count + 1
                    break
                else:
                    count = count + 1
                    continue
        return self.success("Done")

#################################################################################
# SeparateParameters
#################################################################################

class separate_parameters(PrimitiveBase):
    """This skill specifies the learnable parameters for learning scenarios"""
    def createDescription(self):
        self.setDescription(SeparateParameters(), self.__class__.__name__)

    def execute(self):
        task = deserialize_skill(self.params["Plan"].value) 
        learnable_tasks = {}
        for skillholder_object in task.children:
            for key,value in skillholder_object.ph.items():
                if value._param_type == ParamTypes.Required and not skillholder_object.ph.getParamValues(key): # check condition for learnable parameters
                    learnable_tasks[key] = SkillHolder(skillholder_object.manager, skillholder_object.type, skillholder_object.name)
        self.params.specify("LearnableParameters",[learnable_tasks])
        return self.success("Done")

#################################################################################
# CreateLearningScenario
#################################################################################

class create_learning_scenario(PrimitiveBase):
    """This skill creates a json file for learning scenario."""
    def createDescription(self):
        self.setDescription(CreateLearningScenario(), self.__class__.__name__)

    def execute(self):
        skills_to_be_checked = self.params["LearnableParameters"].value.items()
        all_skills = self.wmi.get_individuals("skiros:Skill")
        all_parameters = self.wmi.get_individuals("skiros:Parameter")
        all_values = self.wmi.get_individuals("skiros:Value")
        ############################################################################
        # The following code retrieves the learnable parameters from the world model
        # and saves them in dictionary for later use
        ###################3333#####################################################
        print ("skills_to_be_checked",skills_to_be_checked)
        learn_param_dict = {}
        for key,value in skills_to_be_checked:
            for skill in all_skills:
                for parameter in all_parameters:
                    relations = self.wmi.get_relations(skill, "skiros:hasParam" , parameter)    # Get hasParam relations
                    if relations:
                        ele = self.wmi.get_element(relations[0]["dst"])
                        if str(ele) == ele._id+'-'+key and skill.split("-")[0] == value.type: # check condtion: parameter and type. Makes sure that the parameter belongs to the corresponding skill
                            learn_param_dict[key] = [{'src_skill':value.type}]  
                            # Populate the learnable parametes based on their type
                            # Real or Integer
                            if ele.getProperty("skiros:ParamDataType").value == "real" or ele.getProperty("skiros:ParamDataType").value == "integer":
                                learn_param_dict[key][0]['parameter_type'] = ele.getProperty("skiros:ParamDataType").value
                                learn_param_dict[key][0]['values'] = [float(ele.getProperty("skiros:LowerBound").value), float(ele.getProperty("skiros:UpperBound").value)]
                                learn_param_dict[key][0]['mean'] = ele.getProperty("skiros:Mean").value
                                learn_param_dict[key][0]['std'] = ele.getProperty("skiros:Std").value
                                learn_param_dict[key][0]['prior'] = ele.getProperty("skiros:Prior").value
                            # Ordinal or Categorical
                            else:
                                learn_param_dict[key][0]['values'] = []
                                for val in all_values:
                                    relations_val = self.wmi.get_relations(parameter, "skiros:hasValues" , val) # Get hasValues relations. This relation is important since ordinal type param can have a list of values and each value is treated part of world model
                                    if relations_val:
                                        ele_val = self.wmi.get_element(relations_val[0]["dst"]) 
                                        if str(ele_val) == ele_val._id+'-'+key and skill.split("-")[0] == value.type:   # Similar check condition as above
                                            # For categorical param type, you can have "True" or "False"
                                            if ele_val.getProperty("skiros:Value").value.isalpha():
                                                learn_param_dict[key][0]['values'].append(ele_val.getProperty("skiros:Value").value)
                                            # For ordinal param type, convert string to float values
                                            else:
                                                learn_param_dict[key][0]['values'].append(float(ele_val.getProperty("skiros:Value").value))
                                learn_param_dict[key][0]['parameter_type'] = ele.getProperty("skiros:ParamDataType").value
                                learn_param_dict[key][0]['mean'] = ele.getProperty("skiros:Mean").value
                                learn_param_dict[key][0]['std'] = ele.getProperty("skiros:Std").value
                                learn_param_dict[key][0]['prior'] = ele.getProperty("skiros:Prior").value
        # Final dictionary of learnable parameters
        print ("learn_param_dict",learn_param_dict)

        ##########################################################
        # The following code creates the learning config json file
        ##########################################################
        scenario = {}
        scenario["threads"] = 6
        scenario["max_fun_evals"] = 1000
        scenario["episode_length"] = 12
        scenario["evals_per_param_config"] = 1
        scenario["expected_reward_evals"] = 1
        scenario["learning_platform"] = "sim"
        scenario["scene"] = "peg"
        scenario["domain_randomization"] = True
        scenario["optimizer"] = "hypermapper"
        scenario["optimizer_config"] = {}
        scenario["optimizer_config"]["design_of_experiment"] = {}
        scenario["optimizer_config"]["design_of_experiment"]["doe_type"] = "random sampling"
        scenario["optimizer_config"]["design_of_experiment"]["number_of_samples"] = 60

        # Input Parameters
        scenario["optimizer_config"]["input_parameters"] = {}

        for key,value in learn_param_dict.items():
            x = {}
            x["parameter_type"] = value[0]["parameter_type"]
            x["values"] = value[0]["values"]
            # x["prior"] = value[0]["prior"]
            print ("x",x)
            scenario["optimizer_config"]["input_parameters"][key] = x

        # Models
        scenario["optimizer_config"]["models"] = {}
        scenario["optimizer_config"]["models"]["model"] = "gaussian_process"
        # Feasible Output 
        scenario["optimizer_config"]["feasible_output"] = {}
        scenario["optimizer_config"]["feasible_output"]["enable_feasible_predictor"] = False
        
        # # Rewards
        # scenario["rewards"] = {}
        # # Fixed Success Reward
        # scenario["rewards"]["FixedSuccessReward"] = {}
        # scenario["rewards"]["FixedSuccessReward"]["objective"] = "reward"
        # scenario["rewards"]["FixedSuccessReward"]["type"] = "FixedSuccessReward"
        # scenario["rewards"]["FixedSuccessReward"]["weight"] = 1.0
        # scenario["rewards"]["FixedSuccessReward"]["value"] = 8.0
        # scenario["rewards"]["FixedSuccessReward"]["per_time_step"] = True
        # # Goal Distance Translation Reward
        # scenario["rewards"]["GoalDistanceTranslationReward"] = {}
        # scenario["rewards"]["GoalDistanceTranslationReward"]["objective"] = "reward"
        # scenario["rewards"]["GoalDistanceTranslationReward"]["type"] = "GoalDistanceTranslationReward"
        # scenario["rewards"]["GoalDistanceTranslationReward"]["weight"] = 50.0
        # scenario["rewards"]["GoalDistanceTranslationReward"]["link_name"] = "peg"
        # scenario["rewards"]["GoalDistanceTranslationReward"]["width"] = 0.4
        # scenario["rewards"]["GoalDistanceTranslationReward"]["min_dist"] = 0.25
        # scenario["rewards"]["GoalDistanceTranslationReward"]["goal"] = {}
        # scenario["rewards"]["GoalDistanceTranslationReward"]["goal"]["x"] = -0.6
        # scenario["rewards"]["GoalDistanceTranslationReward"]["goal"]["y"] = 0.0
        # scenario["rewards"]["GoalDistanceTranslationReward"]["goal"]["z"] = 0.7
        # # Table Avoidance
        # scenario["rewards"]["table_avoidance"] = {}
        # scenario["rewards"]["table_avoidance"]["objective"] = "reward"
        # scenario["rewards"]["table_avoidance"]["type"] = "BoxAvoidanceReward"
        # scenario["rewards"]["table_avoidance"]["negative"] = True
        # scenario["rewards"]["table_avoidance"]["weight"] = 0.15
        # scenario["rewards"]["table_avoidance"]["link_name"] = "peg"
        # scenario["rewards"]["table_avoidance"]["width"] = 7.0
        # scenario["rewards"]["table_avoidance"]["min_dist"] = 0.03
        # scenario["rewards"]["table_avoidance"]["box"] = {}
        # scenario["rewards"]["table_avoidance"]["box"]["x_min"] = -0.93
        # scenario["rewards"]["table_avoidance"]["box"]["y_min"] = -0.7
        # scenario["rewards"]["table_avoidance"]["box"]["z_min"] = 0.0        
        # scenario["rewards"]["table_avoidance"]["box"]["x_max"] = -0.13
        # scenario["rewards"]["table_avoidance"]["box"]["y_max"] = 0.5
        # scenario["rewards"]["table_avoidance"]["box"]["z_max"] = 0.6 
        # # Linear Distance To Box
        # scenario["rewards"]["linear_distance_to_box"] = {}
        # scenario["rewards"]["linear_distance_to_box"]["objective"] = "reward"
        # scenario["rewards"]["linear_distance_to_box"]["type"] = "LinearDistanceToBoxReward"
        # scenario["rewards"]["linear_distance_to_box"]["negative"] = True
        # scenario["rewards"]["linear_distance_to_box"]["weight"] = 0.15
        # scenario["rewards"]["linear_distance_to_box"]["link_name"] = "peg"
        # scenario["rewards"]["linear_distance_to_box"]["width"] = 2.0
        # scenario["rewards"]["linear_distance_to_box"]["min_dist"] = 0.006
        # scenario["rewards"]["linear_distance_to_box"]["box"] = {}
        # scenario["rewards"]["linear_distance_to_box"]["box"]["x_min"] = -0.601
        # scenario["rewards"]["linear_distance_to_box"]["box"]["y_min"] = -0.001
        # scenario["rewards"]["linear_distance_to_box"]["box"]["z_min"] = 0.60        
        # scenario["rewards"]["linear_distance_to_box"]["box"]["x_max"] = -0.599
        # scenario["rewards"]["linear_distance_to_box"]["box"]["y_max"] = 0.001
        # scenario["rewards"]["linear_distance_to_box"]["box"]["z_max"] = 0.699 

        # robot
        scenario["robot"] = {}
        scenario["robot"]["setup_name"] = "bh_rss"
        scenario["robot"]["tool"] = "peg_5mm"
        scenario["robot"]["collisions"] = "mesh"

        # robot_init_states
        scenario["robot_init_states"] = {}
        scenario["robot_init_states"]["s0"] = [0.67, 0.28, -0.27, -1.40, 1.10, 1.49, -0.58]
        scenario["robot_init_states"]["s1"] = [0.70, 0.20, -0.27, -1.46, 1.09, 1.47, -0.56]
        scenario["robot_init_states"]["s2"] = [0.56, 0.08, -0.15, -1.72, 1.07, 1.41, -0.69]
        scenario["robot_init_states"]["s3"] = [0.50, 0.17, -0.15, -1.72, 1.09, 1.42, -0.80]
        scenario["robot_init_states"]["s4"] = [0.52, 0.26, -0.15, -1.55, 1.09, 1.44, -0.72]

        # Dumping the data in a json file
        d = rospy.get_param('/wm/workspace_dir', "")
        folder_path = d.replace("owl", "owl/planned_skill_scenes/")
        if os.path.isdir(folder_path):
            count = 1
            while True:
                if not os.path.isfile(folder_path+"peg_insertion_multi_reward_single_objective_"+str(count)+".json"):
                    file_single_objective = folder_path+"peg_insertion_multi_reward_single_objective_"+str(count)+".json"
                    file_multi_objective = folder_path+"peg_insertion_multi_reward_multi_objective_"+str(count)+".json"
                    scenario["application_name"] = "peg_insertion_task_"+str(count)
                    count = count + 1
                    break
                else:
                    count = count + 1
                    continue

        # For single objective
        with open(file_single_objective, "w") as scenario_file:
            json.dump(scenario, scenario_file, indent=4)
        # For multi objective
        # Changes
        # scenario["rewards"]["FixedSuccessReward"]["objective"] = "performance"
        # scenario["rewards"]["GoalDistanceTranslationReward"]["objective"] = "performance"
        # scenario["rewards"]["table_avoidance"]["objective"] = "safety"

        with open(file_multi_objective, "w") as scenario_file:
            json.dump(scenario, scenario_file, indent=4)        
        return self.success("Done")
        
#################################################################################
# JsonSkill
#################################################################################

class json_skill(SkillBase):
    """Definition of a json_skill"""
    def createDescription(self):
        self.setDescription(JsonSkill(), self.__class__.__name__)
        self._expand_on_start = True
        self._skill_to_expand = None
    
    def onStart(self):
        d = rospy.get_param('/wm/workspace_dir', "")
        folder_path = d.replace("owl", "owl/planned_skill_scenes/")
        self.file = folder_path + self.params["JsonString"].value + ".json"
        f = open(self.file, "r")
        self.contents = f.readlines()
        self._parameters = ParamHandler()
        return True

    def expand(self, skill):
        self._skill_to_expand = skill

    def _add_children(self, skill, children):
        string = ""
        for i in children:
            s = self.skill(i.type, i.name)
            s.specifyParamsDefault(i.ph)
            skill.addChild(s)
            string += "{}[{}](".format(s.label, s.params.printState())
            string += self._add_children(skill.last(), i.children)
            string += ")\n"
        return string
    
    def execute(self):
        if self._skill_to_expand is not None:
            task = deserialize_skill(self.contents[0])
            self._skill_to_expand.setProcessor(SerialStar())
            task_string = self._add_children(self._skill_to_expand, task.children)
            self._skill_to_expand = None
            return self.step("{}".format(task_string))
        else:
            super(json_skill, self).execute()

#################################################################################
# complete_plan_learn
#################################################################################

class complete_plan_learn(SkillBase):
    def createDescription(self):
        self.setDescription(CompletePlanLearn(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(ParallelFs()) #SerialStar, Selector, ParallelFf, ParallelFs
        skill(
                self.skill(SerialStar())(
                    self.skill("TaskPlanJson", ""),
                    self.skill("SkillToFile", ""),
                    self.skill("SceneToFile", ""),
                    self.skill("SeparateParameters", ""),
                    self.skill("CreateLearningScenario", "")
                )
            )
